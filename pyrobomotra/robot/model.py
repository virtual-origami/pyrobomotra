# The MIT License (MIT)

# Copyright (c) 2016 - 2021 Atsushi Sakai

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Original Code Snippet used from:
https://github.com/AtsushiSakai/PythonRobotics/blob/master/ArmNavigation/two_joint_arm_to_point_control/two_joint_arm_to_point_control.py

Inverse kinematics of a two-joint arm
Left-click the plot to set the goal position of the end effector

Authors: Daniel Ingram (daniel-s-ingram)
         Atsushi Sakai (@Atsushi_twi)
         Karthik <she@biba.uni-bremen.de>
         Shantanoo <des@biba.uni-bremen.de, shantanoo.desai@gmail.com>

Ref: [P. I. Corke, "Robotics, Vision & Control",
    Springer 2017, ISBN 978-3-319-54413-7 p102](https://link.springer.com/book/10.1007/978-3-642-20144-8)


Changelog:
    - update-1:Dec-06-2020 :Converted the Original function implementation to class based implementation
    - update-2:Mar-03-2021 :Apply Linting on File, remove unused imports and variables
    - update-3:Apr-04-2021 :Refactor Code usage to include `pub_sub` module + Documentation + add MIT License

"""
import asyncio
import json
import logging
import math
import sys
import time
import traceback
import ast
import numpy as np
from pyrobomotra.pub_sub.AMQP import PubSubAMQP

# logger for this file
logging.basicConfig(stream=sys.stdout, level=logging.DEBUG,
                    format='%(levelname)-8s [%(filename)s:%(lineno)d] %(message)s')
logger = logging.getLogger("Robot:Model")


class RobotArm2Tracker:
    """This class implements Robot Arm with 2 joint ARM
    """

    def __init__(self, event_loop, robot_info):
        """RobotArm2: Two-Joint Robotic Arm Model
        - event_loop: Python AsyncIO Eventloop
        - robot_info: Python Dictionary with configuration of the Robot
        """
        try:
            if robot_info is None:
                logger.error("Robot Information Cannot Be None")
                sys.exit(-1)

            self.id = robot_info["id"]
            self.sample_time = robot_info["motion"]["control"]["sample_rate"]
            self.length_shoulder_to_elbow = robot_info["arm"]["length"]["shoulder_to_elbow"]
            self.length_elbow_to_gripper = robot_info["arm"]["length"]["elbow_to_gripper"]
            self.base = np.array([robot_info["initial_position"]["base"]["x"],
                                  robot_info["initial_position"]["base"]["y"]])
            self.shoulder = np.array([robot_info["initial_position"]["base"]["x"],
                                      robot_info["initial_position"]["base"]["y"]])
            self.elbow = np.array([0, 0])
            self.wrist = np.array([0, 0])
            self.theta1 = 0.0
            self.theta2 = 0.0
            self.eventloop = event_loop
            self.amq_publishers = []
            self.amq_subscribers = []

            if robot_info["protocol"]["publishers"] is not None:
                for publisher in robot_info["protocol"]["publishers"]:
                    if publisher["type"] == "amq":
                        logger.debug('Setting Up AMQP Publisher for Robot')
                        self.amq_publishers.append(
                            PubSubAMQP(
                                eventloop=self.eventloop,
                                config_file=publisher,
                                binding_suffix=self.id
                            )
                        )
                    else:
                        logger.error("Provide protocol amq config")
                        raise AssertionError("Provide protocol amq config")

            if robot_info["protocol"]["subscribers"] is not None:
                for subscribers in robot_info["protocol"]["subscribers"]:
                    if subscribers["type"] == "amq":
                        logger.debug('Setting Up AMQP Subcriber for Robot')
                        self.amq_subscribers.append(
                            PubSubAMQP(
                                eventloop=self.eventloop,
                                config_file=subscribers,
                                binding_suffix=self.id,
                                app_callback=self.consume_telemetry_msgs
                            )
                        )
                    else:
                        logger.error("Provide protocol amq config")
                        raise AssertionError("Provide protocol amq config")

        except Exception as e:
            logger.error("Exception during creation of RobotArm2Tracker", e)
            sys.exit(-1)

    async def publish(self, exchange_name, msg):
        """publish: publish robotic arm movement data to Message Broker
        - msg: message content
        """
        for publisher in self.amq_publishers:
            if exchange_name == publisher.exchange_name:
                await publisher.publish(message_content=msg)
                logger.debug(msg)

    async def connect(self):
        """connect: connect to the Message Broker
        """
        for publisher in self.amq_publishers:
            await publisher.connect()

        for subscriber in self.amq_subscribers:
            await subscriber.connect(mode="subscriber")

    async def update(self):
        result = self.get_forward_kinematics()
        await self.publish(
            exchange_name="telemetry_exchange",
            msg=json.dumps(result).encode()
        )

    def get_forward_kinematics(self):
        """get_forward_kinematics: Forward Kinematics for the Robotic Arm"""
        self.elbow = self.shoulder + \
                     np.array(
                         [
                             self.length_shoulder_to_elbow * np.cos(self.theta1),
                             self.length_shoulder_to_elbow * np.sin(self.theta1)
                         ]
                     )

        self.wrist = self.elbow + \
                     np.array(
                         [
                             self.length_elbow_to_gripper * np.cos(self.theta1 + self.theta2),
                             self.length_elbow_to_gripper * np.sin(self.theta1 + self.theta2)
                         ]
                     )

        result = dict(
            shoulder=(self.shoulder[0], self.shoulder[1]),
            elbow=(self.elbow[0], self.elbow[1]),
            wrist=(self.wrist[0], self.wrist[1])
        )
        return result

    def __get_all_states__(self):
        logger.debug(vars(self))

    def consume_telemetry_msgs(self, exchange_name, binding_name, message_body):
        """
        Consume telemetry messages from the Subscriber
        :param exchange_name: Exchange name
        :param binding_name: Binding name
        :param message_body: message body
        :return: None
        """
        try:
            if "telemetry.robot" in binding_name:
                # extract robot id
                binding_delimited_array = binding_name.split(".")
                robot_id = binding_delimited_array[len(binding_delimited_array) - 1]
                message_body = ast.literal_eval(message_body.decode('utf-8'))
                msg_attributes = message_body.keys()
                if ("id" in msg_attributes) and \
                        ("shoulder" in msg_attributes) and \
                        ("theta1" in msg_attributes) and \
                        ("theta2" in msg_attributes) and \
                        ("base" in msg_attributes):
                    if robot_id == message_body["id"] and self.id == robot_id:
                        logger.debug(message_body)
                        self.base = [message_body["base"][0], message_body["base"][1]]
                        self.shoulder = [message_body["shoulder"][0], message_body["shoulder"][1]]
                        self.theta1 = message_body["theta1"]
                        self.theta2 = message_body["theta2"]
                    else:
                        return False  # robot id in binding name and message body does not match
                else:
                    return False  # invalid message body format

        except AssertionError as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()
        except Exception as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()
