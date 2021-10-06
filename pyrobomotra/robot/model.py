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
import queue
import sys
import time
import traceback
import ast
import numpy as np
from pyrobomotra.pub_sub import PubSubAMQP
from pyrobomotra.in_mem_db import RedisDB

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

            self.length_shoulder_to_elbow = 0.0
            self.length_elbow_to_gripper = 0.0
            self.base = [0.0, 0.0]
            self.shoulder = [0.0, 0.0]

            self.consume_telemetry_queue = queue.SimpleQueue()
            self.redis_db = RedisDB(host=robot_info["in_mem_db"]["server"]["address"],
                                    port=robot_info["in_mem_db"]["server"]["port"],
                                    password=robot_info["in_mem_db"]["credentials"]["password"])
            self.eventloop = event_loop
            self.publishers = []
            self.subscribers = []

            if robot_info["protocol"]["publishers"] is not None:
                for publisher in robot_info["protocol"]["publishers"]:
                    if publisher["type"] == "amq":
                        logger.debug('Setting Up AMQP Publisher for Robot')
                        self.publishers.append(
                            PubSubAMQP(
                                eventloop=self.eventloop,
                                config_file=publisher,
                                binding_suffix=""
                            )
                        )
                    else:
                        logger.error("Provide protocol amq config")
                        raise AssertionError("Provide protocol amq config")

            if robot_info["protocol"]["subscribers"] is not None:
                for subscribers in robot_info["protocol"]["subscribers"]:
                    if subscribers["type"] == "amq":
                        logger.debug('Setting Up AMQP Subcriber for Robot')
                        self.subscribers.append(
                            PubSubAMQP(
                                eventloop=self.eventloop,
                                config_file=subscribers,
                                binding_suffix="",
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
        for publisher in self.publishers:
            if exchange_name == publisher.exchange_name:
                await publisher.publish(message_content=msg)
                logger.debug(f'Pub: msg{msg}')

    async def connect(self):
        """connect: connect to the Message Broker
        """
        for publisher in self.publishers:
            await publisher.connect()

        for subscriber in self.subscribers:
            await subscriber.connect(mode="subscriber")

    def update_states(self, state_information):
        self.length_shoulder_to_elbow = float(state_information["length_shoulder_to_elbow"])
        self.length_elbow_to_gripper = float(state_information["length_elbow_to_gripper"])
        self.base = state_information["base"]
        self.shoulder = state_information["shoulder"]

    def get_states_from_db(self, robot_id):
        name = "robot_" + robot_id
        db_result = self.redis_db.get(key=name)
        if db_result is None:
            return None
        result = json.loads(db_result)
        return result

    def restore_states_in_db(self, robot_id):
        state_information = {
            "base": self.base,
            "shoulder": self.shoulder,
            "length_shoulder_to_elbow": self.length_shoulder_to_elbow,
            "length_elbow_to_gripper": self.length_elbow_to_gripper
        }
        name = "robot_" + robot_id
        json_state_information = json.dumps(state_information)
        self.redis_db.set(key=name, value=json_state_information)

    async def update(self):

        if not self.consume_telemetry_queue.empty():
            new_measurement = self.consume_telemetry_queue.get_nowait()

            new_measurement_id = new_measurement["id"]
            state_info = self.get_states_from_db(new_measurement_id)

            if state_info is not None and new_measurement_id is not None:
                self.update_states(state_information=state_info)
                kinematic_result = self.get_forward_kinematics(
                    theta1=new_measurement["theta1"],
                    theta2=new_measurement["theta2"]
                )
                result_rmt_robot = {
                    "id": new_measurement["id"],
                    "base": (self.base[0], self.base[1]),
                    "shoulder": (self.shoulder[0], self.shoulder[1]),
                    "timestamp": time.time_ns()
                }
                result_rmt_robot.update(kinematic_result)
                await self.publish(
                    exchange_name="rmt_robot",
                    msg=json.dumps(result_rmt_robot).encode())
                await self.publish(
                    exchange_name="visual",
                    msg=json.dumps(result_rmt_robot).encode())

    def get_forward_kinematics(self, theta1, theta2):
        """get_forward_kinematics: Forward Kinematics for the Robotic Arm"""
        elbow = self.shoulder + \
                np.array(
                    [
                        self.length_shoulder_to_elbow * np.cos(theta1),
                        self.length_shoulder_to_elbow * np.sin(theta1)
                    ]
                )

        wrist = elbow + \
                np.array(
                    [
                        self.length_elbow_to_gripper * np.cos(theta1 + theta2),
                        self.length_elbow_to_gripper * np.sin(theta1 + theta2)
                    ]
                )

        result = dict(
            elbow=(elbow[0], elbow[1]),
            wrist=(wrist[0], wrist[1])
        )
        return result

    def __get_all_states__(self):
        logger.debug(vars(self))

    async def robot_msg_handler(self, exchange_name, binding_name, message_body):

        msg_attributes = message_body.keys()
        if ("id" in msg_attributes) and \
                ("shoulder" in msg_attributes) and \
                ("theta1" in msg_attributes) and \
                ("theta2" in msg_attributes) and \
                ("base" in msg_attributes):
            logger.debug(f'sub: exchange {exchange_name}: msg {message_body}')
            self.consume_telemetry_queue.put_nowait(item=message_body)

    async def consume_telemetry_msgs(self, **kwargs):
        """
        Consume telemetry messages from the Subscriber
        :return: None
        """
        try:
            exchange_name = kwargs["exchange_name"]
            binding_name = kwargs["binding_name"]
            message_body = json.loads(kwargs["message_body"])

            # check for matching subscriber with exchange and binding name in all subscribers
            for subscriber in self.subscribers:
                # if subscriber.exchange_name == exchange_name:
                cb_str = subscriber.get_callback_handler_name()
                if cb_str is not None:
                    try:
                        cb = getattr(self, cb_str)
                    except:
                        logging.critical(f'No Matching handler found for {cb_str}')
                        continue
                    if cb is not None:
                        await cb(exchange_name=exchange_name, binding_name=binding_name, message_body=message_body)

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
