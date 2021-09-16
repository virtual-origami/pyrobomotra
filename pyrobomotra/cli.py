"""Command-Line Utility Python Script to be installed as binary
   usage: robot-motion-tracker -c /path/to/config.yaml
"""

import argparse
import asyncio
import functools
import logging
import os
import signal
import sys
import yaml
from pyrobomotra.robot import RobotArm2Tracker


logging.basicConfig(level=logging.WARNING, format='%(levelname)-8s [%(filename)s:%(lineno)d] %(message)s')

# logger for this file
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/robomotra.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

# TURN OFF asyncio logger
asyncio_logger = logging.getLogger('asyncio')
asyncio_logger.setLevel(logging.WARNING)

wdt_logger = logging.getLogger('watchdog_timer')
wdt_logger.setLevel(logging.WARNING)

is_sighup_received = False

# Robots in Workspace
robots_in_ws = []


def _graceful_shutdown():
    global robots_in_ws
    for each_robot in robots_in_ws:
        del each_robot


def parse_arguments():
    """Arguments to run the script"""
    parser = argparse.ArgumentParser(description='Robotic Arm Motion Tracker')
    parser.add_argument('--config', '-c', required=True, help='YAML Configuration File for RobotMotionTra with path')
    parser.add_argument('--id', '-i', required=True, help='Provide robot id')
    return parser.parse_args()


def sighup_handler(name):
    """SIGHUP HANDLER"""
    # logger.debug(f'signal_handler {name}')
    logger.info('Updating the Robotic Configuration')
    global is_sighup_received
    is_sighup_received = True


async def app(eventloop, config, robot_id):
    """Main application for Robot Motion Tracker"""
    global robots_in_ws
    global is_sighup_received

    while True:
        # Read configuration
        try:
            robot_motion_tracker_config = read_config(config)
        except Exception as e:
            logger.error('Error while reading configuration:')
            logger.error(e)
            break

        logger.debug("Robot Motion Tracker Version: %s", robot_motion_tracker_config['version'])

        # robot instantiation
        robots_config = robot_motion_tracker_config["robots"]
        loop_interval = robot_motion_tracker_config["attributes"]["interval"]
        for robot_config in robots_config:
            # check for protocol key
            if "protocol" not in robot_config:
                logger.error("no 'protocol' key found.")
                sys.exit(-1)

            robo = RobotArm2Tracker(event_loop=eventloop, robot_info=robot_config, robot_id=robot_id)
            robots_in_ws.append(robo)
            await robo.connect()

        # continuously monitor signal handle and update robot motion
        while not is_sighup_received:
            for robo in robots_in_ws:
                await robo.update()
            await asyncio.sleep(loop_interval)

        # If SIGHUP Occurs, Delete the instances
        _graceful_shutdown()

        # reset sighup handler flag
        is_sighup_received = False


def read_config(yaml_config_file):
    """Parse the given Configuration File"""
    if os.path.exists(yaml_config_file):
        with open(yaml_config_file, 'r') as config_file:
            yaml_as_dict = yaml.load(config_file, Loader=yaml.FullLoader)
        return yaml_as_dict['robot_motion_tracker']
    else:
        logger.error('YAML Configuration File not Found.')
        raise FileNotFoundError


def main():
    """Initialization"""
    args = parse_arguments()
    if not os.path.isfile(args.config):
        logging.error("configuration file not readable. Check path to configuration file")
        sys.exit(-1)

    event_loop = asyncio.get_event_loop()
    event_loop.add_signal_handler(signal.SIGHUP, functools.partial(sighup_handler, name='SIGHUP'))
    try:
        event_loop.run_until_complete(app(eventloop=event_loop, config=args.config, robot_id=args.id))
    except KeyboardInterrupt:
        logger.error('CTRL+C Pressed')
        _graceful_shutdown()


if __name__ == "__main__":
    main()
