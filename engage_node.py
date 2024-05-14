#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2024 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""

import os
import math
import sys
import time

import yaml
import json
import rclpy
import pathlib

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, ClearRoute
from autoware_adapi_v1_msgs.msg import RouteState, MotionState, OperationModeState, LocalizationInitializationState
from rclpy.qos import QoSDurabilityPolicy, DurabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup

from transforms3d.euler import euler2quat

from utils.redis_util import RedisUtil
from utils.logurus import init_log


class EngageNode(Node):
    def __init__(self, log):
        super().__init__('engage_node')

        self.log = log
        self.task_id = None
        self.task_info = None
        self.task_status = None

        config_file = os.path.dirname(os.path.abspath(__file__)) + "/config/config.yaml"
        with open(config_file, encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)["oasis"]
        self.rds = RedisUtil(self.cfg["host"],
                             self.cfg["redis_port"],
                             self.cfg["redis_password"],
                             self.cfg["redis_db"]
                             )

        # self.goal_pub = self.create_publisher(
        #     PoseStamped,
        #     '/planning/mission_planning/goal',
        #     10)

        self.routing_state_sub = self.create_subscription(
            RouteState,
            '/api/routing/state',
            self.routing_state_callback,
            10
        )

        self.operation_mode_sub = self.create_subscription(
            OperationModeState,
            '/api/operation_mode/state',
            self.operation_mode_callback,
            10
        )

        self.localization_init_state_sub = self.create_subscription(
            LocalizationInitializationState,
            '/api/localization/initialization_state',
            self.localization_init_state_callback,
            qos_profile=QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )

        # self.localization_init_state_pub = self.create_publisher(
        #     LocalizationInitializationState,
        #     '/api/localization/initialization_state',
        #     qos_profile=QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # )

        # self.auto_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        # self.stop_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_stop')
        # self.clear_routing_client = self.create_client(ClearRoute, '/api/routing/clear_route')

        self.is_moving = False
        self.is_routing_success = False
        self.is_localization_initialized = False

        # self.is_moving = True
        # self.is_routing_success = True
        # self.is_localization_initialized = True

        # self.reset_universe_state()

        # 1. wait for the vehicle to be positioned successfully
        # 2. Send navigation target point
        # 3. Wait for successful navigation
        # 4. Switch to automatic mode

        # self.send_destination = self.create_timer(3.0, self.send_goal, callback_group=ReentrantCallbackGroup())
        self.set_auto_timer = self.create_timer(2.0, self.change_mode_to_auto, callback_group=ReentrantCallbackGroup())

        # self.set_stop_timer = self.create_timer(3.0, self.change_mode_to_stop, callback_group=ReentrantCallbackGroup())
        # self.clear_route_timer = self.create_timer(3.0, self.clear_route, callback_group=ReentrantCallbackGroup())
        # self.clear_localization_timer = self.create_timer(3.0, self.clear_localization, callback_group=ReentrantCallbackGroup())

        # self.send_destination = self.create_timer(3.0, self.send_goal)
        # self.set_auto_timer = self.create_timer(2.0, self.change_mode_to_auto)
        #
        # self.set_stop_timer = self.create_timer(3.0, self.change_mode_to_stop)
        # self.clear_route_timer = self.create_timer(2.0, self.clear_route)
        # self.clear_localization_timer = self.create_timer(3.0, self.clear_localization)

        self.log.info(f"engage node initialized")

    def reset_universe_state(self):
        self._srv_callback(ChangeOperationMode, self.stop_client)
        self.is_moving = False
        if self.is_routing_success:
            self._srv_callback(ClearRoute, self.clear_routing_client)
            self.is_routing_success = False

        if self.is_localization_initialized:
            localization_init_msg = LocalizationInitializationState()
            localization_init_msg.stamp = self.get_clock().now().to_msg()  
            localization_init_msg.state = LocalizationInitializationState.UNINITIALIZED
            self.localization_init_state_pub.publish(localization_init_msg)
            self.is_localization_initialized = False

    def localization_init_state_callback(self, data):
        localization_state_dict = {
            0: "UNKNOWN",
            1: "UNINITIALIZED",
            2: "INITIALIZING",
            3: "INITIALIZED",
        }

        self.log.info(f"localization state is {localization_state_dict[data.state]}")
        if data.state == 3:
            self.is_localization_initialized = True
            self.log.info(f"localization initialized")

    def routing_state_callback(self, data: RouteState):
        routing_state_dict = {
            0: "UNKNOWN",
            1: "UNSET",
            2: "SET",
            3: "ARRIVED",
            4: "CHANGING",
        }
        self.log.info(f"routing state is {routing_state_dict[data.state]}")
        if data.state not in [0, 1]:
            self.is_routing_success = True
            self.log.info(f"routing success, state is {routing_state_dict[data.state]}")

    def operation_mode_callback(self, data: OperationModeState):
        operation_mode_dict = {
            0: "UNKNOWN",
            1: "STOP",
            2: "AUTONOMOUS",
            3: "LOCAL",
            4: "REMOTE",
        }
        self.log.info(f"operation mode is {operation_mode_dict[data.mode]}")
        if data.mode == 2:
            self.is_moving = True
            self.log.info(f"vehicle is moving")

    def change_mode_to_auto(self):
        # self.log.info("change_mode_to_auto running")
        if self.is_routing_success and not self.is_moving:
            self.log.info('change mode to auto')
            # self._srv_callback(ChangeOperationMode, self.auto_client)
            # while not self.auto_client.wait_for_service(timeout_sec=1.0):
            #     self.log.info('service not available, waiting again...')
            # request = ChangeOperationMode.Request()
            # future = self.auto_client.call_async(request)
            # rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            # response = future.result()
            # self.log.info(f"response is {response}")
            # # self.log.info(f"response is {response.status.message}")
            # if response is not None:
            #     self.log.info(f'{ChangeOperationMode} result: {response.status.message}')
            # else:
            #     self.log.info('wait response timeout...')
            # time.sleep(2)
            set_auto_cmd = "ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode"
            os.system(set_auto_cmd)
            self.is_moving = True
            sys.exit(0)

    def change_mode_to_stop(self):
        self.log.info("change_mode_to_stop running")
        self._get_task_status()
        if self.task_status == "stop" and self.is_moving:
            self.log.info('change mode to stop')
            # self._srv_callback(ChangeOperationMode, self.stop_client)
            # time.sleep(2)
            set_stop_cmd = "ros2 service call /api/operation_mode/change_to_stop autoware_adapi_v1_msgs/srv/ChangeOperationMode"
            os.system(set_stop_cmd)
            self.is_moving = False


    def clear_route(self):
        self.log.info("clear_route running")
        self._get_task_status()
        if self.task_status == "stop" and self.is_routing_success:
            self.log.info('clear route')
            # self._srv_callback(ClearRoute, self.clear_routing_client)
            # time.sleep(2)
            clear_route_cmd = "ros2 service call /api/routing/clear_route autoware_adapi_v1_msgs/srv/ClearRoute"
            self.is_routing_success = False


    def clear_localization(self):
        self.log.info("clear_localization running")
        self._get_task_status()
        if self.task_status == "stop" and self.is_localization_initialized:
            self.log.info('clear localization')
            # time.sleep(2)
            localization_init_msg = LocalizationInitializationState()
            localization_init_msg.stamp = self.get_clock().now().to_msg()
            localization_init_msg.state = LocalizationInitializationState.UNINITIALIZED
            self.localization_init_state_pub.publish(localization_init_msg)
            self.is_localization_initialized = False

    def send_goal(self):
        self.log.info("==================send goal is running====================")
        self._get_task_info()
        if not self.is_localization_initialized or self.is_moving:
            self.log.info(f"do not send goal, because localization not init success")
            return

        goal = self.task_info["scenario_param"]["ego_start_and_end_position"]["end_position"]
        [x, y, z, roll, pitch, yaw] = [float(goal.get("x")), float(goal.get("y")), float(goal.get("z")),
                                       float(goal.get("r")), float(goal.get("p")), float(goal.get("h"))]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        eluer = [roll, pitch, yaw]
        new_eluer = list(map(math.radians, eluer))
        quat = euler2quat(*new_eluer)
        msg.pose.orientation.w = quat[0]
        msg.pose.orientation.x = quat[1]
        msg.pose.orientation.y = quat[2]
        msg.pose.orientation.z = quat[3]

        self.goal_pub.publish(msg)
        self.log.info(f"send goal: {goal}")

    def _srv_callback(self, srv_type, srv_client):
        while not srv_client.wait_for_service(timeout_sec=1.0):
            self.log.info('service not available, waiting again...')
        request = srv_type.Request()
        future = srv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        self.log.info(f"response is {response}")
        # self.log.info(f"response is {response.status.message}")
        if response is not None:
            self.log.info(f'{srv_type} result: {response.status.message}')
        else:
            self.log.info('wait response timeout...')

    def _get_task_info(self):
        if self.task_id is not None and self.task_info is not None:
            return

        task_status_dict = self.rds.get("task_status_dict")
        if task_status_dict is None:
            return
        task_status_dict = json.loads(task_status_dict)
        self.log.info(f"task_status_dict is {task_status_dict}")
        self.task_id = list(task_status_dict.keys())[0]
        self.log.info(f"self.task_id is {self.task_id}")

        task_infos = self.rds.hget("task_infos", self.task_id)
        self.task_info = json.loads(task_infos)["task"] if task_infos else None

    def _get_task_status(self):
        task_status_dict = self.rds.get("task_status_dict")
        if task_status_dict is None:
            return
        task_status_dict = json.loads(task_status_dict)
        self.task_status = list(task_status_dict.values())[0]
        self.log.info(f"self.task_status is {self.task_status}")


def main():
    root_dir = pathlib.Path(__file__).resolve().parent
    log_dir = str(root_dir.joinpath("log"))
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_file = os.path.join(log_dir, "engage.log")
    logger = init_log(log_file)

    rclpy.init()
    engage_obj = EngageNode(logger)
    try:
        rclpy.spin(engage_obj)
    finally:
        engage_obj.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
