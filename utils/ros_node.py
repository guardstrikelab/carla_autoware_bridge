#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2024 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, DurabilityPolicy, QoSProfile


class RosNode(object):

    def __init__(self, name, **kwargs):
        self.kwargs = kwargs
        self.rclpy = rclpy
        self.rclpy.init()
        self.node = Node(name)

    def get_timestamp(self):
        return self.node.get_clock().now().to_msg()

    def create_publisher(self, data_type, name):
        return self.node.create_publisher(data_type, name, 10)

    def create_subscription(self, data_type, name, callback):
        return self.node.create_subscription(data_type, name, callback, 10)

    def create_localization_publisher(self, data_type, name):
        return self.node.create_publisher(data_type, name,
                                          qos_profile=QoSProfile(depth=1,
                                                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def create_localization_subscription(self, data_type, name, callback):
        return self.node.create_subscription(data_type, name, callback,
                                             qos_profile=QoSProfile(depth=1,
                                                                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def create_rate(self, frequency):
        return self.node.create_rate(frequency)

    def create_timer(self, timer_period_sec, callback):
        return self.node.create_timer(timer_period_sec, callback)

    def create_service(self, req_type, name, callback):
        return self.node.create_service(req_type, name, callback)

    def create_client(self, req_type, name):
        return self.node.create_client(req_type, name)

    def get_logger(self):
        return self.node.get_logger()

    def destroy(self):
        self.node.destroy_node()

    def spin(self):
        self.rclpy.spin(self.node)

    def ok(self):
        return self.rclpy.ok()

    def shutdown(self):
        self.rclpy.try_shutdown()

    def spin_until_future_complete(self, future, timeout_sec=1.0):
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
