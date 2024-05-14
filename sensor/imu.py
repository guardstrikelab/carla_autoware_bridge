#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2024 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""

import math
import numpy as np
from transforms3d.euler import euler2mat, quat2euler, euler2quat

from std_msgs.msg import Header, String
from sensor_msgs.msg import Imu

from utils.global_args import GlobalArgs


class IMU:
    def __init__(self, actor, ego_vehicle, node):
        self.actor = actor
        self.node = node
        self.log = GlobalArgs.log
        self.ego_vehicle = ego_vehicle

        self.imu_publisher = self.node.create_publisher(Imu, '/sensing/imu/tamagawa/imu_raw')
        self.actor.listen(self._sensor_data_updated)

    def _sensor_data_updated(self, carla_imu_measurement):
        data = np.array([
            carla_imu_measurement.accelerometer.x,
            carla_imu_measurement.accelerometer.y,
            carla_imu_measurement.accelerometer.z,
            carla_imu_measurement.gyroscope.x,
            carla_imu_measurement.gyroscope.y,
            carla_imu_measurement.gyroscope.z,
            carla_imu_measurement.compass,
            ], dtype=np.float64)

        imu_msg = Imu()
        imu_msg.header = Header(stamp=self.node.get_timestamp(), frame_id="tamagawa/imu_link")
        imu_msg.linear_acceleration.x = data[0]
        imu_msg.linear_acceleration.y = -data[1]
        imu_msg.linear_acceleration.z = data[2]

        imu_msg.angular_velocity.x = -data[3]
        imu_msg.angular_velocity.y = data[4]
        imu_msg.angular_velocity.z = -data[5]

        imu_rotation = data[6]

        quaternion = euler2quat(0, 0, -math.radians(imu_rotation))
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]

        self.imu_publisher.publish(imu_msg)
