#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2024 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""

import copy
import numpy as np

from utils.global_args import GlobalArgs
from std_msgs.msg import Header, String
from sensor_msgs_py.point_cloud2 import create_cloud
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, NavSatStatus, CameraInfo, Range, PointField, Imu


class Lidar:
    def __init__(self,  actor, ego_vehicle, node):
        self.node = node
        self.actor = actor
        self.log = GlobalArgs.log
        self.ego_vehicle = ego_vehicle

        self.lidar_publisher = self.node.create_publisher(PointCloud2, '/carla_pointcloud')
        self.actor.listen(self._sensor_data_updated)

    def _sensor_data_updated(self, carla_lidar_measurement):
        # lidar_data = np.fromstring(bytes(carla_lidar_measurement.raw_data), dtype=np.float32)
        # lidar_data = np.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))
        # we take the opposite of y-axis
        # (as lidar point are express in left-handed coordinate system, and ros need right-handed)
        # lidar_data[:, 1] *= -1

        points = np.frombuffer(carla_lidar_measurement.raw_data, dtype=np.dtype('f4'))
        points = copy.deepcopy(points)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.frombuffer(points, dtype=np.float32)
        lidar_data = np.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))
        lidar_data = lidar_data[..., [1, 0, 2, 3]]

        fields = [PointField(name='x', offset=0,
                             datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4,
                             datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8,
                             datatype=PointField.FLOAT32, count=1),
                  PointField(name='intensity', offset=12,
                             datatype=PointField.FLOAT32, count=1)]
        header = Header(stamp=self.node.get_timestamp(), frame_id='velodyne_top')
        msg = create_cloud(header, fields, lidar_data)

        self.lidar_publisher.publish(msg)

