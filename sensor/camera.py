#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2024 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""

import cv2
import math
import numpy as np

from utils.global_args import GlobalArgs
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, NavSatStatus, CameraInfo, Range, PointField, Imu


class Camera:
    """
    Sensor implementation details for cameras
    """
    def __init__(self, actor, name, node):
        self.node = node
        self.actor = actor
        self.log = GlobalArgs.log
        self.name = name

        self._camera_info = self._build_camera_info()
        self.camera_image_publisher = self.node.create_publisher(Image, "/sensing/camera/traffic_light/image_raw")
        self.camera_info_publisher = self.node.create_publisher(CameraInfo, "/sensing/camera/traffic_light/camera_info")

        self.actor.listen(self._sensor_data_updated)

    def _build_camera_info(self):
        """
        Private function to compute camera info

        camera info doesn't change over time
        """
        camera_info = CameraInfo()
        # store info without header
        camera_info.width = int(self.actor.attributes['image_size_x'])
        camera_info.height = int(self.actor.attributes['image_size_y'])
        camera_info.distortion_model = 'plumb_bob'
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (2.0 * math.tan(float(self.actor.attributes['fov']) * math.pi / 360.0))
        fy = fx
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return camera_info

    def _get_image_data_array(self, carla_camera_data):
        """
        Function to transform the received carla camera data into a numpy data array

        The RGB camera provides a 4-channel int8 color format (bgra).
        """
        if ((carla_camera_data.height != self._camera_info.height) or
                (carla_camera_data.width != self._camera_info.width)):
            self.log.error("Camera{} received image not matching configuration".format(self.actor))
        carla_image_data_array = np.ndarray(shape=(carla_camera_data.height, carla_camera_data.width, 4),
                                            dtype=np.uint8,
                                            buffer=carla_camera_data.raw_data)

        return carla_image_data_array

    def _sensor_data_updated(self, carla_camera_data):
        """
        Function (override) to transform the received carla camera data
        into a Cyber image message
        """

        image_data_array = self._get_image_data_array(carla_camera_data)

        msg = Image()
        frame_id = 'traffic_light_left_camera/camera_link'
        msg.header = Header(stamp=self.node.get_timestamp(), frame_id=frame_id)
        msg.data = cv2.imencode('.jpg', image_data_array)[1].tostring()

        cam_info = self._camera_info
        cam_info.header = msg.header
        self.camera_info_publisher.publish(cam_info)
        self.camera_image_publisher.publish(msg)
