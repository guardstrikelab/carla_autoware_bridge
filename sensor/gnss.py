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
import transforms3d
from utils.global_args import GlobalArgs
from geometry_msgs.msg import Pose
from std_msgs.msg import Header, String
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GNSS:
    def __init__(self, actor, ego_vehicle, node):
        self.actor = actor
        self.node = node
        self.log = GlobalArgs.log
        self.ego_vehicle = ego_vehicle

        self.gnss_carla_publisher = self.node.create_publisher(NavSatFix, "/carla_nav_sat_fix")
        self.actor.listen(self._sensor_data_updated)

    def _sensor_data_updated(self, carla_gnss_measurement):
        data = np.array([
            carla_gnss_measurement.latitude,
            carla_gnss_measurement.longitude,
            carla_gnss_measurement.altitude], dtype=np.float64)

        msg = NavSatFix()
        msg.header = Header(stamp=self.node.get_timestamp(), frame_id='gnss_link')
        msg.latitude = data[0]
        msg.longitude = data[1]
        msg.altitude = data[2]

        msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        msg.status.service = (
                NavSatStatus.SERVICE_GPS |
                NavSatStatus.SERVICE_GLONASS |
                NavSatStatus.SERVICE_COMPASS |
                NavSatStatus.SERVICE_GALILEO
                )
        self.gnss_carla_publisher.publish(msg)


