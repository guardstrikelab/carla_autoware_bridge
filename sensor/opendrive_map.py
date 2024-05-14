#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2024 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""


from utils.global_args import GlobalArgs

from std_msgs.msg import Header, String


class HDMap:
    def __init__(self, ego_vehicle, node):
        self.node = node
        self.log = GlobalArgs.log
        self.ego_vehicle = ego_vehicle

        self.map_file_publisher = self.node.create_publisher(String, '/carla/map_file')

    def update(self):
        world = self.ego_vehicle.get_world()
        carla_map = world.get_map()

        data_msg = String()
        data_msg.data = carla_map.to_opendrive()
        self.map_file_publisher.publish(data_msg)
