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
import time
import yaml
import carla
import pathlib

from sensor.imu import IMU
from sensor.gnss import GNSS
from sensor.lidar import Lidar
from sensor.camera import Camera
from sensor.opendrive_map import HDMap

from utils.ros_node import RosNode

from utils.logurus import init_log
from actor.ego_vehicle import EgoVehicle
from utils.global_args import GlobalArgs
from rosgraph_msgs.msg import Clock


class ProcessTask(RosNode):

    def __init__(self, **kwargs):
        super().__init__("universe_bridge")
        self.log = kwargs.get("log")
        self.cfg = kwargs.get("cfg")

        self.ego_vehicle = None
        self.init_position = {'x': '119.267', 'y': '330.394', 'z': '0', 'r': '0', 'p': '0',  'h': '0'}
        self.dst_position = {'x': '307.291', 'y': '330.394', 'z': '0', 'r': '0', 'p': '0',  'h': '0'}

        # carla info
        self.carla_host = self.cfg["host"]
        self.carla_port = self.cfg["carla_port"]
        self.carla_timeout = self.cfg["carla_timeout"]
        self.client = carla.Client(host=self.carla_host, port=self.carla_port)
        self.client.set_timeout(self.carla_timeout)
        self.world = self.client.get_world()

        self.clock_publisher = self.create_publisher(Clock, "/clock")

    def run(self):
        self.log.info("run task")
        # Wait for Carla until the ego vehicle spawned
        self._find_ego_vehicle()

        self._update_actor_factor()

    def _find_ego_vehicle(self, timeout=60):
        """
        Wait for the ego vehicle to be generated, and then obtain the ego vehicle.
        """
        start_time = time.time()
        while self.ego_vehicle is None and time.time() - start_time < timeout:
            actor_list = self.world.get_actors()
            for actor in actor_list:
                if actor.attributes.get("role_name") == "ego_vehicle":
                    self.ego_vehicle = actor
                    break

        if self.ego_vehicle is None:
            raise TimeoutError(f"Finding ego vehicle timeout in {timeout} seconds.")
        else:
            self.log.info(f"ego_vehicle id: {self.ego_vehicle.id}")

    def _find_gnss_actor(self):
        while True:
            actors = self.world.get_actors().filter("sensor.other.gnss")
            if len(actors) != 0:
                self.log.info("gnss sensor found")
                return actors[0]
            self.log.warning("not found gnss sensor")
            time.sleep(0.1)

    def _find_imu_actor(self):
        while True:
            actors = self.world.get_actors().filter("sensor.other.imu")
            if len(actors) != 0:
                self.log.info("imu sensor found")
                return actors[0]
            self.log.warning("not found imu sensor")
            time.sleep(0.1)

    def _find_lidar_actor(self):
        while True:
            actors = self.world.get_actors().filter("sensor.lidar.ray_cast")
            if len(actors) != 0:
                self.log.info("lidar sensor found")
                return actors[0]
            self.log.warning("not found lidar sensor")
            time.sleep(0.1)

    def _find_camera_actor(self):
        while True:
            actors = self.world.get_actors().filter("sensor.camera.rgb")
            if len(actors) != 0:
                self.log.info("camera sensor found")
                return actors[0]
            self.log.warning("not found camera sensor")
            time.sleep(0.1)

    def _update_actor_factor(self):
        # First find the sensor object from carla
        gnss_actor = self._find_gnss_actor()
        imu_actor = self._find_imu_actor()
        lidar_actor = self._find_lidar_actor()
        front_6mm_actor = self._find_camera_actor()

        GNSS(gnss_actor, self.ego_vehicle, self)
        IMU(imu_actor, self.ego_vehicle, self)
        Lidar(lidar_actor, self.ego_vehicle, self)
        Camera(front_6mm_actor, "camera", self)

        hd_map = HDMap(self.ego_vehicle, self)
        hd_map.update()

        ego_vehicle_obj = EgoVehicle(self.ego_vehicle, self)

        while True:
            time.sleep(0.05)
            obj_clock = Clock()
            obj_clock.clock = self.get_timestamp()
            self.clock_publisher.publish(obj_clock)

            ego_vehicle_obj.update()


def init_logger():
    root_dir = pathlib.Path(__file__).resolve().parent
    log_dir = str(root_dir.joinpath("log"))
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_file = os.path.join(log_dir, "universe_bridge.log")
    return init_log(log_file)


def init_cfg():
    config_file = os.path.dirname(os.path.abspath(__file__)) + "/config/config.yaml"
    with open(config_file, encoding="utf-8") as f:
        config = yaml.safe_load(f)["oasis"]
    return config


if __name__ == "__main__":
    log = init_logger()
    GlobalArgs(log)
    conf = init_cfg()

    task_args_dict = {
        "log": log,
        "cfg": conf,
        "carla_info": {"ip": "localhost", "port": 2000},
    }
    process_task = ProcessTask(**task_args_dict)
    process_task.run()

