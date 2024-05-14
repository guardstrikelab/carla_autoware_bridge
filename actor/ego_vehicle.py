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
import carla
import threading
import transforms3d
import numpy as np
from transforms3d.euler import euler2quat


from std_msgs.msg import Header, String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance, TwistStamped, TwistWithCovarianceStamped
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport, SteeringReport, VelocityReport

from utils.global_args import GlobalArgs


class EgoVehicle:
    def __init__(self, ego_vehicle, node):
        self.node = node
        self.log = GlobalArgs.log
        self.ego_vehicle = ego_vehicle
        self.start_position = node.init_position
        self.dst_position = node.dst_position

        self.steering_factor = 0.45
        self.max_steer_angle = 0.7
        self.step_mode_possible = None
        self.forward_speed = 0
        self.current_control = carla.VehicleControl()

        self.vehicle_status_publisher = self.node.create_publisher(Odometry, '/odo')
        self.odo_pub = self.node.create_publisher(Odometry, "/localization/kinematic_state")

        self.auto_velocity_status_publisher = self.node.create_publisher(
            VelocityReport, '/vehicle/status/velocity_status')
        self.auto_steering_status_publisher = self.node.create_publisher(
            SteeringReport, '/vehicle/status/steering_status')
        self.auto_gear_status_publisher = self.node.create_publisher(
            GearReport, '/vehicle/status/gear_status')
        self.auto_control_mode_publisher = self.node.create_publisher(
            ControlModeReport, '/vehicle/status/control_mode')

        self.vehicle_control_subscriber = self.node.create_subscription(
            TwistStamped, '/carla_op_controller_cmd', self.on_vehicle_control)

        self.init_pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, "/initialpose3d")
        self.pub_init_pose()

        self.universe_control_callback()

        self.goal_pub = self.node.create_publisher(PoseStamped, '/planning/mission_planning/goal')
        self.pub_goal()

    def universe_control_callback(self):
        self.node.create_subscription(
            AckermannControlCommand, 'control/command/control_cmd', self.on_autoware_universe_vehicle_control
        )
        spin_thread = threading.Thread(target=self.node.spin)
        spin_thread.start()

    def pub_init_pose(self):
        init_pose = PoseWithCovarianceStamped()
        init_pose.header = Header(stamp=self.node.get_timestamp(), frame_id='map')
        [x, y, z, roll, pitch, yaw] = [float(self.start_position.get("x")),
                                       float(self.start_position.get("y")),
                                       float(self.start_position.get("z")),
                                       float(self.start_position.get("r")),
                                       float(self.start_position.get("p")),
                                       float(self.start_position.get("h"))]

        init_pose.pose.pose.position.x = x
        init_pose.pose.pose.position.y = -y
        init_pose.pose.pose.position.z = z
        eluer = [roll, pitch, yaw]
        new_eluer = list(map(math.radians, eluer))
        quat = euler2quat(*new_eluer)
        init_pose.pose.pose.orientation.w = quat[0]
        init_pose.pose.pose.orientation.x = quat[1]
        init_pose.pose.pose.orientation.y = quat[2]
        init_pose.pose.pose.orientation.z = quat[3]

        # init_pose.pose.covariance = [
        #     0.01,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.01,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.01,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.01,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.01,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.01,
        # ]
        self.log.info(f"init_pose is {init_pose}")
        self.init_pose_pub.publish(init_pose)

    def pub_goal(self):
        goal = PoseStamped()
        goal.header = Header(stamp=self.node.get_timestamp(), frame_id='map')
        [x, y, z, roll, pitch, yaw] = [float(self.dst_position.get("x")),
                                       float(self.dst_position.get("y")),
                                       float(self.dst_position.get("z")),
                                       float(self.dst_position.get("r")),
                                       float(self.dst_position.get("p")),
                                       float(self.dst_position.get("h"))]

        goal.pose.position.x = x
        goal.pose.position.y = -y
        goal.pose.position.z = z
        quaternion = euler2quat(math.radians(roll), math.radians(pitch), math.radians(yaw))
        goal.pose.orientation.w = quaternion[0]
        goal.pose.orientation.x = quaternion[1]
        goal.pose.orientation.y = quaternion[2]
        goal.pose.orientation.z = quaternion[3]
        self.log.info(f"goal is {goal}")
        self.goal_pub.publish(goal)

    def on_vehicle_control(self, data):
        """
        callback if a new vehicle control command is received
        """
        cmd = carla.VehicleControl()
        cmd.throttle = data.twist.linear.x / 100.0
        cmd.steer = data.twist.angular.z / 100.0
        cmd.brake = data.twist.linear.y / 100.0
        self.current_control = cmd
        self.step_mode_possible = True

    def on_autoware_universe_vehicle_control(self, data):
        """
        callback if a new vehicle control command is received
        """
        cmd = carla.VehicleControl()
        cmd.steer = (-data.lateral.steering_tire_angle / self.max_steer_angle) * self.steering_factor
        speed_diff = data.longitudinal.speed - self.forward_speed
        if speed_diff > 0:
            cmd.throttle = 0.75
            cmd.brake = 0.0
        elif speed_diff < 0.0:
            cmd.throttle = 0.0
            if data.longitudinal.speed <= 0.0:
                cmd.brake = 0.75
            elif speed_diff > -1:
                cmd.brake = 0.0
            else:
                cmd.brake = 0.01

        self.current_control = cmd
        self.step_mode_possible = True

    def _update_pose(self):
        pose_msg = PoseWithCovariance()

        loc = self.ego_vehicle.get_transform().location
        rot = self.ego_vehicle.get_transform().rotation
        quat = transforms3d.euler.euler2quat(
            math.radians(rot.roll),
            math.radians(-rot.pitch),
            math.radians(-rot.yaw)
            )
        pose_msg.pose.position.x = loc.x
        pose_msg.pose.position.y = - loc.y
        pose_msg.pose.position.z = loc.z
        pose_msg.pose.orientation.w = quat[0]
        pose_msg.pose.orientation.x = quat[1]
        pose_msg.pose.orientation.y = quat[2]
        pose_msg.pose.orientation.z = quat[3]
        return pose_msg

    def _update_twist(self):
        twist_msg = TwistWithCovariance()

        speed = self.ego_vehicle.get_velocity()
        local_speed = speed
        # local_speed = self.tf_sensor.get_vector_transform(speed)
        angular_vel = self.ego_vehicle.get_angular_velocity()
        twist_msg.twist.linear.x = local_speed.x
        twist_msg.twist.linear.y = -local_speed.y
        twist_msg.twist.linear.z = local_speed.z
        twist_msg.twist.angular.x = angular_vel.x
        twist_msg.twist.angular.y = -angular_vel.y
        twist_msg.twist.angular.z = -angular_vel.z
        return twist_msg

    def _pub_odo(self):
        odo_msg = Odometry()
        odo_msg.header = Header(stamp=self.node.get_timestamp(), frame_id='map')

        odo_msg.pose = self._update_pose()
        odo_msg.twist = self._update_twist()
        self.odo_pub.publish(odo_msg)

    def _publish_can(self):
        self.forward_speed = self._get_forward_speed()

        twist_msg = TwistWithCovariance()
        twist_msg.twist.linear.x = self.forward_speed
        if twist_msg.twist.linear.x < 0.0:
            twist_msg.twist.linear.x = 0.0
        twist_msg.twist.angular.z = -self.current_control.steer
        twist_msg.twist.linear.z = 1.0  # to tell OpenPlanner to use the steer directly
        twist_msg.twist.angular.x = 1.0  # to tell OpenPlanner to use the steer directly

        odo_msg = Odometry()
        odo_msg.header = Header(stamp=self.node.get_timestamp())
        odo_msg.twist = twist_msg

        vel_rep = VelocityReport()
        vel_rep.header = Header(stamp=self.node.get_timestamp())
        vel_rep.header.frame_id = "base_link"
        vel_rep.longitudinal_velocity = self.forward_speed

        transform = self.ego_vehicle.get_transform()
        vel_rep.heading_rate = math.radians(-transform.rotation.yaw)
        self.auto_velocity_status_publisher.publish(vel_rep)

        steer_rep = SteeringReport()
        steer_rep.steering_tire_angle = (-self.current_control.steer * self.max_steer_angle) / self.steering_factor
        self.auto_steering_status_publisher.publish(steer_rep)

        gear_rep = GearReport()
        gear_rep.stamp = Header(stamp=self.node.get_timestamp()).stamp
        gear_rep.report = GearReport.DRIVE
        self.auto_gear_status_publisher.publish(gear_rep)

        control_mode_rep = ControlModeReport()
        control_mode_rep.stamp = Header(stamp=self.node.get_timestamp()).stamp
        control_mode_rep.mode = ControlModeReport.AUTONOMOUS
        self.auto_control_mode_publisher.publish(control_mode_rep)

    def update(self):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        # self._pub_odo()
        self._publish_can()
        self.ego_vehicle.apply_control(self.current_control)
        # self.log.info(f"ego vehicle update is running")

    def _get_forward_speed(self):
        """ Convert the vehicle transform directly to forward speed """

        velocity = self.ego_vehicle.get_velocity()
        transform = self.ego_vehicle.get_transform()
        vel_np = np.array([velocity.x, velocity.y, velocity.z])
        pitch = np.deg2rad(transform.rotation.pitch)
        yaw = np.deg2rad(transform.rotation.yaw)
        orientation = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch)])
        speed = np.dot(vel_np, orientation)
        return speed


