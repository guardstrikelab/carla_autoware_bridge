#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2024 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""

import time

import carla

sensors = [
    {
        'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.0, 'z': 1.6, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'width': 1280, 'height': 720, 'fov': 100, 'id': 'Center'
    },
    {
        'type': 'sensor.lidar.ray_cast', 'x': 0.0, 'y': 0.0, 'z': 2.4, 'roll': 0.0, 'pitch': 0.0, 'yaw': -90, 'id': 'LIDAR'},
    {
        'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 2.4, 'id': 'GPS'
    },
    {
        'type': 'sensor.other.imu', 'x': 0.0, 'y': 0.0, 'z': 2.4, 'roll': 0.0, 'pitch': 0.0, 'yaw': 270.0, 'id': 'IMU'
    },
]


def setup_sensors(vehicle, client, sensors):
    """
    Create the sensors defined by the user and attach them to the ego-vehicle
    :param vehicle: ego vehicle
    :return:
    """
    bp_library = client.get_world().get_blueprint_library()
    for sensor_spec in sensors:
        # These are the sensors spawned on the carla world
        sensor_location = None
        sensor_rotation = None
        bp = bp_library.find(str(sensor_spec['type']))
        if sensor_spec['type'].startswith('sensor.camera'):
            bp.set_attribute('image_size_x', str(sensor_spec['width']))
            bp.set_attribute('image_size_y', str(sensor_spec['height']))
            bp.set_attribute('fov', str(sensor_spec['fov']))
            bp.set_attribute('lens_circle_multiplier', str(3.0))
            bp.set_attribute('lens_circle_falloff', str(3.0))
            bp.set_attribute('chromatic_aberration_intensity', str(0.5))
            bp.set_attribute('chromatic_aberration_offset', str(0))
            bp.set_attribute('role_name', sensor_spec['id'])
            sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                             roll=sensor_spec['roll'],
                                             yaw=sensor_spec['yaw'])
        elif sensor_spec['type'].startswith('sensor.lidar'):
            bp.set_attribute('range', str(100))
            bp.set_attribute('rotation_frequency', str(120))
            bp.set_attribute('channels', str(32))
            bp.set_attribute('upper_fov', str(10))
            bp.set_attribute('lower_fov', str(-30))
            bp.set_attribute('points_per_second', str(1200000))
            bp.set_attribute('atmosphere_attenuation_rate', str(0.004))
            bp.set_attribute('dropoff_general_rate', str(0.45))
            bp.set_attribute('dropoff_intensity_limit', str(0.8))
            bp.set_attribute('dropoff_zero_intensity', str(0.4))
            bp.set_attribute('role_name', sensor_spec['id'])
            bp.set_attribute('sensor_tick', '0.1')
            sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                             roll=sensor_spec['roll'],
                                             yaw=sensor_spec['yaw'])
        elif sensor_spec['type'].startswith('sensor.other.radar'):
            bp.set_attribute('horizontal_fov', str(
                sensor_spec['fov']))  # degrees
            bp.set_attribute('vertical_fov', str(
                sensor_spec['fov']))  # degrees
            bp.set_attribute('points_per_second', '1500')
            bp.set_attribute('range', '100')  # meters
            bp.set_attribute('role_name', sensor_spec['id'])
            sensor_location = carla.Location(x=sensor_spec['x'],
                                             y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                             roll=sensor_spec['roll'],
                                             yaw=sensor_spec['yaw'])

        elif sensor_spec['type'].startswith('sensor.other.gnss'):
            bp.set_attribute('noise_alt_stddev', str(0.0))
            bp.set_attribute('noise_lat_stddev', str(0.0))
            bp.set_attribute('noise_lon_stddev', str(0.0))
            bp.set_attribute('noise_alt_bias', str(0.0))
            bp.set_attribute('noise_lat_bias', str(0.0))
            bp.set_attribute('noise_lon_bias', str(0.0))

            sensor_location = carla.Location(x=sensor_spec['x'],
                                             y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation()

        elif sensor_spec['type'].startswith('sensor.other.imu'):
            bp.set_attribute('noise_accel_stddev_x', str(0.0))
            bp.set_attribute('noise_accel_stddev_y', str(0.0))
            bp.set_attribute('noise_accel_stddev_z', str(0.0))
            bp.set_attribute('noise_gyro_stddev_x', str(0.0))
            bp.set_attribute('noise_gyro_stddev_y', str(0.0))
            bp.set_attribute('noise_gyro_stddev_z', str(0.0))

            sensor_location = carla.Location(x=sensor_spec['x'],
                                             y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                             roll=sensor_spec['roll'],
                                             yaw=sensor_spec['yaw'])
        # create sensor
        sensor_transform = carla.Transform(sensor_location, sensor_rotation)
        client.get_world().spawn_actor(bp, sensor_transform, vehicle)


if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    client.load_world('Town01')
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.*')[0]
    vehicle_bp.set_attribute('role_name', 'ego_vehicle')
    # spawn_point = world.get_map().get_spawn_points()[0]
    # print(f"spawn_point is {spawn_point}")
    spawn_point = carla.Transform(carla.Location(x=119.267, y=330.394, z=0), carla.Rotation(yaw=0, pitch=0, roll=0))
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    setup_sensors(vehicle, client, sensors)
    # Generate an obstacle
    # obs_point = carla.Transform(spawn_point.location + carla.Location(50, 0, 0),
    #                             spawn_point.rotation)
    # obs = world.spawn_actor(blueprint_library.filter('vehicle.*')[0], obs_point)
    print('done')
    while True:
        spectator = None
        try:
            spectator = client.get_world().get_spectator()
            ego_trans = vehicle.get_transform()
            spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(z=30),
                                                    carla.Rotation(pitch=-90)))
            time.sleep(0.05)
        except KeyboardInterrupt:
            spectator.destroy()
            vehicle.destroy()
            break
