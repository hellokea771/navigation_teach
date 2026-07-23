#!/usr/bin/env python3
"""Launch Pure Pursuit demo: URDF robot + path + controller + RViz."""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('pure_pursuit_rviz_demo')
    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '1')

    # Load URDF
    urdf_path = os.path.join(package_dir, 'urdf', 'simple_car.urdf')
    with open(urdf_path) as f:
        robot_desc = f.read()

    # Write robot_description to a YAML file for rviz2
    params_yaml_path = os.path.join('/tmp', 'pure_pursuit_robot_params.yaml')
    with open(params_yaml_path, 'w') as f:
        yaml.dump({'/**': {'ros__parameters': {'robot_description': robot_desc}}}, f)

    # Arguments
    lookahead_arg = DeclareLaunchArgument('lookahead_distance', default_value='1.0')
    speed_arg = DeclareLaunchArgument('target_speed', default_value='0.6')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    ld = LaunchConfiguration('lookahead_distance')
    ts = LaunchConfiguration('target_speed')
    rviz = LaunchConfiguration('use_rviz')

    # Static TF: map -> odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    )

    # Path Publisher
    path_pub = Node(
        package='pure_pursuit_rviz_demo',
        executable='path_publisher',
    )

    # Virtual Robot (with watchdog timeout 0.5s)
    virtual_robot = Node(
        package='pure_pursuit_rviz_demo',
        executable='virtual_robot',
    )

    # Pure Pursuit Controller
    controller = Node(
        package='pure_pursuit_rviz_demo',
        executable='pure_pursuit_controller',
        parameters=[{'lookahead_distance': ld, 'target_speed': ts}],
    )

    # RViz2 - robot_description loaded from YAML file
    rviz_config = os.path.join(package_dir, 'rviz', 'pure_pursuit_demo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[params_yaml_path],
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        set_domain,
        lookahead_arg, speed_arg, use_rviz_arg,
        static_tf, rsp, path_pub, virtual_robot, controller, rviz_node,
    ])
