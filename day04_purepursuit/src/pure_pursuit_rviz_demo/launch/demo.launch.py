#!/usr/bin/env python3
"""
demo.launch.py

One-click launch for the Pure Pursuit demo.
Launches:
  - robot_state_publisher (URDF)
  - path_publisher
  - virtual_robot
  - pure_pursuit_controller
  - RViz2

Usage:
  ros2 launch pure_pursuit_rviz_demo demo.launch.py
  ros2 launch pure_pursuit_rviz_demo demo.launch.py lookahead_distance:=0.5
  ros2 launch pure_pursuit_rviz_demo demo.launch.py target_speed:=1.2 path_type:=corner
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('pure_pursuit_rviz_demo')

    # ---- Launch arguments ----
    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance', default_value='1.0',
        description='Pure Pursuit lookahead distance (m)'
    )
    speed_arg = DeclareLaunchArgument(
        'target_speed', default_value='0.6',
        description='Target linear speed (m/s)'
    )
    path_type_arg = DeclareLaunchArgument(
        'path_type', default_value='s_shape',
        description='Reference path type: straight, s_shape, or corner'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to launch RViz2'
    )

    lookahead_distance = LaunchConfiguration('lookahead_distance')
    target_speed = LaunchConfiguration('target_speed')
    path_type = LaunchConfiguration('path_type')
    use_rviz = LaunchConfiguration('use_rviz')

    # ---- Robot State Publisher (URDF) ----
    urdf_path = os.path.join(package_dir, 'urdf', 'simple_car.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ---- Path Publisher ----
    path_publisher = Node(
        package='pure_pursuit_rviz_demo',
        executable='path_publisher',
        name='path_publisher',
        parameters=[{'path_type': path_type}],
        output='screen',
    )

    # ---- Virtual Robot ----
    virtual_robot = Node(
        package='pure_pursuit_rviz_demo',
        executable='virtual_robot',
        name='virtual_robot',
        output='screen',
    )

    # ---- Pure Pursuit Controller ----
    controller = Node(
        package='pure_pursuit_rviz_demo',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        parameters=[{
            'lookahead_distance': lookahead_distance,
            'target_speed': target_speed,
        }],
        output='screen',
    )

    # ---- RViz2 ----
    rviz_config = os.path.join(package_dir, 'rviz', 'pure_pursuit_demo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        lookahead_arg,
        speed_arg,
        path_type_arg,
        use_rviz_arg,
        robot_state_publisher,
        path_publisher,
        virtual_robot,
        controller,
        rviz_node,
    ])
