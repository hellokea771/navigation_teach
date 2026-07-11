import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit_rviz_demo')
    urdf_path = os.path.join(pkg_share, 'urdf', 'simple_car.urdf')
    rviz_path = os.path.join(pkg_share, 'rviz', 'pure_pursuit_demo.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = {'robot_description': urdf_file.read()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='pure_pursuit_rviz_demo',
            executable='path_publisher',
            name='path_publisher',
            output='screen',
        ),
        Node(
            package='pure_pursuit_rviz_demo',
            executable='virtual_robot',
            name='virtual_robot',
            output='screen',
        ),
        Node(
            package='pure_pursuit_rviz_demo',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            output='screen',
            parameters=[{
                'linear_speed': 0.8,
                'lookahead_distance': 1.1,
                'goal_tolerance': 0.25,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen',
        ),
    ])
