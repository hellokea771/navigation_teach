from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_my_tf = get_package_share_directory('my_tf')
    pkg_lidar = get_package_share_directory('lidar_node')

    urdf_file = os.path.join(pkg_my_tf, 'urdf', 'my_diffbot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    joint_state_publisher = Node(
        package='my_tf',
        executable='joint_state_pub.py',
        name='joint_state_publisher',
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    tf_broadcaster = Node(
        package='my_tf',
        executable='my_tf2_broadcaster',
        name='my_tf2_broadcaster',
        output='screen',
    )

    lidar_node = Node(
        package='lidar_node',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_lidar, 'config', 'display_all.rviz')],
        output='screen',
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        tf_broadcaster,
        lidar_node,
        rviz,
    ])
