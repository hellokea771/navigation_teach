from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # 接收节点
    subscriber = Node(
        package="my_third_package",
        executable="my_third_node",
        name="my_subscriber",
        output="screen"
    )


    # 发布节点
    publisher = Node(
        package="my_first_package",
        executable="helloworld",
        name="my_publisher",
        output="screen"
    )


    # 延迟5秒启动发布节点
    delayed_publisher = TimerAction(
        period=5.0,
        actions=[
            publisher
        ]
    )


    return LaunchDescription([
        subscriber,
        delayed_publisher
    ])