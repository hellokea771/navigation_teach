#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint', 'caster_joint', 'lidar_joint']
        msg.position = [0.0, 0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
