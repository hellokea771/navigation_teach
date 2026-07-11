import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from pure_pursuit_rviz_demo.common import yaw_to_quaternion


class VirtualRobot(Node):
    def __init__(self):
        super().__init__('virtual_robot')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', -0.4)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.v = 0.0
        self.omega = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/actual_path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.update)

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0:
            return

        if (now - self.last_cmd_time).nanoseconds * 1e-9 > float(self.get_parameter('cmd_timeout').value):
            self.v = 0.0
            self.omega = 0.0

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.omega * dt

        self.publish_odom_and_tf(now)
        self.publish_actual_path(now)

    def publish_odom_and_tf(self, now):
        q = yaw_to_quaternion(self.yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega
        self.odom_pub.publish(odom)

    def publish_actual_path(self, now):
        pose = Odometry()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.pose.position.x = self.x
        pose.pose.pose.position.y = self.y
        pose.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        self.actual_path.header.stamp = now.to_msg()
        self.actual_path.poses.append(self.odom_to_pose_stamped(pose))
        if len(self.actual_path.poses) > 1500:
            self.actual_path.poses = self.actual_path.poses[-1500:]
        self.path_pub.publish(self.actual_path)

    @staticmethod
    def odom_to_pose_stamped(odom):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = VirtualRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
