import math

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from visualization_msgs.msg import Marker

from pure_pursuit_rviz_demo.common import normalize_angle, quaternion_to_yaw


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.declare_parameter('linear_speed', 0.8)
        self.declare_parameter('lookahead_distance', 1.1)
        self.declare_parameter('goal_tolerance', 0.25)

        self.path = None
        self.odom = None

        self.create_subscription(Path, '/reference_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/lookahead_point', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def path_callback(self, msg):
        self.path = msg

    def odom_callback(self, msg):
        self.odom = msg

    def control_loop(self):
        if self.path is None or self.odom is None or not self.path.poses:
            self.publish_stop()
            return

        robot_x = self.odom.pose.pose.position.x
        robot_y = self.odom.pose.pose.position.y
        robot_yaw = quaternion_to_yaw(self.odom.pose.pose.orientation)

        closest_index = self.find_closest_index(robot_x, robot_y)
        target_index = self.find_lookahead_index(closest_index)
        target_pose = self.path.poses[target_index].pose
        target_x = target_pose.position.x
        target_y = target_pose.position.y

        dx = target_x - robot_x
        dy = target_y - robot_y
        target_distance = math.hypot(dx, dy)
        goal_pose = self.path.poses[-1].pose
        goal_distance = math.hypot(goal_pose.position.x - robot_x, goal_pose.position.y - robot_y)

        self.publish_lookahead_marker(target_x, target_y)

        cmd = Twist()
        if target_index == len(self.path.poses) - 1 and goal_distance < self.get_parameter('goal_tolerance').value:
            self.cmd_pub.publish(cmd)
            return

        if target_distance < 1e-4:
            self.cmd_pub.publish(cmd)
            return

        speed = float(self.get_parameter('linear_speed').value)
        alpha = normalize_angle(math.atan2(dy, dx) - robot_yaw)
        curvature = 2.0 * math.sin(alpha) / target_distance

        cmd.linear.x = speed
        cmd.angular.z = speed * curvature
        self.cmd_pub.publish(cmd)

    def find_closest_index(self, robot_x, robot_y):
        best_index = 0
        best_distance = float('inf')
        for i, pose_stamped in enumerate(self.path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            distance = math.hypot(px - robot_x, py - robot_y)
            if distance < best_distance:
                best_distance = distance
                best_index = i
        return best_index

    def find_lookahead_index(self, start_index):
        lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        total = 0.0
        last = self.path.poses[start_index].pose.position
        for i in range(start_index + 1, len(self.path.poses)):
            current = self.path.poses[i].pose.position
            total += math.hypot(current.x - last.x, current.y - last.y)
            if total >= lookahead_distance:
                return i
            last = current
        return len(self.path.poses) - 1

    def publish_lookahead_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pure_pursuit'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=0.18)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.28
        marker.scale.y = 0.28
        marker.scale.z = 0.28
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.05
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
