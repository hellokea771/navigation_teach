import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from pure_pursuit_rviz_demo.common import yaw_to_quaternion


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(Path, '/reference_path', 10)
        self.path = self.create_reference_path()
        self.timer = self.create_timer(1.0, self.publish_path)

    def create_reference_path(self):
        path = Path()
        path.header.frame_id = 'odom'

        points = []
        step = 0.15
        count = int(14.0 / step) + 1
        for i in range(count):
            x = i * step
            y = 1.2 * math.sin(0.55 * x)
            points.append((x, y))

        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = x
            pose.pose.position.y = y
            if i + 1 < len(points):
                nx, ny = points[i + 1]
            else:
                nx, ny = points[i]
            yaw = math.atan2(ny - y, nx - x)
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)

        return path

    def publish_path(self):
        stamp = self.get_clock().now().to_msg()
        self.path.header.stamp = stamp
        for pose in self.path.poses:
            pose.header.stamp = stamp
        self.publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
