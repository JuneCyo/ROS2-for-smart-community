#!/usr/bin/env python3
"""Show occupied map cells in RViz without its broken VMware map shader."""
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class MapObstaclesCloud(Node):
    def __init__(self):
        super().__init__('map_obstacles_cloud')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PointCloud2, '/map_obstacles', qos)
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.on_map, qos)

    def on_map(self, msg):
        info = msg.info
        origin = info.origin
        q = origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        points = []
        for row in range(info.height):
            for col in range(info.width):
                if msg.data[row * info.width + col] < 65:
                    continue
                x = (col + 0.5) * info.resolution
                y = (row + 0.5) * info.resolution
                points.append((origin.position.x + cosine * x - sine * y,
                               origin.position.y + sine * x + cosine * y,
                               origin.position.z + 0.02))
        header = Header()
        header.frame_id = msg.header.frame_id or 'map'
        header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, points))
        self.get_logger().info(f'RViz 地图障碍点云已发布：{len(points)} 个占用栅格')


def main():
    rclpy.init()
    node = MapObstaclesCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
    try:
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
