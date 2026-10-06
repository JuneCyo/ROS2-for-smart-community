#!/usr/bin/env python3
"""Continuously broadcast map->odom from AMCL pose and Gazebo odometry.

Humble AMCL in this simulation can stop refreshing its TF while still
publishing /amcl_pose. Nav2's controller rejects that stale transform.
"""
import math

import rclpy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AmclTfBridge(Node):
    def __init__(self):
        super().__init__('amcl_tf_bridge')
        # Seed map->odom before AMCL publishes its first pose. Otherwise a
        # startup race can leave Nav2 waiting for the transform indefinitely.
        self.declare_parameter('initial_x', 3.95)
        self.declare_parameter('initial_y', -4.0)
        self.declare_parameter('initial_yaw', math.pi / 2)
        self.amcl_pose = Pose()
        self.amcl_pose.position.x = float(self.get_parameter('initial_x').value)
        self.amcl_pose.position.y = float(self.get_parameter('initial_y').value)
        initial_yaw = float(self.get_parameter('initial_yaw').value)
        self.amcl_pose.orientation.z = math.sin(initial_yaw / 2)
        self.amcl_pose.orientation.w = math.cos(initial_yaw / 2)
        self.odom_pose = None
        self.odom_at_amcl = None
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_amcl_pose, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.1, self.broadcast)
        self.get_logger().info('等待 /odom 后以初始位姿发布 map→odom；收到 /amcl_pose 后改用定位位姿')

    def on_amcl_pose(self, msg):
        self.amcl_pose = msg.pose.pose
        # Keep the odometry pose from this localization update. Recomputing
        # map->odom against every newer odom message would pin the robot to
        # the last AMCL position whenever AMCL temporarily stops publishing.
        self.odom_at_amcl = self.odom_pose

    def on_odom(self, msg):
        self.odom_pose = msg.pose.pose
        if self.odom_at_amcl is None:
            self.odom_at_amcl = self.odom_pose

    def broadcast(self):
        if self.odom_at_amcl is None:
            return
        map_pose = self.amcl_pose
        odom_pose = self.odom_at_amcl
        yaw = yaw_of(map_pose.orientation) - yaw_of(odom_pose.orientation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        ox, oy = odom_pose.position.x, odom_pose.position.y

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = map_pose.position.x - cosine * ox + sine * oy
        transform.transform.translation.y = map_pose.position.y - sine * ox - cosine * oy
        transform.transform.translation.z = map_pose.position.z - odom_pose.position.z
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = AmclTfBridge()
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
