#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""沿场地外圈和中间通道行驶，供 Cartographer 建图。

用 /odom 跟踪航点，用 /scan 避开近处障碍。路线、速度和圈数由 ROS 参数设置。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# 东侧走 x=3.9，穿过红绿灯立柱；南侧走 y≈-3.6，避开 y=-4 的停车列。
# 最后一段沿楼宇北侧 y=-1.4 补扫。
ROUTE_CCW = [
    (3.90, -3.60), (3.90, 3.90),
    (-0.20, 3.90), (-4.20, 3.90),
    (-4.20, -3.30),
    (-0.30, -3.60), (3.60, -3.60),
    (3.60, -1.40),
    (-3.60, -1.40),
]
ROUTE_CW = [
    (3.90, 3.90), (3.90, -3.60),
    (-0.30, -3.60), (-4.20, -3.30),
    (-4.20, 3.90), (-0.20, 3.90),
    (3.60, -1.40), (-3.60, -1.40),
]


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SurveyDriver(Node):
    def __init__(self):
        super().__init__('survey_driver')
        self.declare_parameter('speed', 0.30)
        self.declare_parameter('turn_speed', 0.8)
        self.declare_parameter('goal_tol', 0.18)
        self.declare_parameter('stop_dist', 0.35)
        self.declare_parameter('loops', 1)
        self.declare_parameter('route', 'default')

        self.speed = float(self.get_parameter('speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.loops_target = int(self.get_parameter('loops').value)
        route_name = str(self.get_parameter('route').value)
        self.route = ROUTE_CW if route_name == 'clockwise' else ROUTE_CCW

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.x = self.y = self.yaw = None
        self.scan = None
        self.idx = 0
        self.loops = 0
        self.finished = False
        # 20 秒内位移过小时短暂倒车脱困。
        self.last_x = self.last_y = None
        self.last_progress = 0.0
        self.reverse_until = 0.0
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info('survey_driver 启动: %d 个航点, 速度 %.2f m/s, 跑 %d 圈'
                               % (len(self.route), self.speed, self.loops_target))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_scan(self, msg):
        self.scan = msg

    def sector_min(self, a0, a1):
        """取扇区内第三近的距离，过滤单条过近读数。"""
        if self.scan is None:
            return float('inf')
        inc = self.scan.angle_increment
        i0 = int(round((a0 - self.scan.angle_min) / inc))
        i1 = int(round((a1 - self.scan.angle_min) / inc))
        i0 = max(0, min(i0, len(self.scan.ranges) - 1))
        i1 = max(0, min(i1, len(self.scan.ranges) - 1))
        if i1 < i0:
            i0, i1 = i1, i0
        vals = sorted(r for r in self.scan.ranges[i0:i1 + 1]
                      if r > self.scan.range_min and r < self.scan.range_max)
        if not vals:
            return float('inf')
        return vals[2] if len(vals) >= 3 else vals[0]

    def stop_robot(self):
        self.pub.publish(Twist())

    def on_timer(self):
        if self.finished:
            return
        if self.x is None or self.scan is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_x is None or now - self.last_progress > 20.0:
            if self.last_x is not None:
                moved = math.hypot(self.x - self.last_x, self.y - self.last_y)
                if moved < 0.25 and now > self.reverse_until:
                    self.get_logger().warn('20s 只挪了 %.2f m, 倒车脱困' % moved)
                    self.reverse_until = now + 1.5
            self.last_x, self.last_y = self.x, self.y
            self.last_progress = now
        if now < self.reverse_until:
            cmd = Twist()
            cmd.linear.x = -0.15
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return

        tx, ty = self.route[self.idx]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist < self.goal_tol:
            self.idx += 1
            if self.idx >= len(self.route):
                self.idx = 0
                self.loops += 1
                self.get_logger().info('已跑完第 %d 圈' % self.loops)
                if self.loops >= self.loops_target:
                    self.get_logger().info('建图路线全部跑完, 停车')
                    self.stop_robot()
                    self.finished = True
            return

        err = wrap(math.atan2(dy, dx) - self.yaw)
        front = self.sector_min(-0.45, 0.45)
        front_near = self.sector_min(-1.1, 1.1)
        left_free = self.sector_min(0.5, 2.2)
        right_free = self.sector_min(-2.2, -0.5)

        cmd = Twist()
        if front < self.stop_dist:
            # 前方被挡住时，转向空余更大的一侧。
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed if left_free >= right_free else -self.turn_speed
            self.get_logger().warn('前方 %.2f m 有障碍, 原地转向(左%.1f/右%.1f)'
                                   % (front, left_free, right_free), throttle_duration_sec=2.0)
        elif abs(err) > 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.turn_speed, min(self.turn_speed, 1.6 * err))
        else:
            slow = 1.0 - min(0.7, abs(err) / 0.8)
            if front_near < 0.6:
                slow *= 0.6
            cmd.linear.x = self.speed * slow
            cmd.angular.z = max(-self.turn_speed, min(self.turn_speed, 1.6 * err))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = SurveyDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
