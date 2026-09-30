#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动"跑遍全场"的航点跟踪节点 —— 给 Cartographer 建图用
=====================================================
* 位姿直接取 /odom(gazebo_ros_diff_drive 发的是世界坐标, 见 my_robot/README.md)
* 前方障碍用 /scan 的前向扇区判断, 太近就原地转向躲开(简单反应式避障)
* 默认路线: 沿外墙跑一圈 + 中间走廊补扫, 覆盖 10x10 m 全场景

用法:
    ros2 run my_robot survey_driver.py
    ros2 run my_robot survey_driver.py --ros-args -p speed:=0.25 -p loops:=2
    ros2 run my_robot survey_driver.py --ros-args -p route:=clockwise

参数:
    speed       前进速度(m/s)            默认 0.22
    turn_speed  原地/转向角速度(rad/s)    默认 0.8
    goal_tol    到点判定半径(m)           默认 0.18
    stop_dist   前方障碍急停距离(m)       默认 0.35
    loops       跑几圈                    默认 1
    route       default(逆时针) / clockwise
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# 场地 10x10 m, 内墙 ±4.8; 已经存在的障碍:
#   * 车(平面贴图 1x1m, 很薄): y=-4, x=-1.2/-2.6/-4.0
#   * 人偶立牌(1.6x1.2m 竖板): (2.5, 2.0) 竖着放, 约占 x≈2.5, y∈[1.2,2.8]
#   * 红绿灯 1(龙门架): 中心 (4,0), 两根立柱在 x=3.27 / x=4.73, 横梁在 1.85m 高处
#   * 红绿灯 2(龙门架): 中心 (-2,0.25), 立柱 x=-2.73 / x=-1.27
#   * 楼宇A/B/C + 垃圾桶: 东西向一排, 占 x∈[-3.31, 3.31], y∈[-3.03, -2.17](长白线走廊里)
# 路线: 东侧走 x=3.9(从红绿灯两根柱子中间穿过), 西侧 x=-4.2, 南侧 y≈-3.6(避开 y=-4 的车),
#       再从楼宇排北面 y=-1.40 一路向西补扫北面。
ROUTE_CCW = [
    (3.90, -3.60), (3.90, 3.90),          # 东侧主干道向北(穿过红绿灯龙门架)
    (-0.20, 3.90), (-4.20, 3.90),         # 北侧向西
    (-4.20, -3.30),                        # 西侧向南(在车北边停住)
    (-0.30, -3.60), (3.60, -3.60),        # 南侧向东(从车与楼宇排之间过)
    (3.60, -1.40),                         # 向北
    (-3.60, -1.40),                        # 沿楼宇排北面一路向西(补扫这一侧)
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
        # 卡死检测: 一段时间内几乎没挪动就倒车一下
        self.last_x = self.last_y = None
        self.last_progress = 0.0
        self.reverse_until = 0.0
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info('survey_driver 启动: %d 个航点, 速度 %.2f m/s, 跑 %d 圈'
                               % (len(self.route), self.speed, self.loops_target))

    # ---------------------------------------------------------------- 回调
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_scan(self, msg):
        self.scan = msg

    def sector_min(self, a0, a1):
        """某个角度扇区内第 3 小的距离(抗单条杂散读数)"""
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

    # ---------------------------------------------------------------- 主循环
    def on_timer(self):
        if self.finished:
            return
        if self.x is None or self.scan is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        # --- 卡死检测(每 20s 检查一次走了多远) ---
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
        front = self.sector_min(-0.45, 0.45)      # 前方 ±26°
        front_near = self.sector_min(-1.1, 1.1)   # 前方 ±63°
        left_free = self.sector_min(0.5, 2.2)     # 左侧
        right_free = self.sector_min(-2.2, -0.5)  # 右侧

        cmd = Twist()
        if front < self.stop_dist:
            # 太近: 停车, 朝空间更大的一侧原地转(而不是死盯着目标方向)
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
