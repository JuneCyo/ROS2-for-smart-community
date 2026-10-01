#!/usr/bin/env python3
"""AMCL-localized closed route through both traffic-light openings.

Nav2 plans each leg from the live AMCL pose. A small pure-pursuit controller
tracks that plan on /cmd_vel because this Gazebo/Humble Nav2 controller can
continue driving after its internal TF cache stops updating.
"""
import math
import os
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


START = (3.95, -4.0, math.pi / 2)
ROUTE = [
    ('起步直行', 3.95, -2.65, math.pi / 2),
    ('红绿灯 1 南侧', 3.95, -0.85, math.pi / 2),
    ('通过红绿灯 1', 3.95, 0.85, math.pi / 2),
    ('北端左转', 3.95, 3.8, math.pi / 2),
    ('北侧向西', 1.05, 3.8, math.pi),
    ('再次左转向南', 1.05, 0.10, -math.pi / 2),
    ('右转通过红绿灯 2', -2.0, 0.10, math.pi),
    ('西端左转', -4.0, 0.10, math.pi),
    ('车牌白线前左转', -4.0, -2.90, -math.pi / 2),
    ('横穿至东端右转', 3.95, -2.90, 0.0),
    ('回到起点', *START),
]


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrafficLightRoute(Node):
    def __init__(self):
        super().__init__('navigate_traffic_lights')
        self.declare_parameter('start_at', 0)
        self.declare_parameter('speed', 0.18)
        self.index = int(self.get_parameter('start_at').value)
        if not 0 <= self.index < len(ROUTE):
            raise ValueError('start_at must be between 0 and %d' % (len(ROUTE) - 1))
        self.speed = float(self.get_parameter('speed').value)
        self.pose = None
        self.pose_time = None
        self.scan = None
        self.scan_time = None
        self.static_map = None
        self.map_origin_yaw = 0.0
        self.light_truth = [('unknown', 0), ('unknown', 0)]
        self.light_vision = [('unknown', 0), ('unknown', 0)]
        self.avoidance = None
        self.white_distance = self.load_white_line_distance()
        self.path = []
        self.planning = False
        self.plan_tries = 0
        self.next_plan_time_ns = 0
        self.finished = False
        self.exit_code = 1
        # Nav2's velocity_smoother owns /cmd_vel; feed its input to avoid
        # competing publishers on the Gazebo diff-drive command topic.
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        signal_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, signal_qos)
        for light_id in (1, 2):
            self.create_subscription(
                String, f'/traffic_light/light{light_id}/state',
                lambda msg, i=light_id - 1: self.on_light(msg, i, False), signal_qos)
            self.create_subscription(
                String, f'/traffic_light/light{light_id}/vision_state',
                lambda msg, i=light_id - 1: self.on_light(msg, i, True), 10)
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.create_timer(0.1, self.tick)
        self.get_logger().info('等待 AMCL、/scan 和 Nav2 路径规划器；路线共 %d 个航点' % len(ROUTE))

    def leg(self):
        start = START if self.index == 0 else ROUTE[self.index - 1][1:4]
        goal = ROUTE[self.index][1:4]
        return start[0], start[1], goal[0], goal[1]

    def distance_from_leg(self, x, y):
        sx, sy, gx, gy = self.leg()
        dx, dy = gx - sx, gy - sy
        t = max(0.0, min(1.0, ((x - sx) * dx + (y - sy) * dy) /
                               max(dx * dx + dy * dy, 1e-6)))
        return math.hypot(x - sx - t * dx, y - sy - t * dy)

    def on_pose(self, msg):
        self.pose = msg.pose.pose
        self.pose_time = self.get_clock().now()

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.get_clock().now()

    def on_light(self, msg, index, vision):
        value = msg.data if msg.data in ('red', 'yellow', 'green') else 'unknown'
        target = self.light_vision if vision else self.light_truth
        target[index] = (value, self.get_clock().now().nanoseconds)

    def on_map(self, msg):
        self.static_map = msg
        self.map_origin_yaw = yaw_of(msg.info.origin.orientation)

    def static_obstacle_near(self, x, y):
        """Check the published static map with localization tolerance."""
        if self.static_map is None:
            return False
        info = self.static_map.info
        dx = x - info.origin.position.x
        dy = y - info.origin.position.y
        c, s = math.cos(self.map_origin_yaw), math.sin(self.map_origin_yaw)
        col = math.floor((c * dx + s * dy) / info.resolution)
        row = math.floor((-s * dx + c * dy) / info.resolution)
        radius = math.ceil(0.12 / info.resolution)
        for r in range(max(0, row - radius), min(info.height, row + radius + 1)):
            for k in range(max(0, col - radius), min(info.width, col + radius + 1)):
                if self.static_map.data[r * info.width + k] >= 65:
                    return True
        return not (0 <= row < info.height and 0 <= col < info.width)

    def load_white_line_distance(self):
        """Ground texture with the road-colored SDF gaps removed."""
        try:
            share = get_package_share_directory('smart_community')
            image_file = os.path.join(share, 'models', 'competition_ground',
                                      'materials', 'textures', 'map1.jpg')
            world_file = os.path.join(share, 'worlds', 'competition_world.sdf')
            image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise FileNotFoundError(image_file)
            white = np.uint8(image >= 150)
            height, width = white.shape
            gaps = ET.parse(world_file).find(".//model[@name='navigation_line_gaps']")
            if gaps is not None:
                for visual in gaps.findall('.//visual'):
                    pose = visual.findtext('pose')
                    size = visual.findtext('./geometry/box/size')
                    if not pose or not size:
                        continue
                    px, py = [float(v) for v in pose.split()[:2]]
                    sx, sy = [float(v) for v in size.split()[:2]]
                    left = round((px - sx / 2 + 5) * width / 10)
                    right = round((px + sx / 2 + 5) * width / 10)
                    top = round((5 - py - sy / 2) * height / 10)
                    bottom = round((5 - py + sy / 2) * height / 10)
                    cv2.rectangle(white, (left, top), (right, bottom), 0, -1)
            self.get_logger().info('已加载地面白线图和通行缺口；绕行保持车体与白线的间距')
            return cv2.distanceTransform(1 - white, cv2.DIST_L2, 5) * 10.0 / width
        except Exception as exc:
            self.get_logger().error(f'无法加载白线安全图，遇障碍将停车：{exc}')
            return None

    def white_line_clear_path(self, x, y, tx, ty):
        if self.white_distance is None:
            return False
        height, width = self.white_distance.shape
        distance = math.hypot(tx - x, ty - y)
        steps = max(2, math.ceil(distance / 0.05))
        for i in range(1, steps + 1):
            fraction = i / steps
            px = x + fraction * (tx - x)
            py = y + fraction * (ty - y)
            col = round((px + 5) * width / 10)
            row = round((5 - py) * height / 10)
            if not (0 <= row < height and 0 <= col < width):
                return False
            if self.white_distance[row, col] < 0.21:
                return False
        return True

    def stop(self):
        self.pub.publish(Twist())

    def front_distance(self):
        if self.scan is None:
            return 0.0
        values = []
        for i, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if abs(angle) <= 0.38 and math.isfinite(distance) and self.scan.range_min < distance < self.scan.range_max:
                values.append(distance)
        values.sort()
        return values[2] if len(values) >= 3 else (values[0] if values else float('inf'))

    def light_state(self, index):
        """Use image recognition when visible; reject stale or conflicting data."""
        now = self.get_clock().now().nanoseconds
        truth, truth_time = self.light_truth[index]
        vision, vision_time = self.light_vision[index]
        truth = truth if now - truth_time < 1_500_000_000 else 'unknown'
        vision = vision if now - vision_time < 1_000_000_000 else 'unknown'
        if truth in ('red', 'yellow') or vision in ('red', 'yellow'):
            return truth if truth in ('red', 'yellow') else vision
        if truth == 'green' or vision == 'green':
            return 'green'
        return 'unknown'

    def signal_gate(self, x, y):
        """Stop on the approach side, before the lamp and its white line."""
        if self.index in (1, 2) and abs(x - 3.95) < 0.43 and -1.52 < y < -0.35:
            return 1, y - (-1.22)
        if self.index == 6 and abs(y - 0.10) < 0.43 and -1.65 < x < -0.52:
            return 2, -0.82 - x
        return None, None

    def scan_points(self):
        """Laser points in the robot's forward-left coordinate frame."""
        points = []
        for i, distance in enumerate(self.scan.ranges):
            if not math.isfinite(distance) or not self.scan.range_min < distance < min(self.scan.range_max, 1.7):
                continue
            angle = self.scan.angle_min + i * self.scan.angle_increment
            points.append((distance * math.cos(angle), distance * math.sin(angle)))
        return points

    def bypass_target(self, x, y, heading, along, length, sx, sy, ux, uy):
        """Pick a clear side inside the surveyed lane; wait if neither fits."""
        if self.avoidance is None and abs(angle_diff(math.atan2(uy, ux), heading)) > 0.45:
            return False
        points = self.scan_points()
        remaining = max(0.0, length - along)
        obstacle = min((p for p in points if 0.18 < p[0] < min(1.25, remaining - 0.35)
                        and abs(p[1]) < 0.23 and
                        not self.static_obstacle_near(
                            x + math.cos(heading) * p[0] - math.sin(heading) * p[1],
                            y + math.sin(heading) * p[0] + math.cos(heading) * p[1])),
                       key=lambda p: p[0], default=None)
        if self.avoidance is not None:
            side, obstacle_along = self.avoidance
            if along > obstacle_along + 0.30:
                self.avoidance = None
        if self.avoidance is None and obstacle is not None:
            ox = x + math.cos(heading) * obstacle[0] - math.sin(heading) * obstacle[1]
            oy = y + math.sin(heading) * obstacle[0] + math.cos(heading) * obstacle[1]
            obstacle_along = (ox - sx) * ux + (oy - sy) * uy
            for side in (1, -1):
                target_along = min(length, max(0.0, along) + 0.55)
                tx = sx + ux * target_along - uy * side * 0.38
                ty = sy + uy * target_along + ux * side * 0.38
                if (self.side_clear(points, x, y, heading, tx, ty) and
                        self.white_line_clear_path(x, y, tx, ty)):
                    self.avoidance = (side, obstacle_along)
                    self.get_logger().info('检测到路线前方障碍：在车道内向%s绕行' % ('左' if side > 0 else '右'))
                    break
            if self.avoidance is None:
                self.get_logger().warn('障碍两侧空间不足，停车等待', throttle_duration_sec=2.0)
                return None
        if self.avoidance is None:
            return False
        side, _ = self.avoidance
        for lookahead in (0.55, 0.40, 0.25, 0.15):
            target_along = min(length, max(0.0, along) + lookahead)
            tx = sx + ux * target_along - uy * side * 0.38
            ty = sy + uy * target_along + ux * side * 0.38
            if (self.side_clear(points, x, y, heading, tx, ty) and
                    self.white_line_clear_path(x, y, tx, ty)):
                return tx, ty
        self.get_logger().warn('绕行方向或白线余量不足，停车等待', throttle_duration_sec=2.0)
        return None

    @staticmethod
    def side_clear(points, x, y, heading, tx, ty):
        dx, dy = tx - x, ty - y
        forward = math.cos(heading) * dx + math.sin(heading) * dy
        lateral = -math.sin(heading) * dx + math.cos(heading) * dy
        length2 = forward * forward + lateral * lateral
        if length2 < 0.01:
            return False
        for px, py in points:
            fraction = max(0.0, min(1.0, (px * forward + py * lateral) / length2))
            # The body is 0.20 m wide. Keep 0.22 m from the measured
            # obstacle surface, including turning and scan uncertainty.
            if fraction > 0.06 and math.hypot(px - forward * fraction,
                                               py - lateral * fraction) < 0.22:
                return False
        return True

    def request_plan(self):
        if (self.planning or not self.planner.server_is_ready() or
                self.get_clock().now().nanoseconds < self.next_plan_time_ns):
            return
        self.planning = True
        name, x, y, yaw = ROUTE[self.index]
        stamp = self.get_clock().now().to_msg()
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.planner_id = 'GridBased'
        goal.start.header.frame_id = 'map'
        goal.start.header.stamp = stamp
        goal.start.pose = self.pose
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = stamp
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = math.sin(yaw / 2)
        goal.goal.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info('航点 %d/%d：%s (%.2f, %.2f)，规划起点 (%.2f, %.2f)' %
                               (self.index + 1, len(ROUTE), name, x, y,
                                self.pose.position.x, self.pose.position.y))
        self.planner.send_goal_async(goal).add_done_callback(self.on_plan_accepted)

    def on_plan_accepted(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('规划目标被拒绝')
            handle.get_result_async().add_done_callback(self.on_plan_result)
        except Exception as exc:
            self.plan_failed(str(exc))

    def on_plan_result(self, future):
        try:
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED or len(result.result.path.poses) < 2:
                raise RuntimeError('无有效路径，状态码 %d' % result.status)
            self.path = [(p.pose.position.x, p.pose.position.y)
                         for p in result.result.path.poses]
            if any(self.distance_from_leg(x, y) > 0.35 for x, y in self.path):
                raise RuntimeError('规划路径偏离车道中心超过 0.35 m')
            self.plan_tries = 0
            self.planning = False
        except Exception as exc:
            self.plan_failed(str(exc))

    def plan_failed(self, reason):
        self.planning = False
        self.plan_tries += 1
        self.next_plan_time_ns = self.get_clock().now().nanoseconds + 2_000_000_000
        if '无有效路径' in reason or '偏离车道中心' in reason:
            self.get_logger().warn('当前路线被障碍挡住，已停车；2 秒后重新规划：%s' % reason,
                                   throttle_duration_sec=5.0)
            return
        if self.plan_tries > 2:
            self.get_logger().error('航点 %d 规划失败：%s' % (self.index + 1, reason))
            self.finished = True
        else:
            self.get_logger().warn('规划重试 %d/2：%s' % (self.plan_tries, reason))

    def tick(self):
        if self.finished:
            self.stop()
            return
        if self.pose is None or self.scan is None or self.pose_time is None:
            missing = '/amcl_pose' if self.pose is None else '/scan'
            self.get_logger().warn('导航等待 %s；请检查 Gazebo 传感器和 AMCL' % missing,
                                   throttle_duration_sec=8.0)
            return
        if (self.get_clock().now() - self.pose_time).nanoseconds > 2_000_000_000:
            self.stop()
            self.get_logger().warn('AMCL 位姿超过 2 秒未更新，已停车等待',
                                   throttle_duration_sec=8.0)
            return
        if self.scan_time is None or (self.get_clock().now() - self.scan_time).nanoseconds > 1_000_000_000:
            self.stop()
            self.get_logger().warn('激光扫描超过 1 秒未更新，已停车等待', throttle_duration_sec=5.0)
            return
        if not self.path:
            self.stop()
            if not self.planner.server_is_ready():
                self.get_logger().warn('导航等待 Nav2 路径规划器启动',
                                       throttle_duration_sec=8.0)
            self.request_plan()
            return

        x, y = self.pose.position.x, self.pose.position.y
        heading = yaw_of(self.pose.orientation)
        name, gx, gy, gyaw = ROUTE[self.index]
        distance_to_goal = math.hypot(gx - x, gy - y)
        light_id, gate_progress = self.signal_gate(x, y)
        if light_id is not None and gate_progress >= 0.0 and self.light_state(light_id - 1) != 'green':
            self.stop()
            self.get_logger().warn('红绿灯 %d 为 %s：停车等待绿灯' %
                                   (light_id, self.light_state(light_id - 1)),
                                   throttle_duration_sec=2.0)
            return
        if distance_to_goal < 0.14:
            self.stop()
            if abs(angle_diff(gyaw, heading)) < 0.22:
                self.get_logger().info('已到达：%s' % name)
                self.index += 1
                self.path = []
                if self.index == len(ROUTE):
                    self.get_logger().info('已通过两个红绿灯并回到起点，导航完成')
                    self.exit_code = 0
                    self.finished = True
                return
            command = Twist()
            command.angular.z = max(-0.55, min(0.55, 1.4 * angle_diff(gyaw, heading)))
            self.pub.publish(command)
            return

        # Keep the robot on the surveyed lane center instead of cutting a
        # corner between waypoints. Nav2 still checks each leg for obstacles.
        sx, sy, _, _ = self.leg()
        dx, dy = gx - sx, gy - sy
        length = math.hypot(dx, dy)
        along = ((x - sx) * dx + (y - sy) * dy) / max(length, 1e-6)
        target_along = min(length, max(0.0, along) + 0.35)
        tx, ty = sx + dx * target_along / length, sy + dy * target_along / length
        bypass = self.bypass_target(x, y, heading, along, length, sx, sy,
                                    dx / length, dy / length)
        if bypass is None:
            self.stop()
            return
        if bypass is not False:
            tx, ty = bypass
        if distance_to_goal < 0.45 and self.avoidance is None:
            tx, ty = gx, gy
        error = angle_diff(math.atan2(ty - y, tx - x), heading)
        command = Twist()
        if self.front_distance() < 0.32:
            self.get_logger().warn('前方障碍过近，停车等待', throttle_duration_sec=2.0)
        elif abs(error) > 0.45:
            command.angular.z = max(-0.75, min(0.75, 1.8 * error))
        else:
            command.linear.x = min(self.speed, max(0.06, 0.7 * distance_to_goal))
            if light_id is not None and gate_progress < 0 and self.light_state(light_id - 1) != 'green':
                command.linear.x = min(command.linear.x, 0.08)
            command.linear.x *= max(0.35, 1.0 - abs(error))
            command.angular.z = max(-0.75, min(0.75, 1.8 * error))
        self.pub.publish(command)


def main():
    rclpy.init()
    node = TrafficLightRoute()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.stop()
    code = node.exit_code
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
