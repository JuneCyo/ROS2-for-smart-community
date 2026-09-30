#!/usr/bin/env python3
"""AMCL-localized closed route through both traffic-light openings.

Nav2 plans each leg from the live AMCL pose. A small pure-pursuit controller
tracks that plan on /cmd_vel because this Gazebo/Humble Nav2 controller can
continue driving after its internal TF cache stops updating.
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


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
        if distance_to_goal < 0.45:
            tx, ty = gx, gy
        error = angle_diff(math.atan2(ty - y, tx - x), heading)
        command = Twist()
        if self.front_distance() < 0.32:
            self.get_logger().warn('前方障碍过近，停车等待', throttle_duration_sec=2.0)
        elif abs(error) > 0.45:
            command.angular.z = max(-0.75, min(0.75, 1.8 * error))
        else:
            command.linear.x = min(self.speed, max(0.06, 0.7 * distance_to_goal))
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
