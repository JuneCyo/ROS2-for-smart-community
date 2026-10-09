#!/usr/bin/env python3
"""Recognize the active simulated traffic lamp from the forward camera."""
import math

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_lighting import enhance_lighting


LIGHTS = ((4.0, 0.0), (-2.0, 0.25))


def detect_lamp(image, x, y, yaw, light_id):
    """Return a lamp color or unknown using the known housing projection."""
    # 只在灯架投影足够可靠的距离内判断灯色。
    target_x, target_y = LIGHTS[light_id - 1]
    dist = math.hypot(x - target_x, y - target_y)
    dist_threshold = 2.3
    if dist > dist_threshold:
        return 'unknown'
    
    height, width = image.shape[:2]
    fx = width / (2.0 * math.tan(0.8))  # sensor horizontal FOV = 1.6 rad
    dx, dy = LIGHTS[light_id - 1][0] - x, LIGHTS[light_id - 1][1] - y
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy - 0.162
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    dz = 1.7 - 0.082
    camera_forward = math.cos(0.4) * forward + math.sin(0.4) * dz
    camera_up = -math.sin(0.4) * forward + math.cos(0.4) * dz
    if forward < 0.35 or camera_forward < 0.5:
        return 'unknown'
    u = width / 2.0 - fx * lateral / camera_forward
    v = height / 2.0 - fx * camera_up / camera_forward
    half_width = fx * 0.46 / camera_forward + 10
    half_height = fx * 0.25 / camera_forward + 10
    x0, x1 = max(0, int(u - half_width)), min(width, int(u + half_width))
    y0, y1 = max(0, int(v - half_height)), min(height, int(v + half_height))
    if x1 - x0 < 8 or y1 - y0 < 8:
        return 'unknown'
    hsv = cv2.cvtColor(enhance_lighting(image[y0:y1, x0:x1]), cv2.COLOR_BGR2HSV)
    masks = {
        'red': cv2.inRange(hsv, (0, 110, 65), (10, 255, 255)) |
               cv2.inRange(hsv, (170, 110, 65), (179, 255, 255)),
        'yellow': cv2.inRange(hsv, (16, 110, 65), (36, 255, 255)),
        'green': cv2.inRange(hsv, (37, 110, 65), (90, 255, 255)),
    }
    max_disc_size = max(45, int(1.7 * fx * 0.26 / camera_forward))
    scores = {}
    for name, mask in masks.items():
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        scores[name] = max((stats[i, cv2.CC_STAT_AREA] for i in range(1, count)
                            if 2 <= stats[i, cv2.CC_STAT_WIDTH] <= max_disc_size and
                            2 <= stats[i, cv2.CC_STAT_HEIGHT] <= max_disc_size), default=0)
    color = max(scores, key=scores.get)
    return color if scores[color] >= 7 else 'unknown'


class TrafficLightVision(Node):
    def __init__(self):
        super().__init__('traffic_light_vision')
        self.bridge = CvBridge()
        self.pose = None
        self.yaw = 0.0
        self.last_image_ns = 0
        self.last_colors = ['unknown', 'unknown']
        self.light_publishers = [self.create_publisher(
            String, f'/traffic_light/light{i}/vision_state', 10) for i in (1, 2)]
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image,
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info('相机灯色识别已启动：只检测已知灯架附近的红/黄/绿圆盘')

    def on_pose(self, msg):
        pose = msg.pose.pose
        self.pose = (pose.position.x, pose.position.y)
        q = pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_image(self, msg):
        now = self.get_clock().now().nanoseconds
        if self.pose is None or now - self.last_image_ns < 180_000_000:
            return
        self.last_image_ns = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'相机图像格式错误：{exc}', throttle_duration_sec=5.0)
            return
        for i in (1, 2):
            color = detect_lamp(image, *self.pose, self.yaw, i)
            self.light_publishers[i - 1].publish(String(data=color))
            if color != 'unknown' and color != self.last_colors[i - 1]:
                self.get_logger().info(f'相机识别红绿灯 {i}：{color}')
            self.last_colors[i - 1] = color


def main():
    rclpy.init()
    node = TrafficLightVision()
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
