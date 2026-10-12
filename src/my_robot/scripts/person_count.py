#!/usr/bin/env python3
"""Count the people printed on each board when the route faces it."""

import os
from statistics import median

import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String
from ultralytics import YOLO
from vision_lighting import enhance_lighting


BOARD_IDS = {'A': 1, 'B': 2}


def _same_figure(a, b):
    """Discard nested or near-identical boxes around one printed figure."""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    overlap_w = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    overlap_h = max(0.0, min(ay2, by2) - max(ay1, by1))
    smaller_area = min((ax2 - ax1) * (ay2 - ay1),
                       (bx2 - bx1) * (by2 - by1))
    if smaller_area > 0 and overlap_w * overlap_h / smaller_area >= 0.65:
        return True
    center_gap = abs((ax1 + ax2) - (bx1 + bx2)) / 2
    shorter_height = min(ay2 - ay1, by2 - by1)
    return (center_gap < 0.4 * min(ax2 - ax1, bx2 - bx1)
            and overlap_h > 0.65 * shorter_height)


def count_people(model, image):
    """Count separate, full-height figures sharing the board's baseline."""
    height, width = image.shape[:2]
    result = model(image, classes=[0], conf=0.08, imgsz=1280, verbose=False)[0]
    candidates = []
    for box in result.boxes:
        x1, y1, x2, y2 = map(float, box.xyxy[0])
        center_x = (x1 + x2) / 2
        if (0.05 * width < center_x < 0.95 * width
                and y2 - y1 > 0.25 * height
                and y1 < 0.60 * height
                and y2 > 0.65 * height):
            candidates.append(((x1, y1, x2, y2), float(box.conf[0])))
    if not candidates:
        return 0
    baseline = median(coords[3] for coords, _ in candidates)
    figures = []
    for coords, _ in sorted(candidates, key=lambda candidate: -candidate[1]):
        if abs(coords[3] - baseline) > 0.10 * height:
            continue
        if not any(_same_figure(coords, existing) for existing in figures):
            figures.append(coords)
    return len(figures)


class PersonCounter(Node):
    def __init__(self):
        super().__init__('person_counter')
        model_file = os.path.join(get_package_share_directory('my_robot'),
                                  'models', 'person_yolov8n.pt')
        torch.set_num_threads(2)
        self.model = YOLO(model_file)
        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.latest_image_ns = 0
        self.latest_frame_seq = 0
        self.last_inference_frame_seq = -1
        self.last_inference_ns = 0
        try:
            count_people(self.model, np.zeros((576, 768, 3), dtype=np.uint8))
            self.get_logger().info('人偶模型已完成首次推理准备')
        except Exception as exc:
            self.get_logger().warn(f'人偶模型首次推理失败：{exc}')
        self.result_pub = self.create_publisher(Int32MultiArray, '/person_count/result', 10)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, image_qos)
        self.create_subscription(String, '/person_count/request', self.request_callback, 1)
        self.get_logger().info('人偶立牌人数识别已启动；等待路线停靠 A、B 立牌')

    def image_callback(self, msg):
        self.latest_image_msg = msg
        self.latest_image_ns = self.get_clock().now().nanoseconds
        self.latest_frame_seq += 1

    def request_callback(self, msg):
        if msg.data not in BOARD_IDS:
            return
        now_ns = self.get_clock().now().nanoseconds
        if (self.latest_image_msg is None or now_ns - self.latest_image_ns > 1_000_000_000
                or now_ns - self.last_inference_ns < 700_000_000
                or self.latest_frame_seq == self.last_inference_frame_seq):
            return
        self.last_inference_ns = now_ns
        self.last_inference_frame_seq = self.latest_frame_seq
        try:
            image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
            count = count_people(self.model, enhance_lighting(image))
        except Exception as exc:
            self.get_logger().warn(f'人偶识别失败：{exc}', throttle_duration_sec=5.0)
            return
        self.result_pub.publish(Int32MultiArray(data=[BOARD_IDS[msg.data], count]))
        self.get_logger().info(f'{msg.data} 立牌检测到 {count} 人', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PersonCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
