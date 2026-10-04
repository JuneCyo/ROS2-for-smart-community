#!/usr/bin/env python3
"""Read the text in the centered plate box and publish confirmed OCR candidates."""

import math
import re

import cv2
import rclpy
from cv_bridge import CvBridge
from rapidocr import RapidOCR
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


PROVINCES = '京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新'
PLATE_PATTERN = re.compile(rf'[{PROVINCES}][A-Z][A-Z0-9]{{5,6}}')


def normalize_plate(text):
    """Accept only a complete Chinese plate; never turn a partial OCR result into a plate."""
    cleaned = re.sub(r'[\s·•.\-]', '', text).upper()
    return cleaned if PLATE_PATTERN.fullmatch(cleaned) else None


def crop_plate(frame, detection):
    """The detector publishes pixel coordinates, with a small border for OCR."""
    if len(detection) != 5:
        return None
    x1, y1, x2, y2, confidence = detection
    if confidence < 0.3 or x2 <= x1 or y2 <= y1:
        return None
    height, width = frame.shape[:2]
    pad_x = max(2, round((x2 - x1) * 0.05))
    pad_y = max(2, round((y2 - y1) * 0.10))
    left = max(0, math.floor(x1) - pad_x)
    top = max(0, math.floor(y1) - pad_y)
    right = min(width, math.ceil(x2) + pad_x)
    bottom = min(height, math.ceil(y2) + pad_y)
    crop = frame[top:bottom, left:right]
    return crop if crop.size else None


class PlateRecognizer(Node):
    def __init__(self):
        super().__init__('plate_recognizer')
        self.bridge = CvBridge()
        self.ocr = RapidOCR()
        self.latest_image = None
        self.latest_image_ns = 0
        self.last_ocr_ns = 0
        self.plate_pub = self.create_publisher(String, '/plate_text', 10)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, image_qos)
        self.create_subscription(Float32MultiArray, '/plate_detect', self.box_callback, 1)
        self.get_logger().info('车牌文字识别已启动，结果发布到 /plate_text')

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image_ns = self.get_clock().now().nanoseconds
        except Exception as exc:
            self.get_logger().warn(f'车牌识别图像转换失败：{exc}', throttle_duration_sec=5.0)

    def box_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if (self.latest_image is None or now_ns - self.latest_image_ns > 1_000_000_000
                or now_ns - self.last_ocr_ns < 350_000_000):
            return
        crop = crop_plate(self.latest_image, msg.data)
        if crop is None:
            return
        self.last_ocr_ns = now_ns
        scale = max(1, min(8, math.ceil(240 / crop.shape[1])))
        enlarged = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        try:
            result = self.ocr(enlarged, use_det=False, use_cls=False)
        except Exception as exc:
            self.get_logger().warn(f'车牌 OCR 失败：{exc}', throttle_duration_sec=5.0)
            return
        if not result.txts or not result.scores:
            return
        plate_text = normalize_plate(result.txts[0])
        if plate_text is None or result.scores[0] < 0.75:
            return
        self.plate_pub.publish(String(data=plate_text))
        self.get_logger().info(f'读到车牌：{plate_text}，OCR 置信度 {result.scores[0]:.2f}',
                               throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PlateRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
