#!/usr/bin/env python3
"""Read the text in the centered plate box and publish confirmed OCR candidates."""

import re

import cv2
import rclpy
from cv_bridge import CvBridge
from rapidocr import RapidOCR
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_lighting import enhance_lighting


PROVINCES = '京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新'
PLATE_PATTERN = re.compile(rf'[{PROVINCES}][A-Z][A-Z0-9]{{5,6}}')


def normalize_plate(text):
    """Accept only a complete Chinese plate; never turn a partial OCR result into a plate."""
    cleaned = re.sub(r'[\s·•.\-]', '', text).upper()
    return cleaned if PLATE_PATTERN.fullmatch(cleaned) else None


class PlateRecognizer(Node):
    def __init__(self):
        super().__init__('plate_recognizer')
        self.bridge = CvBridge()
        self.ocr = RapidOCR()
        self.last_ocr_ns = 0
        self.plate_pub = self.create_publisher(String, '/plate_text', 10)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/plate_crop', self.crop_callback, image_qos)
        self.get_logger().info('车牌文字识别已启动，结果发布到 /plate_text')

    def crop_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_ocr_ns < 350_000_000:
            return
        try:
            crop = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'车牌识别图像转换失败：{exc}', throttle_duration_sec=5.0)
            return
        if crop.size == 0:
            return
        self.last_ocr_ns = now_ns
        scale = max(1, min(8, (240 + crop.shape[1] - 1) // crop.shape[1]))
        enlarged = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        corrected = enhance_lighting(enlarged)
        variants = (corrected, enlarged) if corrected is not enlarged else (
            enlarged, enhance_lighting(enlarged, force=True))
        for variant in variants:
            try:
                result = self.ocr(variant, use_det=False, use_cls=False)
            except Exception as exc:
                self.get_logger().warn(f'车牌 OCR 失败：{exc}', throttle_duration_sec=5.0)
                continue
            texts = result.txts if result.txts is not None else ()
            scores = result.scores if result.scores is not None else ()
            for text, score in zip(texts, scores):
                plate_text = normalize_plate(text)
                if plate_text is None or score < 0.75:
                    continue
                self.plate_pub.publish(String(data=plate_text))
                self.get_logger().info(f'读到车牌：{plate_text}，OCR 置信度 {score:.2f}',
                                       throttle_duration_sec=5.0)
                return


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
