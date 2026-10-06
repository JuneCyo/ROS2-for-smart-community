#!/usr/bin/env python3
"""Count the people printed on each board when the route faces it."""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String
from ultralytics import YOLO


BOARD_IDS = {'A': 1, 'B': 2}


def count_people(model, image):
    """Detect visible full-height figures on the board, excluding distant scenery."""
    height, width = image.shape[:2]
    result = model(image, classes=[0], conf=0.15, imgsz=1280, verbose=False)[0]
    count = 0
    for box in result.boxes:
        x1, y1, x2, y2 = map(float, box.xyxy[0])
        center_x = (x1 + x2) / 2
        if (0.05 * width < center_x < 0.95 * width
                and y2 - y1 > 0.25 * height
                and y1 < 0.60 * height
                and y2 > 0.65 * height):
            count += 1
    return count


class PersonCounter(Node):
    def __init__(self):
        super().__init__('person_counter')
        model_file = os.path.join(get_package_share_directory('my_robot'),
                                  'models', 'person_yolov8n.pt')
        self.model = YOLO(model_file)
        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.latest_image_ns = 0
        self.last_inference_ns = 0
        self.result_pub = self.create_publisher(Int32MultiArray, '/person_count/result', 10)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, image_qos)
        self.create_subscription(String, '/person_count/request', self.request_callback, 10)
        self.get_logger().info('人偶立牌人数识别已启动；等待路线停靠 A、B 立牌')

    def image_callback(self, msg):
        # Keep only the newest camera message; image conversion is needed only
        # at a board stop, not for every frame during the whole route.
        self.latest_image_msg = msg
        self.latest_image_ns = self.get_clock().now().nanoseconds

    def request_callback(self, msg):
        if msg.data not in BOARD_IDS:
            return
        now_ns = self.get_clock().now().nanoseconds
        if (self.latest_image_msg is None or now_ns - self.latest_image_ns > 1_000_000_000
                or now_ns - self.last_inference_ns < 700_000_000):
            return
        self.last_inference_ns = now_ns
        try:
            image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
            count = count_people(self.model, image)
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
