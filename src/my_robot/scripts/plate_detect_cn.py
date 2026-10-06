#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Display the live camera stream and publish the centered YOLO plate box."""

import os
import threading
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO


WINDOW_NAME = 'cn_plate_view'
DETECTION_PERIOD = 0.30  # Leave CPU time for Gazebo and navigation.


def detect_plates(model, frame):
    height, width = frame.shape[:2]
    boxes = []
    nearest_center = width * 0.25
    selected = [0.0] * 5
    for result in model(frame, conf=0.3, verbose=False):
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width - 1, x2), min(height - 1, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            confidence = float(box.conf[0])
            boxes.append((x1, y1, x2, y2, confidence))
            center_offset = abs((x1 + x2) / 2 - width / 2)
            if center_offset < nearest_center:
                nearest_center = center_offset
                selected = [float(x1), float(y1), float(x2), float(y2), confidence]
    return boxes, selected


class PlateDetectCNNode(Node):
    def __init__(self):
        super().__init__('plate_detect_cn')
        self.declare_parameter('use_gui', True)
        self.use_gui = bool(self.get_parameter('use_gui').value)
        self.bridge = CvBridge()
        model_path = os.path.join(get_package_share_directory('my_robot'),
                                  'models', 'plate_detect.pt')
        self.model = YOLO(model_path)

        self.lock = threading.Lock()
        self.new_image = threading.Event()
        self.stop_worker = threading.Event()
        self.latest_frame = None
        self.frame_seq = 0
        self.boxes = []
        self.selected = [0.0] * 5
        self.result_seq = 0
        self.result_time = 0.0
        self.published_seq = 0
        self.display_state = (-1, -1)

        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.img_callback, image_qos)
        self.plate_pub = self.create_publisher(Float32MultiArray, '/plate_detect', 10)
        self.create_timer(0.04, self.update_display_and_publish)
        self.worker = threading.Thread(target=self.detect_loop, name='plate_yolo', daemon=True)
        self.worker.start()
        self.get_logger().info('车牌检测已启动；画面实时刷新，后台按最新图像检测')

    def img_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'图像转换失败：{exc}', throttle_duration_sec=5.0)
            return
        with self.lock:
            self.latest_frame = frame
            self.frame_seq += 1
        self.new_image.set()

    def detect_loop(self):
        last_frame_seq = -1
        next_inference = 0.0
        while not self.stop_worker.is_set():
            if not self.new_image.wait(timeout=0.1):
                continue
            self.new_image.clear()
            delay = next_inference - time.monotonic()
            if delay > 0 and self.stop_worker.wait(delay):
                break
            with self.lock:
                frame, frame_seq = self.latest_frame, self.frame_seq
            if frame is None or frame_seq == last_frame_seq:
                continue
            last_frame_seq = frame_seq
            next_inference = time.monotonic() + DETECTION_PERIOD
            try:
                boxes, selected = detect_plates(self.model, frame)
            except Exception as exc:
                self.get_logger().warn(f'车牌检测失败：{exc}', throttle_duration_sec=5.0)
                continue
            with self.lock:
                self.boxes = boxes
                self.selected = selected
                self.result_seq += 1
                self.result_time = time.monotonic()

    def update_display_and_publish(self):
        with self.lock:
            frame = self.latest_frame
            frame_seq = self.frame_seq
            boxes = self.boxes
            selected = self.selected
            result_seq = self.result_seq
            result_time = self.result_time

        if result_seq > self.published_seq:
            self.plate_pub.publish(Float32MultiArray(data=selected))
            self.published_seq = result_seq

        if not self.use_gui or frame is None:
            return
        state = (frame_seq, result_seq)
        if state != self.display_state:
            view = frame.copy()
            if time.monotonic() - result_time < 0.6:
                for x1, y1, x2, y2, confidence in boxes:
                    cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(view, f'plate {confidence:.2f}',
                                (x1, max(18, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 0), 2)
            cv2.imshow(WINDOW_NAME, view)
            self.display_state = state
        cv2.waitKey(1)

    def close(self):
        self.stop_worker.set()
        self.new_image.set()
        self.worker.join(timeout=2.0)
        if self.use_gui:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = PlateDetectCNNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
