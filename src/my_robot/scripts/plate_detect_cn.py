#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基础车牌检测节点，不带朝向判断、不带锁定
订阅图像，YOLO检测车牌框，输出检测框话题 /plate_detect
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os

class PlateDetectCNNode(Node):
    def __init__(self):
        super().__init__('plate_detect_cn')
        # 增加可视化开关参数，launch可以传 use_gui:=true/false
        self.declare_parameter('use_gui', True)
        self.use_gui = self.get_parameter('use_gui').get_parameter_value().bool_value

        self.bridge = CvBridge()
        pkg_dir = get_package_share_directory('my_robot')
        model_path = os.path.join(pkg_dir, 'models', 'plate_detect.pt')
        self.model = YOLO(model_path)

        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.img_sub = self.create_subscription(
            Image, "/camera/image_raw", self.img_callback, image_qos)
        self.plate_pub = self.create_publisher(Float32MultiArray, "/plate_detect", 10)
        self.get_logger().info("✅车牌检测节点启动，无角度/锁定限制")

    def img_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {e}")
            return

        detect_msg = Float32MultiArray()
        # 默认全0，代表无车牌
        detect_msg.data = [0.0,0.0,0.0,0.0,0.0]
        img_h, img_w = frame.shape[:2]
        nearest_center = img_w * 0.25

        results = self.model(frame, conf=0.3)
        for res in results:
            boxes = res.boxes
            for box in boxes:
                x1,y1,x2,y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                # 边界限制，防止坐标越界
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(img_w - 1, x2)
                y2 = min(img_h - 1, y2)

                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.putText(frame,f"plate {conf:.2f}",(x1,y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
                # 同一画面可见三辆车时，只发布画面中心那辆车的车牌框。
                center_offset = abs((x1 + x2) / 2 - img_w / 2)
                if center_offset < nearest_center:
                    nearest_center = center_offset
                    detect_msg.data = [float(x1),float(y1),float(x2),float(y2),conf]

        self.plate_pub.publish(detect_msg)

        # 只有开启GUI才打开opencv窗口
        if self.use_gui:
            cv2.imshow("cn_plate_view", frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PlateDetectCNNode()
    rclpy.spin(node)
    node.destroy_node()
    # 销毁窗口
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
