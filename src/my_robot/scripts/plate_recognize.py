#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plate Recognizer: LPRNet 国内蓝牌车牌识别
订阅：相机图像 /camera/image_raw  + YOLO检测框 /plate_detect
发布识别结果到 /plate_text
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.nn as nn

# ====================== 国内车牌完整字符集（蓝牌7位） ======================
CHARS = [
    # 省份
    "京", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑",
    "苏", "浙", "皖", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤",
    "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁", "新",
    # 字母数字
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J",
    "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T",
    "U", "V", "W", "X", "Y", "Z"
]
CHAR_NUM = len(CHARS)
BLANK_INDEX = CHAR_NUM # CTC空白符

# ====================== 标准LPRNet  ======================
class LPRNet(nn.Module):
    def __init__(self, class_num):
        super(LPRNet, self).__init__()
        self.backbone = nn.Sequential(
            nn.Conv2d(3, 64, 3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),

            nn.Conv2d(64, 128, 3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),

            nn.Conv2d(128, 256, 3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(256, 256, 3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=(2,1), stride=(2,1)),

            nn.Conv2d(256, 512, 3, stride=1, padding=1),
            nn.BatchNorm2d(512),
            nn.ReLU(),
            nn.Conv2d(512, 512, 3, stride=1, padding=1),
            nn.BatchNorm2d(512),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=(2,1), stride=(2,1)),

            nn.Conv2d(512, 512, 3, stride=1, padding=1),
            nn.BatchNorm2d(512),
            nn.ReLU(),
        )
        self.container = nn.Sequential(
            nn.Conv2d(512, 512, 3, stride=1, padding=1),
            nn.BatchNorm2d(512),
            nn.ReLU(),
            nn.Conv2d(512, class_num+1, kernel_size=1, stride=1, padding=0),
        )

    def forward(self, x):
        x = self.backbone(x)
        x = self.container(x)
        x = x.permute(2,0,3,1)
        return x

# CTC解码：去重复，去除空白字符
def ctc_decode(pred, blank_idx):
    pred_idx = np.argmax(pred, axis=1)
    res = []
    last = blank_idx
    for idx in pred_idx:
        if idx != blank_idx and idx != last:
            res.append(CHARS[idx])
        last = idx
    return "".join(res)


class PlateRecognizer(Node):
    def __init__(self):
        super().__init__('plate_recognizer')
        self.bridge = CvBridge()
        self.latest_cv_img = None

        # 订阅相机图像 和 YOLO输出车牌框
        self.img_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.box_sub = self.create_subscription(
            Float32MultiArray, "/plate_detect", self.box_callback, 10
        )
        # 发布识别车牌文本
        self.plate_pub = self.create_publisher(String, "/plate_text", 10)

        # CPU/GPU自动选择
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = LPRNet(class_num=CHAR_NUM).to(self.device)

        # =========【重要】加载预训练权重 =========
        # 你需要下载国内车牌预训练lprnet.pth，放到scripts目录
        weight_path = "/home/voilet/ros2_ws/src/my_robot/scripts/lprnet.pth"
        try:
            ckpt = torch.load(weight_path, map_location=self.device)
            self.model.load_state_dict(ckpt)
            self.get_logger().info(f"成功加载LPRNet预训练权重：{weight_path}")
        except Exception as e:
            self.get_logger().error(f"权重加载失败！检查路径：{e}")

        self.model.eval()
        self.get_logger().info("LPRNet车牌识别节点就绪，等待YOLO检测框")

    def image_callback(self, msg):
        try:
            self.latest_cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"图像转换失败 {e}")

    def box_callback(self, msg):
        if self.latest_cv_img is None:
            return
        h_img, w_img = self.latest_cv_img.shape[:2]
        data = msg.data
        x1, y1, x2, y2, conf = data

        # YOLO输出归一化坐标转像素坐标
        x1 = max(0, int(x1 * w_img))
        y1 = max(0, int(y1 * h_img))
        x2 = min(w_img, int(x2 * w_img))
        y2 = min(h_img, int(y2 * h_img))

        plate_crop = self.latest_cv_img[y1:y2, x1:x2]
        if plate_crop.size == 0:
            return

        # LPRNet固定输入尺寸 (94,24)
        plate_crop = cv2.resize(plate_crop, (94, 24))
        # 归一化 +转tensor
        img_tensor = torch.from_numpy(plate_crop).float()
        img_tensor = img_tensor.permute(2, 0, 1) / 255.0
        img_tensor = img_tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(img_tensor)
        output = output.squeeze().detach().cpu().numpy()
        plate_result = ctc_decode(output, BLANK_INDEX)

        # 发布
        out_msg = String()
        out_msg.data = plate_result
        self.plate_pub.publish(out_msg)
        self.get_logger().info(f"识别车牌: {plate_result}, 检测置信度:{conf:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = PlateRecognizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

