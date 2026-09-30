#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 ROS 图像话题抓一帧存成 png:  grab_image.py <topic> <out.png>"""
import sys
import time

import numpy as np
import rclpy
from PIL import Image as PILImage
from sensor_msgs.msg import Image


def main():
    topic, out = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node('grab_image')
    box = {}

    def cb(msg):
        if 'm' not in box:
            box['m'] = msg

    node.create_subscription(Image, topic, cb, 10)
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < 40.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    m = box.get('m')
    if m is None:
        print('NO_IMAGE on', topic)
        return 1
    arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
    arr = arr[:, :m.width * 3].reshape(m.height, m.width, 3)
    if m.encoding == 'bgr8':
        arr = arr[:, :, ::-1]
    PILImage.fromarray(arr).save(out)
    print('SAVED %s %dx%d enc=%s mean=%.2f' % (out, m.width, m.height, m.encoding, arr.mean()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
