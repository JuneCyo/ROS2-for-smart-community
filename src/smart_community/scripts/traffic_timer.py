#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityVisual

class TrafficTimerNode(Node):
    def __init__(self):
        super().__init__('traffic_timer_node')
        self.client = self.create_client(SetEntityVisual, '/gz/set_entity_visual')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待 /gz/set_entity_visual 服务...")

        self.l1_red = "traffic_light1::light_red::red_vis"
        self.l1_yellow = "traffic_light1::light_yellow::yellow_vis"
        self.l1_green = "traffic_light1::light_green::green_vis"
        self.l2_red = "traffic_light2::light_red::red_vis"
        self.l2_yellow = "traffic_light2::light_yellow::yellow_vis"
        self.l2_green = "traffic_light2::light_green::green_vis"

        # 与 Gazebo Classic 插件保持相同的红、绿、黄周期。
        self.state = 0
        self.durations = [10.0, 15.0, 3.0]

        self.set_all_lights(
            l1_red=[1,0,0,1], l1_yellow=[0,0,0,1], l1_green=[0,0,0,1],
            l2_red=[1,0,0,1], l2_yellow=[0,0,0,1], l2_green=[0,0,0,1]
        )
        self.timer = self.create_timer(self.durations[self.state], self.timer_callback)

    def set_visual_color(self, entity_name, rgba):
        req = SetEntityVisual.Request()
        name_split = entity_name.split("::")
        req.entity_name = name_split[0]
        req.visual_name = name_split[-1]
        req.material.diffuse.r = rgba[0]
        req.material.diffuse.g = rgba[1]
        req.material.diffuse.b = rgba[2]
        req.material.diffuse.a = rgba[3]
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def set_all_lights(self, l1_red, l1_yellow, l1_green, l2_red, l2_yellow, l2_green):
        self.set_visual_color(self.l1_red, l1_red)
        self.set_visual_color(self.l1_yellow, l1_yellow)
        self.set_visual_color(self.l1_green, l1_green)
        self.set_visual_color(self.l2_red, l2_red)
        self.set_visual_color(self.l2_yellow, l2_yellow)
        self.set_visual_color(self.l2_green, l2_green)

    def timer_callback(self):
        self.state = (self.state + 1) % 3
        self.timer.cancel()
        if self.state == 0:
            self.set_all_lights(
                [1,0,0,1],[0,0,0,1],[0,0,0,1],
                [1,0,0,1],[0,0,0,1],[0,0,0,1]
            )
        elif self.state == 1:
            self.set_all_lights(
                [0,0,0,1],[0,0,0,1],[0,1,0,1],
                [0,0,0,1],[0,0,0,1],[0,1,0,1]
            )
        elif self.state == 2:
            self.set_all_lights(
                [0,0,0,1],[1,1,0,1],[0,0,0,1],
                [0,0,0,1],[1,1,0,1],[0,0,0,1]
            )
        self.timer = self.create_timer(self.durations[self.state], self.timer_callback)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficTimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

