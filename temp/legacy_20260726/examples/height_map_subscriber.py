#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from autonomy_light.msg import HeightMap


class HeightMapSubscriber(Node):
    def __init__(self):
        super().__init__("autonomy_light_height_map_subscriber_example")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.last_print_time = 0.0
        self.create_subscription(
            HeightMap,
            "/autonomy_light/height_map_data",
            self.on_height_map,
            qos,
        )

    def on_height_map(self, msg):
        now = time.monotonic()
        if now - self.last_print_time < 1.0:
            return
        self.last_print_time = now

        width = int(math.ceil(msg.x_length / msg.resolution)) if msg.resolution > 0.0 else 0
        height = int(math.ceil(msg.y_length / msg.resolution)) if msg.resolution > 0.0 else 0
        values = list(msg.data)
        if values:
            value_range = f"min={min(values):.3f} max={max(values):.3f}"
        else:
            value_range = "empty"
        self.get_logger().info(
            f"height_map frame={msg.header.frame_id} "
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
            f"{width}x{height} res={msg.resolution:.3f} "
            f"len={len(values)} {value_range}"
        )


def main():
    rclpy.init()
    node = HeightMapSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
