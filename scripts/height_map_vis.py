#!/usr/bin/env python3
import argparse
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from autonomy_light.msg import HeightMap

try:
    import cv2
except ImportError as exc:
    raise SystemExit(
        "OpenCV Python bindings are required for --vis. "
        "Install them with: sudo apt install python3-opencv"
    ) from exc


class HeightMapVis(Node):
    def __init__(self, topic):
        super().__init__("autonomy_light_height_map_vis")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._lock = threading.Lock()
        self._latest = None
        self._count = 0
        self.create_subscription(HeightMap, topic, self._on_height_map, qos)

    def _on_height_map(self, msg):
        with self._lock:
            self._latest = msg
            self._count += 1

    def latest(self):
        with self._lock:
            return self._latest, self._count


def parse_args():
    parser = argparse.ArgumentParser(description="Fast 2D HeightMap visualizer.")
    parser.add_argument("--topic", default="/autonomy_light/height_map_data")
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--scale", type=int, default=10)
    parser.add_argument("--window", default="autonomy_light height_map")
    parser.add_argument(
        "--raw-distance",
        action="store_true",
        help="Show raw HeightMap.data distance values instead of obstacle-bright height-like values.",
    )
    return parser.parse_args()


def message_to_image(msg, raw_distance):
    if msg.resolution <= 0.0:
        return None, "invalid resolution"

    width = int(math.ceil(msg.x_length / msg.resolution))
    height = int(math.ceil(msg.y_length / msg.resolution))
    expected = width * height
    if width <= 0 or height <= 0 or len(msg.data) < expected:
        return None, f"invalid grid {width}x{height} len={len(msg.data)}"

    raw = np.asarray(msg.data[:expected], dtype=np.float32).reshape((height, width))
    finite = np.isfinite(raw)
    if not finite.any():
        return None, f"{width}x{height} all invalid"

    values = raw.copy()
    values[~finite] = np.nanmax(values[finite])
    if raw_distance:
        display = values
    else:
        display = np.nanmax(values[finite]) - values

    display = np.flipud(display.T)
    low = float(np.nanmin(display))
    high = float(np.nanmax(display))
    span = max(1.0e-6, high - low)
    gray = np.clip((display - low) * (255.0 / span), 0.0, 255.0).astype(np.uint8)
    image = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    stats = f"{width}x{height} raw=[{float(np.nanmin(raw[finite])):.3f},{float(np.nanmax(raw[finite])):.3f}]"
    return image, stats


def draw_overlay(image, text):
    cv2.rectangle(image, (0, 0), (image.shape[1], 44), (0, 0, 0), -1)
    cv2.putText(
        image,
        text,
        (8, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "+x forward up, obstacle bright",
        (8, 38),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def main():
    args = parse_args()
    rclpy.init()
    node = HeightMapVis(args.topic)
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)

    period = 1.0 / max(1.0, args.fps)
    last_count = 0
    last_rate_time = time.monotonic()
    measured_hz = 0.0

    try:
        while rclpy.ok():
            start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            msg, count = node.latest()

            if msg is None:
                image = np.zeros((320, 640, 3), dtype=np.uint8)
                draw_overlay(image, f"waiting for {args.topic}")
            else:
                now = time.monotonic()
                elapsed = now - last_rate_time
                if elapsed >= 0.5:
                    measured_hz = (count - last_count) / elapsed
                    last_count = count
                    last_rate_time = now

                image, stats = message_to_image(msg, args.raw_distance)
                if image is None:
                    image = np.zeros((320, 640, 3), dtype=np.uint8)
                else:
                    image = cv2.resize(
                        image,
                        None,
                        fx=max(1, args.scale),
                        fy=max(1, args.scale),
                        interpolation=cv2.INTER_NEAREST,
                    )
                stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
                draw_overlay(image, f"{args.topic} {measured_hz:4.1f}Hz stamp={stamp} {stats}")

            cv2.imshow(args.window, image)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

            sleep_s = period - (time.monotonic() - start)
            if sleep_s > 0.0:
                time.sleep(sleep_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
