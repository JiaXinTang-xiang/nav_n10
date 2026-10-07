#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class LocalizationWaiter(Node):
    def __init__(self):
        super().__init__('wait_for_localization')
        self.max_age_sec = float(self.declare_parameter('max_age_sec', 0.7).value)
        self.timeout_sec = float(self.declare_parameter('timeout_sec', 30.0).value)
        self.stable_samples = int(self.declare_parameter('stable_samples', 5).value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)

    def wait(self):
        started = time.monotonic()
        stable = 0
        last_report = 0.0
        while rclpy.ok() and time.monotonic() - started < self.timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self.buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
                now = self.get_clock().now().nanoseconds / 1e9
                age = now - stamp
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                valid = (
                    stamp > 0.0 and
                    -0.1 <= age <= self.max_age_sec and
                    all(math.isfinite(value) for value in (
                        translation.x, translation.y,
                        rotation.z, rotation.w)))
                stable = stable + 1 if valid else 0
                if valid and stable >= self.stable_samples:
                    self.get_logger().info(
                        f'定位 TF 已就绪: map->base_link age={age:.3f}s')
                    return True
                if time.monotonic() - last_report >= 2.0:
                    self.get_logger().info(
                        f'等待新鲜定位 TF: age={age:.3f}s, '
                        f'连续有效={stable}/{self.stable_samples}')
                    last_report = time.monotonic()
            except TransformException:
                stable = 0
                if time.monotonic() - last_report >= 2.0:
                    self.get_logger().info('等待 map->base_link 定位 TF')
                    last_report = time.monotonic()
        self.get_logger().error(
            f'定位 TF 未在 {self.timeout_sec:.1f}s 内达到新鲜度要求 '
            f'(最大允许 {self.max_age_sec:.2f}s)')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationWaiter()
    success = node.wait()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
