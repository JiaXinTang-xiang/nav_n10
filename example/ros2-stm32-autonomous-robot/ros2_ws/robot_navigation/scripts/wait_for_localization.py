#!/usr/bin/python3
"""Boot ordering only: wait for the existing map -> base_link TF authority."""
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    rclpy.init()
    node = Node('navigation_startup_wait')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    node.get_logger().info('Waiting for initialized localization (RViz 2D Pose Estimate); no motion')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                tf = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                stamp = tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec
                age = node.get_clock().now().nanoseconds - stamp
                if stamp > 0 and -50_000_000 <= age <= 700_000_000:
                    node.get_logger().info('Existing localization TF ready; starting native Nav2')
                    return
            except TransformException:
                pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
