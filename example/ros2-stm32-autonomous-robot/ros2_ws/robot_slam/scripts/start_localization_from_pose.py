#!/usr/bin/python3
"""Start one localization trajectory from an explicit map-frame RViz pose."""

import math

import rclpy
from cartographer_ros_msgs.srv import StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class StartLocalizationFromPose(Node):
    def __init__(self):
        super().__init__("start_localization_from_pose")
        self.directory = self.declare_parameter("configuration_directory", "").value
        self.rx = self.declare_parameter("reference_x", 0.0).value
        self.ry = self.declare_parameter("reference_y", 0.0).value
        self.ryaw = self.declare_parameter("reference_yaw", 0.0).value
        self.client = self.create_client(StartTrajectory, "/start_trajectory")
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_pose, 1
        )
        self.pending = False
        self.done = False
        self.get_logger().info(
            "Map loaded; waiting for an explicit map-frame RViz 2D Pose Estimate. "
            "This helper never arms or commands motion."
        )

    def on_pose(self, message):
        if self.pending or self.done:
            return
        if message.header.frame_id != "map":
            self.get_logger().error("Initial pose must be in frame 'map'; pose rejected")
            return
        pose = message.pose.pose
        q = pose.orientation
        values = (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not all(math.isfinite(value) for value in values) or norm < 1e-6:
            self.get_logger().error("Invalid initial pose; pose rejected")
            return
        if not self.client.service_is_ready():
            self.get_logger().warning("Cartographer not ready; set 2D Pose Estimate again")
            return
        qx, qy, qz, qw = (value / norm for value in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        c, s = math.cos(self.ryaw), math.sin(self.ryaw)
        dx, dy = pose.position.x - self.rx, pose.position.y - self.ry
        request = StartTrajectory.Request()
        request.configuration_directory = self.directory
        request.configuration_basename = "rplidar_a1_2d_localization.lua"
        request.use_initial_pose = True
        request.relative_to_trajectory_id = 0
        request.initial_pose.position.x = c * dx + s * dy
        request.initial_pose.position.y = -s * dx + c * dy
        request.initial_pose.orientation.z = math.sin((yaw - self.ryaw) / 2)
        request.initial_pose.orientation.w = math.cos((yaw - self.ryaw) / 2)
        self.get_logger().info(
            f"Explicit initial map pose: x={pose.position.x:.6f}, "
            f"y={pose.position.y:.6f}, yaw={yaw:.6f}"
        )
        self.pending = True
        self.client.call_async(request).add_done_callback(self.on_started)

    def on_started(self, future):
        try:
            response = future.result()
        except Exception as error:
            # Do not retry an ambiguous request: a trajectory may already exist.
            self.get_logger().error(f"StartTrajectory response failed: {error}; restart required")
            self.done = True
            return
        if response.status.code != 0:
            self.get_logger().error(f"StartTrajectory rejected: {response.status.message}")
        else:
            self.get_logger().info(
                f"Localization trajectory {response.trajectory_id} started successfully; "
                "confirm scan/map alignment before explicit navigation START"
            )
        self.done = True


def main():
    rclpy.init()
    node = StartLocalizationFromPose()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
