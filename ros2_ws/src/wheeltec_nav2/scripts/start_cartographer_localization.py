#!/usr/bin/env python3

import math

import rclpy
from cartographer_ros_msgs.srv import StartTrajectory, TrajectoryQuery
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class StartCartographerLocalization(Node):
    def __init__(self):
        super().__init__('start_cartographer_localization')
        self.configuration_directory = self.declare_parameter(
            'configuration_directory', '').value
        self.configuration_basename = self.declare_parameter(
            'configuration_basename', 'lsn10_localization.lua').value
        self.client = self.create_client(StartTrajectory, '/start_trajectory')
        self.trajectory_client = self.create_client(
            TrajectoryQuery, '/trajectory_query')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.pose_callback, 1)
        self.request_pending = False
        self.trajectory_started = False
        self.reference_pose = None
        self.reference_pending = False
        self.reference_timer = self.create_timer(0.5, self.query_reference_pose)
        self.get_logger().info(
            '地图已加载，等待 RViz 2D Pose Estimate；不会发送运动指令。')

    def query_reference_pose(self):
        if self.reference_pose is not None or self.reference_pending:
            return
        if not self.trajectory_client.service_is_ready():
            return
        request = TrajectoryQuery.Request()
        request.trajectory_id = 0
        self.reference_pending = True
        self.trajectory_client.call_async(request).add_done_callback(
            self.reference_done)

    def reference_done(self, future):
        self.reference_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f'读取冻结轨迹基准失败: {error}')
            return
        if response.status.code != 0 or not response.trajectory:
            self.get_logger().warning('冻结轨迹尚未可以查询')
            return
        self.reference_pose = response.trajectory[0].pose
        quaternion = self.reference_pose.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z
                   + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y
                         + quaternion.z * quaternion.z))
        self.get_logger().info(
            '已读取冻结轨迹基准: '
            f'x={self.reference_pose.position.x:.6f}, '
            f'y={self.reference_pose.position.y:.6f}, yaw={yaw:.6f}')

    def pose_callback(self, message):
        if self.request_pending or self.trajectory_started:
            return
        if self.reference_pose is None:
            self.get_logger().warning('冻结轨迹基准尚未读取，请稍后再次设置初始位姿')
            return
        if message.header.frame_id != 'map':
            self.get_logger().error("初始位姿必须使用 'map' 坐标系")
            return
        if not self.client.service_is_ready():
            self.get_logger().warning('Cartographer 尚未就绪，请再次设置初始位姿')
            return

        pose = message.pose.pose
        quaternion = pose.orientation
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w)
        if not all(math.isfinite(value) for value in values) or norm < 1e-6:
            self.get_logger().error('初始位姿无效')
            return

        request = StartTrajectory.Request()
        request.configuration_directory = self.configuration_directory
        request.configuration_basename = self.configuration_basename
        request.use_initial_pose = True
        reference = self.reference_pose
        reference_quaternion = reference.orientation
        reference_yaw = math.atan2(
            2.0 * (reference_quaternion.w * reference_quaternion.z
                   + reference_quaternion.x * reference_quaternion.y),
            1.0 - 2.0 * (reference_quaternion.y * reference_quaternion.y
                         + reference_quaternion.z * reference_quaternion.z))
        pose_yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z
                   + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y
                         + quaternion.z * quaternion.z))
        delta_x = pose.position.x - reference.position.x
        delta_y = pose.position.y - reference.position.y
        cosine = math.cos(reference_yaw)
        sine = math.sin(reference_yaw)

        request.initial_pose.position.x = cosine * delta_x + sine * delta_y
        request.initial_pose.position.y = -sine * delta_x + cosine * delta_y
        request.initial_pose.position.z = pose.position.z
        relative_yaw = pose_yaw - reference_yaw
        request.initial_pose.orientation.z = math.sin(relative_yaw / 2.0)
        request.initial_pose.orientation.w = math.cos(relative_yaw / 2.0)
        request.relative_to_trajectory_id = 0

        self.request_pending = True
        self.client.call_async(request).add_done_callback(self.start_done)

    def start_done(self, future):
        self.request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'启动定位轨迹失败: {error}')
            return

        if response.status.code != 0:
            self.get_logger().error(
                f'定位轨迹被拒绝: {response.status.message}')
            return

        self.trajectory_started = True
        self.get_logger().info(
            f'定位轨迹 {response.trajectory_id} 已启动；先检查激光与地图重合，再启动 Nav2。')


def main(args=None):
    rclpy.init(args=args)
    node = StartCartographerLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
