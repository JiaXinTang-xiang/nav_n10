#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node


class NavigationAutostarter(Node):
    def __init__(self):
        super().__init__('navigation_autostarter')
        self.localized = False
        self.request_pending = False
        self.started = False
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10,
        )
        self.client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.timer = self.create_timer(0.5, self.try_start_navigation)
        self.get_logger().info(
            '等待 RViz 2D Pose Estimate；AMCL 定位成功后自动启动导航。'
        )

    def pose_callback(self, _message):
        if not self.localized:
            self.localized = True
            self.get_logger().info('已收到 AMCL 位姿，正在启动 Nav2 导航节点。')

    def try_start_navigation(self):
        if not self.localized or self.request_pending or self.started:
            return
        if not self.client.service_is_ready():
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.request_pending = True
        future = self.client.call_async(request)
        future.add_done_callback(self.startup_done)

    def startup_done(self, future):
        self.request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'启动 Nav2 失败，将重试: {error}')
            return

        if response.success:
            self.started = True
            self.get_logger().info('Nav2 导航节点已全部激活，可以发布导航目标。')
        else:
            self.get_logger().error('生命周期管理器未能启动 Nav2，将自动重试。')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationAutostarter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
