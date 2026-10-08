#!/usr/bin/env python3

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigationGoalGuard(Node):
    def __init__(self):
        super().__init__('navigation_goal_guard')
        self._action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self._stop_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self._subscription = self.create_subscription(
            PoseStamped, '/navigation_goal', self._goal_callback, 1)
        self._busy = False
        self._goal_handle = None
        self.get_logger().info(
            '导航目标保护已启动；执行中再次点击目标将被忽略。')

    def _goal_callback(self, pose):
        if self._busy:
            self.get_logger().warning('当前导航尚未结束，忽略新的导航目标。')
            return
        if not self._action_client.server_is_ready():
            self.get_logger().error('Nav2 action 尚未就绪，忽略导航目标。')
            self._publish_stop()
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._busy = True
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            f'接受导航目标: x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}')

    def _goal_response_callback(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'发送导航目标失败: {error}')
            self._finish()
            return

        if not self._goal_handle.accepted:
            self.get_logger().error('Nav2 拒绝导航目标。')
            self._finish()
            return

        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self.get_logger().error(f'读取导航结果失败: {error}')
            self._finish()
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('导航目标已完成，可以发送下一个目标。')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning('导航目标已取消，可以发送下一个目标。')
        else:
            self.get_logger().error(
                f'导航目标失败，状态码={status}；小车已停车。')
        self._finish()

    def _finish(self):
        self._publish_stop()
        self._goal_handle = None
        self._busy = False

    def _publish_stop(self):
        self._stop_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGoalGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
