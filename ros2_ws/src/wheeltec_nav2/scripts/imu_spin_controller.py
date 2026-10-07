#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import Spin
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class ImuSpinController(Node):
    """Execute a relative in-place rotation using IMU yaw feedback."""

    def __init__(self):
        super().__init__('imu_spin_controller')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('min_angular_speed', 0.60)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('kp', 1.8)
        self.declare_parameter('slow_down_angle', 0.45)
        self.declare_parameter('angle_tolerance', 0.045)
        self.declare_parameter('imu_timeout_sec', 0.25)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pulse_period_sec', 0.24)
        self.declare_parameter('pulse_on_sec', 0.08)

        self.min_speed = float(self.get_parameter('min_angular_speed').value)
        self.max_speed = float(self.get_parameter('max_angular_speed').value)
        self.kp = float(self.get_parameter('kp').value)
        self.slow_down_angle = float(self.get_parameter('slow_down_angle').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)
        control_rate = float(self.get_parameter('control_rate_hz').value)
        self.pulse_period = float(self.get_parameter('pulse_period_sec').value)
        self.pulse_on = float(self.get_parameter('pulse_on_sec').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        if not 0.0 < self.min_speed <= self.max_speed:
            raise ValueError('min_angular_speed must be positive and <= max_angular_speed')
        if self.kp <= 0.0 or self.angle_tolerance <= 0.0:
            raise ValueError('kp and angle_tolerance must be positive')
        if self.pulse_period <= 0.0 or not 0.0 < self.pulse_on <= self.pulse_period:
            raise ValueError('pulse_on_sec must be within pulse_period_sec')

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        imu_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, imu_topic, self.imu_callback, imu_qos)

        self.callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            Spin,
            '/imu_spin',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.imu_lock = threading.Lock()
        self.current_yaw = None
        self.unwrapped_yaw = None
        self.last_imu_yaw = None
        self.last_imu_time = None
        self.get_logger().info(
            f'IMU闭环转弯已启动: min={self.min_speed:.2f} rad/s, '
            f'max={self.max_speed:.2f} rad/s, tolerance={self.angle_tolerance:.3f} rad')

    def imu_callback(self, message):
        yaw = yaw_from_quaternion(message.orientation)
        now = time.monotonic()
        with self.imu_lock:
            if self.last_imu_yaw is None:
                self.unwrapped_yaw = yaw
            else:
                self.unwrapped_yaw += wrap_angle(yaw - self.last_imu_yaw)
            self.current_yaw = self.unwrapped_yaw
            self.last_imu_yaw = yaw
            self.last_imu_time = now

    def goal_callback(self, _goal_request):
        with self.imu_lock:
            ready = self.unwrapped_yaw is not None
        if not ready:
            self.get_logger().warning('IMU尚未收到有效航向，拒绝旋转目标')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def get_imu_state(self):
        with self.imu_lock:
            return self.unwrapped_yaw, self.last_imu_time

    def publish_speed(self, angular_speed):
        if not rclpy.ok():
            return
        command = Twist()
        command.angular.z = float(angular_speed)
        try:
            self.cmd_pub.publish(command)
        except rclpy.exceptions.RCLError:
            pass

    def stop(self):
        if not rclpy.ok():
            return
        for _ in range(3):
            self.publish_speed(0.0)
            time.sleep(0.02)

    def execute_callback(self, goal_handle):
        requested_angle = float(goal_handle.request.target_yaw)
        time_allowance = float(goal_handle.request.time_allowance.sec)
        time_allowance += float(goal_handle.request.time_allowance.nanosec) * 1e-9
        if time_allowance <= 0.0:
            time_allowance = max(10.0, abs(requested_angle) / self.min_speed * 4.0)

        start_yaw, imu_time = self.get_imu_state()
        target_yaw = start_yaw + requested_angle
        started = time.monotonic()
        feedback = Spin.Feedback()
        result = Spin.Result()

        self.get_logger().info(
            f'开始IMU闭环转弯: target={math.degrees(requested_angle):.1f} deg, '
            f'timeout={time_allowance:.1f}s')

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.stop()
                    goal_handle.canceled()
                    result.total_elapsed_time = self.duration_message(
                        time.monotonic() - started)
                    return result

                current_yaw, last_imu_time = self.get_imu_state()
                now = time.monotonic()
                if current_yaw is None or last_imu_time is None:
                    raise RuntimeError('等待IMU航向')
                if now - last_imu_time > self.imu_timeout_sec:
                    raise RuntimeError('IMU数据超时，已停车')

                error = target_yaw - current_yaw
                remaining = abs(error)
                feedback.angular_distance_traveled = float(
                    abs(current_yaw - start_yaw))
                goal_handle.publish_feedback(feedback)

                if remaining <= self.angle_tolerance:
                    self.stop()
                    goal_handle.succeed()
                    result.total_elapsed_time = self.duration_message(
                        time.monotonic() - started)
                    self.get_logger().info(
                        f'旋转完成: error={math.degrees(error):.2f} deg')
                    return result

                if now - started > time_allowance:
                    raise RuntimeError('旋转超时，已停车')

                speed = min(self.max_speed, self.kp * remaining)
                speed = max(self.min_speed, speed)
                if remaining <= self.slow_down_angle:
                    pulse_phase = (now - started) % self.pulse_period
                    speed = speed if pulse_phase < self.pulse_on else 0.0
                self.publish_speed(math.copysign(speed, error) if speed else 0.0)
                time.sleep(1.0 / 30.0)
        except Exception as error:
            self.stop()
            if goal_handle.is_active:
                goal_handle.abort()
            result.total_elapsed_time = self.duration_message(
                time.monotonic() - started)
            self.get_logger().error(f'旋转失败: {error}')
            return result

        self.stop()
        if goal_handle.is_active:
            goal_handle.abort()
        result.total_elapsed_time = self.duration_message(
            time.monotonic() - started)
        return result

    @staticmethod
    def duration_message(seconds):
        from builtin_interfaces.msg import Duration

        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - duration.sec) * 1e9)
        return duration


def main(args=None):
    rclpy.init(args=args)
    node = ImuSpinController()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
