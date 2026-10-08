
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


class ImuSpinController(Node):
    """Execute a relative in-place rotation using IMU yaw feedback."""

    def __init__(self):
        super().__init__('imu_spin_controller')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('min_angular_speed', 0.55)
        self.declare_parameter('max_angular_speed', 0.60)
        self.declare_parameter('kp', 1.8)
        self.declare_parameter('slow_down_angle', 0.45)
        self.declare_parameter('angle_tolerance', 0.045)
        self.declare_parameter('imu_timeout_sec', 0.2)
        self.declare_parameter('settle_time_sec', 0.3)
        self.declare_parameter('imu_yaw_sign', 1.0)
        self.declare_parameter('imu_gyro_deadband_rad_s', 0.03)
        self.declare_parameter('imu_max_delta_rad', 0.25)
        self.declare_parameter('stopped_rate_rad_s', 0.05)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pulse_period_sec', 0.24)
        self.declare_parameter('pulse_on_sec', 0.08)

        self.min_speed = float(self.get_parameter('min_angular_speed').value)
        self.max_speed = float(self.get_parameter('max_angular_speed').value)
        self.kp = float(self.get_parameter('kp').value)
        self.slow_down_angle = float(self.get_parameter('slow_down_angle').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)
        self.yaw_sign = float(self.get_parameter('imu_yaw_sign').value)
        self.gyro_deadband = float(self.get_parameter('imu_gyro_deadband_rad_s').value)
        self.max_delta = float(self.get_parameter('imu_max_delta_rad').value)
        self.stopped_rate = float(self.get_parameter('stopped_rate_rad_s').value)
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.settle_time = float(self.get_parameter('settle_time_sec').value)
        self.pulse_period = float(self.get_parameter('pulse_period_sec').value)
        self.pulse_on = float(self.get_parameter('pulse_on_sec').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        values = (self.min_speed, self.max_speed, self.kp, self.angle_tolerance,
                  self.imu_timeout_sec, self.control_rate, self.settle_time,
                  self.yaw_sign, self.gyro_deadband, self.slow_down_angle,
                  self.pulse_period, self.pulse_on, self.max_delta, self.stopped_rate)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Parameters must be finite')
        if not 0.55 <= self.min_speed <= self.max_speed <= 0.60:
            raise ValueError('Nonzero spin speed must be between 0.55 and 0.60 rad/s')
        if min(self.imu_timeout_sec, self.control_rate, self.settle_time,
               self.max_delta, self.stopped_rate) <= 0.0:
            raise ValueError('Timeout, control rate and settle time must be positive')
        if self.kp <= 0.0 or self.angle_tolerance <= 0.0:
            raise ValueError('kp and angle_tolerance must be positive')
        if self.slow_down_angle <= self.angle_tolerance:
            raise ValueError('slow_down_angle must exceed angle_tolerance')
        if abs(self.yaw_sign) != 1.0:
            raise ValueError('imu_yaw_sign must be +1 or -1')
        if self.gyro_deadband < 0.0:
            raise ValueError('imu_gyro_deadband_rad_s must not be negative')
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
        self.last_imu_time = None
        self.last_imu_stamp = None
        self.imu_ready = False
        self.imu_generation = 0
        self.gyro_rate = 0.0
        self.busy = False
        self.get_logger().info(
            f'IMU闭环转弯已启动(gyro_z积分): min={self.min_speed:.2f} rad/s, '
            f'max={self.max_speed:.2f} rad/s, tolerance={self.angle_tolerance:.3f} rad, '
            f'yaw_sign={self.yaw_sign}, deadband={self.gyro_deadband} rad/s')

    def imu_callback(self, message):
        """Integrate gyro_z using source message timestamps."""
        wz = message.angular_velocity.z * self.yaw_sign
        now = time.monotonic()
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        with self.imu_lock:
            if stamp <= 0 or not math.isfinite(wz) or not -0.02 <= age <= self.imu_timeout_sec:
                self.imu_ready = False
                self.last_imu_stamp = None
                self.imu_generation += 1
                return
            if self.last_imu_stamp is not None and stamp <= self.last_imu_stamp:
                return
            if self.unwrapped_yaw is None:
                self.unwrapped_yaw = 0.0
            dt = (stamp - self.last_imu_stamp) * 1e-9 if self.last_imu_stamp is not None else 0.0
            delta = wz * dt if abs(wz) >= self.gyro_deadband else 0.0
            continuous = (
                self.last_imu_stamp is not None and 0.0 < dt <= self.imu_timeout_sec
                and now - self.last_imu_time <= self.imu_timeout_sec
                and abs(delta) <= self.max_delta)
            if continuous:
                self.unwrapped_yaw += delta
            else:
                self.imu_generation += 1
            self.imu_ready = continuous
            self.gyro_rate = wz
            self.current_yaw = self.unwrapped_yaw
            self.last_imu_stamp = stamp
            self.last_imu_time = now

    def imu_fresh(self):
        if not self.imu_ready or self.last_imu_stamp is None:
            return False
        age = (self.get_clock().now().nanoseconds - self.last_imu_stamp) * 1e-9
        return (-0.02 <= age <= self.imu_timeout_sec
                and time.monotonic() - self.last_imu_time <= self.imu_timeout_sec)

    def goal_callback(self, goal_request):
        with self.imu_lock:
            if (self.busy or not self.imu_fresh() or not math.isfinite(goal_request.target_yaw)
                    or goal_request.time_allowance.sec < 0
                    or goal_request.time_allowance.nanosec >= 1000000000):
                self.get_logger().warning('正在转弯、IMU未就绪或目标无效，拒绝旋转目标')
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

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
        try:
            return self.execute_goal(goal_handle)
        finally:
            self.stop()
            with self.imu_lock:
                self.busy = False

    def execute_goal(self, goal_handle):
        requested_angle = float(goal_handle.request.target_yaw)
        time_allowance = float(goal_handle.request.time_allowance.sec)
        time_allowance += float(goal_handle.request.time_allowance.nanosec) * 1e-9
        if time_allowance <= 0.0:
            time_allowance = max(10.0, abs(requested_angle) / self.min_speed * 4.0)

        with self.imu_lock:
            start_yaw = self.unwrapped_yaw
            generation = self.imu_generation
        target_yaw = start_yaw + requested_angle
        started = time.monotonic()
        settled_since = None
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

                now = time.monotonic()
                with self.imu_lock:
                    if not self.imu_fresh() or self.imu_generation != generation:
                        raise RuntimeError('IMU数据过期或积分中断，已停车')
                    current_yaw = self.unwrapped_yaw
                    gyro_rate = self.gyro_rate
                if now - started > time_allowance:
                    raise RuntimeError('旋转超时，已停车')

                error = target_yaw - current_yaw
                remaining = abs(error)
                feedback.angular_distance_traveled = float(
                    current_yaw - start_yaw)
                goal_handle.publish_feedback(feedback)

                if remaining <= self.angle_tolerance:
                    self.publish_speed(0.0)
                    if abs(gyro_rate) > self.stopped_rate:
                        settled_since = None
                    elif settled_since is None:
                        settled_since = now
                    if settled_since is None or now - settled_since < self.settle_time:
                        time.sleep(1.0 / self.control_rate)
                        continue
                    goal_handle.succeed()
                    result.total_elapsed_time = self.duration_message(
                        time.monotonic() - started)
                    self.get_logger().info(
                        f'旋转完成: error={math.degrees(error):.2f} deg')
                    return result

                settled_since = None

                speed = min(self.max_speed, self.kp * remaining)
                speed = max(self.min_speed, speed)
                if remaining <= self.slow_down_angle:
                    pulse_phase = (now - started) % self.pulse_period
                    speed = speed if pulse_phase < self.pulse_on else 0.0
                self.publish_speed(math.copysign(speed, error) if speed else 0.0)
                time.sleep(1.0 / self.control_rate)
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
