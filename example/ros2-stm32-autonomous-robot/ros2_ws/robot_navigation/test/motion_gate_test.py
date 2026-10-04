#!/usr/bin/python3
"""One bounded native-controller boundary check; isolated DDS domain, mock actuators only.

ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 motion_gate_test.py
The gate executable is the first argument. Never run this in the robot's domain.
"""
import math
import os
import subprocess
import sys
import threading
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_stm32_bridge.msg import BridgeStatus
from std_msgs.msg import String
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


class Harness(Node):
    def __init__(self, mock_navigation=True):
        super().__init__('goal_gate_mock')
        group = ReentrantCallbackGroup()
        self.armed = False
        self.arm_count = 0
        self.disarm_count = 0
        self.replan_requested = False
        self.goals = 0
        self.canceled = 0
        self.range = 1.5
        self.scan_angle = math.pi
        self.scan_enabled = True
        self.fault_flags = 0
        self.finish = False
        self.command_enabled = True
        self.control_active = True
        self.control_status = None
        self.control_sequence = 0
        self.status = None
        self.command = Twist()
        self.control_pub = self.create_publisher(GoalStatusArray, '/follow_path/_action/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_pub = self.create_publisher(LaserScan, '/test_gate/scan', qos_profile_sensor_data)
        self.bridge_pub = self.create_publisher(BridgeStatus, '/test_gate/bridge', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.command_pub = self.create_publisher(Twist, '/test_gate/nav_cmd', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/test_gate/goal', 1)
        self.create_subscription(String, '/test_goal_gate/status', self.on_status, 10)
        self.create_subscription(Twist, '/test_gate/cmd', self.on_command, 10)
        self.create_service(Trigger, '/test_gate/arm', self.arm, callback_group=group)
        self.create_service(Trigger, '/test_gate/disarm', self.disarm, callback_group=group)
        self.stop_client = self.create_client(Trigger, '/test_goal_gate/stop', callback_group=group)
        self.reset_client = self.create_client(Trigger, '/test_goal_gate/reset', callback_group=group)
        self.server = ActionServer(
            self, NavigateToPose, '/test_gate/navigate', self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group) if mock_navigation else None
        self.create_timer(0.02, self.telemetry, callback_group=group)

    def on_status(self, message):
        self.status = message

    def on_command(self, message):
        self.command = message

    def arm(self, _, response):
        self.armed = True
        self.arm_count += 1
        response.success = True
        return response

    def disarm(self, _, response):
        self.armed = False
        self.disarm_count += 1
        response.success = True
        return response

    def telemetry(self):
        stamp = self.get_clock().now().to_msg()
        if self.scan_enabled:
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.angle_min = self.scan_angle
            scan.angle_max = self.scan_angle + 0.01
            scan.angle_increment = 0.01
            scan.range_min = 0.05
            scan.range_max = 12.0
            scan.ranges = [self.range]
            self.scan_pub.publish(scan)
        status = BridgeStatus()
        status.header.stamp = stamp
        status.connected = status.protocol_valid = status.can_healthy = status.status_fresh = True
        status.system_state = BridgeStatus.SYSTEM_SAFE
        status.fault_flags = self.fault_flags
        status.authority_armed = self.armed
        status.motion_flags = (1 << 15) | (((1 << 7) | (1 << 8)) if self.armed else 0)
        status.last_disarm_tx_stamp = stamp
        self.bridge_pub.publish(status)
        transforms = []
        for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        self.tf_pub.publish(TFMessage(transforms=transforms))

    def execute(self, handle):
        self.goals += 1
        self.finish = False
        self.control_status = None
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self.canceled += 1
                self.command_pub.publish(Twist())
                handle.canceled()
                return NavigateToPose.Result()
            if self.finish:
                self.command_pub.publish(Twist())
                handle.succeed()
                return NavigateToPose.Result()
            if self.control_active and (self.control_status is None or self.control_status.status != GoalStatus.STATUS_EXECUTING):
                self.control_sequence += 1
                self.control_status = GoalStatus()
                self.control_status.goal_info.stamp = self.get_clock().now().to_msg()
                self.control_status.goal_info.goal_id.uuid = [self.control_sequence] * 16
                self.control_status.status = GoalStatus.STATUS_EXECUTING
            elif not self.control_active and self.control_status is not None:
                self.control_status.status = GoalStatus.STATUS_ABORTED
            if self.replan_requested:
                old = self.control_status
                self.control_sequence += 1
                new = GoalStatus()
                new.goal_info.stamp = self.get_clock().now().to_msg()
                new.goal_info.goal_id.uuid = [self.control_sequence] * 16
                new.status = GoalStatus.STATUS_EXECUTING
                self.control_pub.publish(GoalStatusArray(status_list=[old, new]))
                old.status = GoalStatus.STATUS_ABORTED
                self.control_pub.publish(GoalStatusArray(status_list=[old, new]))
                self.control_status = new
                self.replan_requested = False
            if self.control_status is not None:
                self.control_pub.publish(GoalStatusArray(status_list=[self.control_status]))
            command = Twist()
            command.linear.x = 0.30  # Verify gate clamps mock Nav2 input to .20.
            if self.command_enabled and self.control_active:
                self.command_pub.publish(command)
            feedback = NavigateToPose.Feedback()
            feedback.current_pose.header.frame_id = 'map'
            feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
            feedback.distance_remaining = 0.5
            handle.publish_feedback(feedback)
            time.sleep(0.05)
        handle.abort()
        return NavigateToPose.Result()

    def goal(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.stamp.sec += 3600  # A remote UI clock is not a velocity watchdog.
        pose.pose.position.x = 0.5
        pose.pose.orientation.w = 1.0
        self.goal_pub.publish(pose)


class GoalGateTest(unittest.TestCase):
    def wait_for(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.assertIsNone(self.process.poll(), 'gate process exited')
            time.sleep(0.02)
        self.fail(f'condition timed out; status={self.harness.status}')

    def call(self, client):
        self.assertTrue(client.wait_for_service(timeout_sec=2.0))
        future = client.call_async(Trigger.Request())
        self.wait_for(future.done)
        self.assertTrue(future.result().success, future.result().message)

    def test_goal_pause_resume_success_and_fault_latches(self):
        rclpy.init()
        self.harness = h = Harness()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(h)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        args = [GATE_BINARY, '--ros-args', '-r', '__node:=test_goal_gate']
        latch_directory = tempfile.TemporaryDirectory(prefix='motion-gate-check-')
        latch_file = os.path.join(latch_directory.name, 'stop_latch')
        parameters = {
            'latch_file': latch_file,
            'startup_locked': 'true',
            'max_linear_speed_mps': '0.20',
            'scan_topic': '/test_gate/scan',
            'bridge_status_topic': '/test_gate/bridge', 'cmd_vel_topic': '/test_gate/cmd',
            'input_cmd_vel_topic': '/test_gate/nav_cmd', 'arm_service': '/test_gate/arm',
            'disarm_service': '/test_gate/disarm', 'goal_topic': '/test_gate/goal',
            'navigation_action': '/test_gate/navigate'}
        for key, value in parameters.items():
            args += ['-p', f'{key}:={value}']
        self.process = subprocess.Popen(args)
        try:
            self.wait_for(lambda: h.status is not None)
            time.sleep(0.3)
            h.goal()
            time.sleep(0.2)
            self.assertEqual(h.goals, 0, 'startup lock requires manual reset')
            self.call(h.reset_client)
            h.goal()
            self.wait_for(lambda: h.armed and h.command.linear.x > 0)
            self.assertAlmostEqual(h.command.linear.x, 0.20)
            arms, disarms = h.arm_count, h.disarm_count
            h.replan_requested = True
            self.wait_for(lambda: not h.replan_requested)
            time.sleep(0.4)
            self.assertEqual(h.arm_count, arms, 'native path replacement must not rearm')
            self.assertEqual(h.disarm_count, disarms, 'native path replacement must not withdraw')
            self.assertGreater(h.command.linear.x, 0)
            h.range = 0.19
            time.sleep(0.35)
            self.assertTrue(h.armed, 'range interpretation belongs to Nav2, not the boundary')
            h.control_active = False
            self.wait_for(lambda: not h.armed and h.command.linear.x == 0)
            time.sleep(0.6)
            self.assertEqual(h.canceled, 0, 'explicit controller termination must retain NavigateToPose')
            self.assertEqual(h.goals, 1)
            h.control_active = True
            h.command_enabled = False
            time.sleep(0.10)
            self.assertFalse(h.armed, 'new action alone cannot authorize stale Twist')
            h.command_enabled = True
            self.wait_for(lambda: h.armed and h.command.linear.x > 0)
            self.assertEqual(h.goals, 1)
            h.finish = True
            self.wait_for(lambda: not h.armed and 'GOAL_REACHED' in h.status.data)
            h.goal()
            self.wait_for(lambda: h.goals == 2 and h.armed)
            h.control_active = False
            self.wait_for(lambda: not h.armed)
            h.fault_flags = 2
            self.wait_for(lambda: h.canceled == 1 and 'latched' in h.status.data)
            h.fault_flags = 0
            h.control_active = True
            time.sleep(0.3)
            h.goal()
            time.sleep(0.2)
            self.assertFalse(h.armed)
            self.assertEqual(h.goals, 2, 'restored health cannot auto-clear a real fault')
            self.call(h.reset_client)
            h.goal()
            self.wait_for(lambda: h.goals == 3 and h.armed)
            h.command_enabled = False
            self.wait_for(lambda: h.canceled == 2 and not h.armed and 'Twist timeout' in h.status.data)
            self.assertIn('Twist timeout', h.status.data)
            h.command_enabled = True
            self.call(h.reset_client)
            h.goal()
            self.wait_for(lambda: h.goals == 4 and h.armed)
            h.range = float('nan')
            self.wait_for(lambda: not h.armed and h.canceled == 3 and 'no valid samples' in h.status.data)
            self.assertIn('no valid samples', h.status.data)
            h.range = 1.5
            time.sleep(0.2)
            self.call(h.reset_client)
            h.goal()
            self.wait_for(lambda: h.goals == 5 and h.armed)
            self.call(h.stop_client)
            self.wait_for(lambda: not h.armed and h.canceled == 4)
            h.goal()
            time.sleep(0.3)
            self.assertEqual(h.goals, 5, 'STOP remains latched')
            self.assertTrue(os.path.isfile(latch_file))
            self.process.terminate()
            self.process.wait(timeout=3.0)
            h.status = None
            restarted_args = [a.replace('startup_locked:=true', 'startup_locked:=false') for a in args]
            self.process = subprocess.Popen(restarted_args)
            self.wait_for(lambda: h.status is not None)
            h.goal()
            time.sleep(0.3)
            self.assertFalse(h.armed)
            self.assertEqual(h.goals, 5, 'restart must preserve saved operator STOP')
        finally:
            self.process.terminate()
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            executor.shutdown(timeout_sec=2.0)
            thread.join(timeout=2.0)
            h.destroy_node()
            rclpy.shutdown()
            latch_directory.cleanup()


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '73' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('REFUSED: requires isolated ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1')
    GATE_BINARY = sys.argv.pop(1)
    unittest.main()
