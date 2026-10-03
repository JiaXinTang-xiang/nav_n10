#!/usr/bin/env python3
"""
chassis_bridge.py — 底盘串口桥接节点

STM32F103 (USART1, 115200bps, 8N1):
  PC → MCU: 0xBB + v_linear(int16 BE, mm/s) + v_angular(int16 BE, mrad/s) + cksum + 0x55
  MCU → PC: 0xCC + x(float LE, mm) + y(float LE, mm) + theta(float LE, rad) + cksum + 0x55

功能:
  1. 收 MCU 里程计 → 发布 /odom + 广播 odom→base_link TF
  2. 订阅 /cmd_vel → 转 0xBB 帧发给 MCU
"""

import struct
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
from tf2_ros import TransformBroadcaster


# ─── 协议常亮 ─────────────────────────────────────────────────────────
HEADER_CMD  = 0xBB   # PC → MCU 速度指令
HEADER_ODOM = 0xCC   # MCU → PC 里程计回传
FOOTER      = 0x55


class ChassisBridge(Node):
    """底盘串口桥接节点"""

    def __init__(self):
        super().__init__('chassis_bridge')

        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baud').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value

        # 串口
        self._open_serial(port, baud)

        # 发布者
        self._odom_pub = self.create_publisher(Odometry, '/odom', 50)

        # TF 广播
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self._publish_tf else None)

        # 订阅 /cmd_vel
        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # 定时器轮询串口
        self._timer = self.create_timer(0.005, self._poll)  # 200Hz

        # 接收状态机
        self._rx_buf = bytearray(32)
        self._rx_idx = 0
        self._rx_state = 0       # 0=wait header, 1=receiving
        self._rx_expect_len = 0

        # 里程计累计（从 MCU 收到的绝对位姿）
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0

        self.get_logger().info(f'底盘串口已打开: {port} @ {baud} bps')

    # ─── 串口 ──────────────────────────────────────────────────────

    def _open_serial(self, port: str, baud: int):
        try:
            import serial
        except ImportError:
            self.get_logger().fatal('pyserial 未安装: pip install pyserial')
            sys.exit(1)

        try:
            self._serial = serial.Serial(
                port=port, baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001,
            )
        except serial.SerialException as e:
            self.get_logger().fatal(f'无法打开串口 {port}: {e}')
            sys.exit(1)

    # ─── 接收：MCU → PC (0xCC 里程计) ──────────────────────────────

    def _poll(self):
        """定时轮询串口"""
        try:
            n = self._serial.in_waiting
            if n > 0:
                self._feed(self._serial.read(n))
        except Exception:
            pass

    def _feed(self, data: bytes):
        """喂入字节流，状态机解析"""
        for byte in data:
            if self._rx_state == 0:           # 等待帧头
                if byte == HEADER_ODOM:
                    self._rx_buf[0] = byte
                    self._rx_idx = 1
                    self._rx_expect_len = 15   # CC + x(4) + y(4) + θ(4) + ck(1) + 55(1)
                    self._rx_state = 1
                # 非 CC 字节忽略
            elif self._rx_state == 1:          # 收数据
                if self._rx_idx < self._rx_expect_len:
                    self._rx_buf[self._rx_idx] = byte
                    self._rx_idx += 1

                if self._rx_idx >= self._rx_expect_len:
                    self._rx_state = 0
                    if self._rx_buf[self._rx_expect_len - 1] == FOOTER:
                        self._decode_odom()

    def _decode_odom(self):
        """解码 0xCC 包 → 发布 /odom + TF"""
        # 校验
        cksum = 0
        for i in range(1, 13):
            cksum = (cksum + self._rx_buf[i]) & 0xFF
        if cksum != self._rx_buf[13]:
            return  # 校验失败，丢弃

        # 解析（小端 float32）
        x, y, theta = struct.unpack_from('<fff', self._rx_buf, 1)

        # MCU 里程计是累加值，直接用绝对位姿
        self._odom_x = x / 1000.0      # mm → m
        self._odom_y = y / 1000.0      # mm → m
        self._odom_theta = theta       # rad

        now = self.get_clock().now()

        # 发布 /odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.position.z = 0.0

        # 欧拉角 → 四元数
        half = self._odom_theta / 2.0
        odom.pose.pose.orientation.w = math.cos(half)
        odom.pose.pose.orientation.z = math.sin(half)

        # 协方差（编码器里程计，平移约 ±5cm/m，旋转约 ±2°/90°）
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # yaw
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[7] = 0.1
        odom.twist.covariance[35] = 0.1

        self._odom_pub.publish(odom)

        # 广播 TF
        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._odom_x
            tf.transform.translation.y = self._odom_y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    # ─── 发送：PC → MCU (0xBB 速度指令) ────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        """订阅 /cmd_vel → 0xBB 帧发给 MCU"""
        # geometry_msgs/Twist: m/s, rad/s
        # MCU 期望: mm/s, mrad/s
        v_linear = int(msg.linear.x * 1000.0)    # m/s → mm/s
        v_angular = int(msg.angular.z * 1000.0)  # rad/s → mrad/s

        # 限幅（防止超出 MCU 预期范围）
        v_linear = max(-32767, min(32767, v_linear))
        v_angular = max(-32767, min(32767, v_angular))

        # 组帧: BB + v_linear(int16 BE) + v_angular(int16 BE) + cksum + 55
        buf = bytearray(7)
        buf[0] = HEADER_CMD

        # 大端 int16
        buf[1] = (v_linear >> 8) & 0xFF
        buf[2] = v_linear & 0xFF
        buf[3] = (v_angular >> 8) & 0xFF
        buf[4] = v_angular & 0xFF

        # 校验: bytes 1-4 累加
        cksum = (buf[1] + buf[2] + buf[3] + buf[4]) & 0xFF
        buf[5] = cksum
        buf[6] = FOOTER

        try:
            self._serial.write(buf)
        except Exception as e:
            self.get_logger().error(f'串口发送失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ChassisBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
