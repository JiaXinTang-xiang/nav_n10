#!/usr/bin/env python3
"""
anoros_dt_node.py — ROS2 桥接节点
读取匿名飞控 (ANO Tech) 串口数据，解析 ANO PT v7 协议，
发布 sensor_msgs/Imu 到 /imu/data 话题。

协议参考：ROS1 anorosdt 包 (anoSerial.cpp, AnoPTv7.h)
飞控固件：STM32F407, UART3 921600bps, ANO PT v7
"""

import struct
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


# ─── ANO PT v7 协议常量 ───────────────────────────────────────────────
ANO_HEADER = 0xAA
ANO_BROADCAST = 0xFF

# 帧 ID
FRAME_ID_IMU_RAW   = 0x01   # 加速度 + 陀螺仪 (1ms 周期)
FRAME_ID_MAG        = 0x02   # 磁力计 (5ms 周期)
FRAME_ID_QUAT       = 0x04   # 四元数姿态 (1ms 周期)

# 帧解析状态机
STATE_HEADER = 0
STATE_ADDR   = 1
STATE_ID     = 2
STATE_LEN    = 3
STATE_DATA   = 4
STATE_SC1    = 5
STATE_SC2    = 6


class AnoParser:
    """ANO PT v7 协议解析器（纯协议层，不依赖 ROS）"""

    def __init__(self):
        self._state = STATE_HEADER
        self._raw = bytearray(300)
        self._cnt = 0
        self._data_cnt = 0
        self._data_len = 0

        # 解析出的 IMU 数据
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.gyr_x = 0.0
        self.gyr_y = 0.0
        self.gyr_z = 0.0
        self.q_w = 1.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0

        # 回调
        self.on_imu_update = None

    def feed(self, data: int) -> bool:
        """喂入一个字节，返回 True 表示成功解析完一帧"""
        if self._state == STATE_HEADER:
            if data == ANO_HEADER:
                self._cnt = 0
                self._raw[self._cnt] = data
                self._cnt += 1
                self._state = STATE_ADDR

        elif self._state == STATE_ADDR:
            self._raw[self._cnt] = data
            self._cnt += 1
            self._state = STATE_ID

        elif self._state == STATE_ID:
            self._raw[self._cnt] = data
            self._cnt += 1
            self._state = STATE_LEN

        elif self._state == STATE_LEN:
            self._raw[self._cnt] = data
            self._cnt += 1
            self._data_len = data
            self._data_cnt = 0
            self._state = STATE_DATA if self._data_len > 0 else STATE_SC1

        elif self._state == STATE_DATA:
            self._raw[self._cnt] = data
            self._cnt += 1
            self._data_cnt += 1
            if self._data_cnt >= self._data_len:
                self._state = STATE_SC1

        elif self._state == STATE_SC1:
            self._sc1 = data
            self._state = STATE_SC2

        elif self._state == STATE_SC2:
            self._sc2 = data
            self._state = STATE_HEADER

            # 校验
            if self._checksum(self._raw, self._data_len + 4, self._sc1, self._sc2):
                return self._dispatch(self._raw[2], self._raw[4:4 + self._data_len])

        return False

    # ─── 内部 ──────────────────────────────────────────────────────

    def _checksum(self, buf, length, sc1_recv, sc2_recv) -> bool:
        s1 = 0
        s2 = 0
        for i in range(length):
            s1 = (s1 + buf[i]) & 0xFF
            s2 = (s2 + s1) & 0xFF
        return s1 == sc1_recv and s2 == sc2_recv

    def _dispatch(self, frame_id: int, payload: bytes) -> bool:
        """分发帧数据"""
        if frame_id == FRAME_ID_IMU_RAW and len(payload) >= 12:
            # 加速度：int16 / 100 = m/s²
            self.acc_x, self.acc_y, self.acc_z = struct.unpack_from('<hhh', payload, 0)
            self.acc_x /= 100.0
            self.acc_y /= 100.0
            self.acc_z /= 100.0

            # 角速度：int16 / 16.384 * π / 180 = rad/s
            self.gyr_x, self.gyr_y, self.gyr_z = struct.unpack_from('<hhh', payload, 6)
            self.gyr_x = self.gyr_x / 16.384 * math.pi / 180.0
            self.gyr_y = self.gyr_y / 16.384 * math.pi / 180.0
            self.gyr_z = self.gyr_z / 16.384 * math.pi / 180.0

            return True

        if frame_id == FRAME_ID_QUAT and len(payload) >= 8:
            # 四元数：int16 / 10000
            self.q_w, self.q_x, self.q_y, self.q_z = struct.unpack_from('<hhhh', payload, 0)
            self.q_w /= 10000.0
            self.q_x /= 10000.0
            self.q_y /= 10000.0
            self.q_z /= 10000.0

            return True

        return False


class AnorosDTNode(Node):
    """ROS2 飞控 IMU 桥接节点"""

    def __init__(self):
        super().__init__('anoros_dt')

        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 921600)
        self.declare_parameter('pub_topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')

        # 获取参数
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baud').value
        topic = self.get_parameter('pub_topic').value
        self._frame_id = self.get_parameter('frame_id').value

        # 创建发布者
        self._pub = self.create_publisher(Imu, topic, 200)

        # 打开串口
        self._open_serial(port, baud)

        # 协议解析器
        self._parser = AnoParser()

        # 定时器轮询串口 (1ms 周期)
        self._timer = self.create_timer(0.001, self._poll)

        # 已知的串口读取方法
        self._reader = None
        # 尝试不同的 pyserial API
        if hasattr(self._serial, 'in_waiting'):
            self._reader = self._read_in_waiting
        elif hasattr(self._serial, 'read'):
            self._reader = self._read_single

        self.get_logger().info(f'串口已打开: {port} @ {baud} bps')
        self.get_logger().info(f'发布 IMU 话题: {topic} (frame_id: {self._frame_id})')

    def _open_serial(self, port: str, baud: int):
        """打开串口，兼容不同 pyserial 版本"""
        try:
            import serial
        except ImportError:
            self.get_logger().fatal('未安装 pyserial，请执行: pip install pyserial')
            sys.exit(1)

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001,
            )
        except serial.SerialException as e:
            self.get_logger().fatal(f'无法打开串口 {port}: {e}')
            sys.exit(1)

    def _poll(self):
        """定时轮询串口，读取并解析数据"""
        try:
            if self._reader:
                self._reader()
        except Exception:
            pass  # 单帧解析失败不影响后续

    def _read_in_waiting(self):
        """批量读取方式（pyserial >= 3.0）"""
        n = self._serial.in_waiting
        if n > 0:
            data = self._serial.read(n)
            self._feed_bytes(data)

    def _read_single(self):
        """逐字节读取方式（兼容旧版 pyserial）"""
        b = self._serial.read(1)
        if b:
            self._feed_bytes(b)

    def _feed_bytes(self, data: bytes):
        now = self.get_clock().now()
        for byte in data:
            if self._parser.feed(byte):
                self._publish_imu(now)

    def _publish_imu(self, stamp):
        """发布 sensor_msgs/Imu"""
        msg = Imu()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._frame_id

        # 方向四元数（来自飞控姿态解算）
        msg.orientation.w = self._parser.q_w
        msg.orientation.x = self._parser.q_x
        msg.orientation.y = self._parser.q_y
        msg.orientation.z = self._parser.q_z
        msg.orientation_covariance[0] = 0.01   # w
        msg.orientation_covariance[4] = 0.01   # x
        msg.orientation_covariance[8] = 0.01   # y

        # 角速度 rad/s
        msg.angular_velocity.x = self._parser.gyr_x
        msg.angular_velocity.y = self._parser.gyr_y
        msg.angular_velocity.z = self._parser.gyr_z
        msg.angular_velocity_covariance[0] = 0.01  # x
        msg.angular_velocity_covariance[4] = 0.01  # y
        msg.angular_velocity_covariance[8] = 0.01  # z

        # 线加速度 m/s²
        msg.linear_acceleration.x = self._parser.acc_x
        msg.linear_acceleration.y = self._parser.acc_y
        msg.linear_acceleration.z = self._parser.acc_z
        msg.linear_acceleration_covariance[0] = 0.01  # x
        msg.linear_acceleration_covariance[4] = 0.01  # y
        msg.linear_acceleration_covariance[8] = 0.01  # z

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AnorosDTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
