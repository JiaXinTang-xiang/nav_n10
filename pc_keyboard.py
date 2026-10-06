#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist


LINEAR_SPEED = 0.15
ANGULAR_SPEED = 0.55
PUBLISH_PERIOD = 0.05


def make_twist(linear, angular):
    message = Twist()
    message.linear.x = linear
    message.angular.z = angular
    return message


def main():
    if not sys.stdin.isatty():
        raise RuntimeError('请在交互式终端中运行 pc_keyboard.py')

    rclpy.init()
    node = rclpy.create_node('pc_keyboard_continuous')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    terminal_settings = termios.tcgetattr(sys.stdin)
    command = make_twist(0.0, 0.0)

    print('连续键盘控制（20Hz）')
    print('i 前进  , 后退  j 左转  l 右转  k/空格 停止  q 退出')
    print(f'直线 {LINEAR_SPEED:.2f} m/s，转弯 {ANGULAR_SPEED:.2f} rad/s')

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], PUBLISH_PERIOD)
            if readable:
                key = sys.stdin.read(1)
                if key == 'i':
                    command = make_twist(LINEAR_SPEED, 0.0)
                elif key == ',':
                    command = make_twist(-LINEAR_SPEED, 0.0)
                elif key == 'j':
                    command = make_twist(0.0, ANGULAR_SPEED)
                elif key == 'l':
                    command = make_twist(0.0, -ANGULAR_SPEED)
                elif key in ('k', ' '):
                    command = make_twist(0.0, 0.0)
                elif key in ('q', '\x03'):
                    break

            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        stop = make_twist(0.0, 0.0)
        for _ in range(3):
            publisher.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(PUBLISH_PERIOD)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
