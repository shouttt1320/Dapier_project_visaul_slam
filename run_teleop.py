#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RedBox Responsive Teleop Keyboard Controller
- Supports: Arrow keys (↑, ↓, ←, →) AND (w, a, s, d, x, space)
- Native ROS 2 Jazzy TwistStamped (/cmd_vel)
- Smooth 10 Hz continuous command loop so robot never stalls due to deadband/timeout
- Real-time Battery & Speed dashboard
"""

import sys
import os
import tty
import termios
import select
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState

HELP_MSG = """
======================================================================
  TurtleBot3 High-Responsiveness Teleop Controller
======================================================================
  [Controls]
      ↑ / w  : 전진 (Forward, 기본 0.15 m/s)
      ↓ / x  : 후진 (Reverse, 기본 -0.15 m/s)
      ← / a  : 좌회전 (Turn Left, 기본 0.4 rad/s)
      → / d  : 우회전 (Turn Right, 기본 -0.4 rad/s)
   스페이스 / s: 즉시 정지 (Full Stop)

      + / -  : 속도 증가 / 감소
      q / Ctrl+C : 종료
======================================================================
"""

class ResponsiveTeleop(Node):
    def __init__(self):
        super().__init__('responsive_teleop')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.battery_sub = self.create_subscription(BatteryState, '/battery_state', self.battery_cb, 10)
        
        self.battery_volt = 0.0
        self.target_vx = 0.0
        self.target_wz = 0.0
        
        self.speed_step_vx = 0.15
        self.speed_step_wz = 0.40
        self.max_vx = 0.26
        self.max_wz = 1.80

        # Continuous 10 Hz publisher loop
        self.timer = self.create_timer(0.1, self.timer_cb)

    def battery_cb(self, msg):
        self.battery_volt = msg.voltage

    def timer_cb(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(self.target_vx)
        msg.twist.angular.z = float(self.target_wz)
        self.pub.publish(msg)

    def print_status(self):
        sys.stdout.write(
            f"\r[조종 상태] 선속도: {self.target_vx:+.2f} m/s | 각속도: {self.target_wz:+.2f} rad/s | 배터리: {self.battery_volt:.2f}V    "
        )
        sys.stdout.flush()

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            # Handle multi-byte arrow key sequences
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = ResponsiveTeleop()

    # Spin ROS 2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP_MSG)
    node.print_status()

    try:
        while rclpy.ok():
            key = get_key(settings)
            if not key:
                continue

            # Forward
            if key in ('w', 'W', '\x1b[A'):
                node.target_vx = node.speed_step_vx
                node.target_wz = 0.0
            # Backward
            elif key in ('x', 'X', '\x1b[B'):
                node.target_vx = -node.speed_step_vx
                node.target_wz = 0.0
            # Turn Left
            elif key in ('a', 'A', '\x1b[D'):
                node.target_vx = 0.0
                node.target_wz = node.speed_step_wz
            # Turn Right
            elif key in ('d', 'D', '\x1b[C'):
                node.target_vx = 0.0
                node.target_wz = -node.speed_step_wz
            # Stop
            elif key in (' ', 's', 'S'):
                node.target_vx = 0.0
                node.target_wz = 0.0
            # Increase Speed
            elif key in ('+', '='):
                node.speed_step_vx = min(node.max_vx, node.speed_step_vx + 0.03)
                node.speed_step_wz = min(node.max_wz, node.speed_step_wz + 0.10)
                print(f"\n>> 속도 증가: 기준 속도 {node.speed_step_vx:.2f} m/s, 회전 {node.speed_step_wz:.2f} rad/s")
            # Decrease Speed
            elif key in ('-', '_'):
                node.speed_step_vx = max(0.06, node.speed_step_vx - 0.03)
                node.speed_step_wz = max(0.15, node.speed_step_wz - 0.10)
                print(f"\n>> 속도 감소: 기준 속도 {node.speed_step_vx:.2f} m/s, 회전 {node.speed_step_wz:.2f} rad/s")
            # Quit
            elif key in ('q', 'Q', '\x03'):
                break

            node.print_status()

    except Exception as ex:
        print(f"\nError: {ex}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        # Send zero velocity on quit
        node.target_vx = 0.0
        node.target_wz = 0.0
        node.timer_cb()
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop controller stopped cleanly.")

if __name__ == '__main__':
    main()
