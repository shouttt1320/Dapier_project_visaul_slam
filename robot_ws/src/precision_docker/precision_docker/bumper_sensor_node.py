#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
 [TurtleBot3 Bumper Contact Sensor Node]
  - Hardware interface: Micro-switch contact sensor connected to Raspberry Pi GPIO.
  - Default Wiring:
      * Physical Pin 16 = BCM GPIO 23 (Input with Internal Pull-up)
      * Physical Pin 14 = GND (Common ground)
  - Publishes:
      * /robot/bumper/contact (std_msgs/msg/Bool) at 50Hz with 30ms debounce filter
==============================================================================
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from rclpy.executors import ExternalShutdownException

try:
    import RPi.GPIO as GPIO
    RPI_GPIO_AVAILABLE = True
except ImportError:
    RPI_GPIO_AVAILABLE = False


class BumperSensorNode(Node):
    def __init__(self):
        super().__init__('bumper_sensor_node')

        # Parameters
        self.declare_parameter('pin', 23)               # BCM GPIO 23 (Physical Pin 16)
        self.declare_parameter('poll_rate_hz', 50.0)    # 50 Hz polling
        self.declare_parameter('debounce_ms', 15.0)     # 15 ms debounce filter
        self.declare_parameter('active_low', True)      # Pull-up: contact to GND = LOW (0)

        self.pin = int(self.get_parameter('pin').value)
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.debounce_sec = float(self.get_parameter('debounce_ms').value) / 1000.0
        self.active_low = bool(self.get_parameter('active_low').value)

        # State tracking
        self.contact_detected = False
        self.last_raw_state = None
        self.state_change_time = None

        # Publisher
        self.contact_pub = self.create_publisher(Bool, '/robot/bumper/contact', 10)

        # Hardware Initialization
        self.gpio_ready = False
        if RPI_GPIO_AVAILABLE:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                pull_mode = GPIO.PUD_UP if self.active_low else GPIO.PUD_DOWN
                GPIO.setup(self.pin, GPIO.IN, pull_up_down=pull_mode)
                self.gpio_ready = True
                self.get_logger().info(
                    f"[BUMPER] RPi.GPIO initialized on BCM Pin {self.pin} "
                    f"(Active {'LOW (Pull-Up)' if self.active_low else 'HIGH (Pull-Down)'}, "
                    f"Poll: {self.poll_rate_hz:.0f}Hz, Debounce: {self.debounce_sec*1000:.0f}ms)"
                )
            except Exception as e:
                self.get_logger().error(f"[BUMPER] Failed to initialize GPIO {self.pin}: {e}")
        else:
            self.get_logger().warn(
                "[BUMPER] RPi.GPIO not available on this platform. "
                "Running in MOCK mode (publishes /robot/bumper/contact=False)."
            )

        # Polling Timer
        timer_period = 1.0 / max(1.0, self.poll_rate_hz)
        self.timer = self.create_timer(timer_period, self.poll_sensor)

    def poll_sensor(self):
        now = time.time()

        if self.gpio_ready:
            raw_val = GPIO.input(self.pin)
            is_contact_raw = (raw_val == GPIO.LOW) if self.active_low else (raw_val == GPIO.HIGH)
        else:
            is_contact_raw = False

        # Debounce logic
        if self.last_raw_state is None or is_contact_raw != self.last_raw_state:
            self.last_raw_state = is_contact_raw
            self.state_change_time = now

        if self.state_change_time is not None:
            if (now - self.state_change_time) >= self.debounce_sec:
                if is_contact_raw != self.contact_detected:
                    self.contact_detected = is_contact_raw
                    if self.contact_detected:
                        self.get_logger().info(">>> [BUMPER] CONTACT DETECTED! (Switch Pressed -> GND)")
                    else:
                        self.get_logger().info("<<< [BUMPER] CONTACT RELEASED (Switch Open)")

        # Publish status
        msg = Bool()
        msg.data = bool(self.contact_detected)
        self.contact_pub.publish(msg)

    def destroy_node(self):
        if self.gpio_ready:
            try:
                GPIO.cleanup(self.pin)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BumperSensorNode()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
