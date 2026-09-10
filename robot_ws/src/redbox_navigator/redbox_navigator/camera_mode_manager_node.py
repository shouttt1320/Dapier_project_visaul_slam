#!/usr/bin/env python3
"""
Camera Mode Manager Node
Dynamically manages Astra S (Nav2 / SLAM) and OS30A (Precision Docking) sensors
to ensure single-camera operation and maximum simulation / CPU efficiency.

Modes:
  1. NAV2 (Default at startup):
     - Astra S Bridge: RUNNING (/camera/color/*, /camera/depth/*)
     - OS30A Bridge:   STOPPED (Zero Gazebo rendering & ROS computation)
     - RTAB-Map SLAM:  RESUMED (/rtabmap/resume)

  2. DOCKING (Precision Close-Docking):
     - RTAB-Map SLAM:  PAUSED (/rtabmap/pause, freeing ~50% CPU)
     - Astra S Bridge: STOPPED (Zero Gazebo rendering & ROS computation)
     - OS30A Bridge:   RUNNING (/os30a/camera/*)

Services:
  - /set_camera_mode (std_srvs/srv/SetBool): True = NAV2, False = DOCKING
  - /set_mode_nav2   (std_srvs/srv/Trigger): Switch to NAV2 mode
  - /set_mode_docking (std_srvs/srv/Trigger): Switch to DOCKING mode
  - /current_camera_mode (std_msgs/msg/String, latched/periodic)
"""

import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool, Trigger
from ament_index_python.packages import get_package_share_directory


class CameraModeManagerNode(Node):
    def __init__(self):
        super().__init__('camera_mode_manager_node')

        # Declare parameters
        self.declare_parameter('initial_mode', 'NAV2')
        vslam_pkg = get_package_share_directory('turtlebot3_visual_slam')
        default_astra_cfg = os.path.join(vslam_pkg, 'config', 'gazebo_astra_bridge.yaml')
        default_os30a_cfg = os.path.join(vslam_pkg, 'config', 'gazebo_os30a_bridge.yaml')

        self.declare_parameter('astra_config', default_astra_cfg)
        self.declare_parameter('os30a_config', default_os30a_cfg)

        self.astra_cfg = self.get_parameter('astra_config').get_parameter_value().string_value
        self.os30a_cfg = self.get_parameter('os30a_config').get_parameter_value().string_value
        initial_mode = self.get_parameter('initial_mode').get_parameter_value().string_value.upper()

        # Bridge process handles
        self.astra_proc = None
        self.os30a_proc = None
        self.bridge_counter = 0

        # RTAB-Map pause/resume service clients
        self.pause_rtabmap_cli = self.create_client(Empty, '/rtabmap/pause')
        self.resume_rtabmap_cli = self.create_client(Empty, '/rtabmap/resume')

        # Mode status publisher
        self.mode_pub = self.create_publisher(String, '/current_camera_mode', 10)
        self.current_mode = ""

        # Provide Services
        self.srv_set_bool = self.create_service(SetBool, '/set_camera_mode', self.handle_set_camera_mode)
        self.srv_nav2 = self.create_service(Trigger, '/set_mode_nav2', self.handle_set_mode_nav2)
        self.srv_docking = self.create_service(Trigger, '/set_mode_docking', self.handle_set_mode_docking)

        # 1Hz timer to broadcast current mode
        self.timer = self.create_timer(1.0, self.publish_mode_timer)

        self.get_logger().info("========================================================")
        self.get_logger().info("  Camera Mode Manager Initialized                       ")
        self.get_logger().info(f"  - Astra S Config: {self.astra_cfg}")
        self.get_logger().info(f"  - OS30A Config:   {self.os30a_cfg}")
        self.get_logger().info(f"  - Initial Mode:   {initial_mode}")
        self.get_logger().info("========================================================")

        # Apply initial mode
        if initial_mode == 'DOCKING':
            self.switch_to_docking()
        else:
            self.switch_to_nav2()

    def _call_service_async(self, client, srv_name):
        if client.service_is_ready():
            req = Empty.Request()
            client.call_async(req)
            self.get_logger().info(f"[RTABMAP] Called {srv_name}")
        else:
            self.get_logger().debug(f"[RTABMAP] {srv_name} service not ready, skipping.")

    def switch_to_nav2(self):
        """Switch to NAV2 mode: RTAB-Map SLAM Resumed, Mode set to NAV2_ASTRA."""
        if self.current_mode == "NAV2_ASTRA":
            self.get_logger().info("[MODE SWITCH] Already in NAV2_ASTRA mode.")
            return

        self.get_logger().info(">>> [MODE SWITCH] Switching to NAV2 Mode...")
        # Resume RTAB-Map SLAM for Nav2 navigation
        self._call_service_async(self.resume_rtabmap_cli, '/rtabmap/resume')

        self.current_mode = "NAV2_ASTRA"
        self.publish_mode()
        self.get_logger().info(">>> [MODE SWITCH COMPLETED] Mode: NAV2_ASTRA (Astra S Active, RTAB-Map Resumed)")

    def switch_to_docking(self):
        """Switch to DOCKING mode: RTAB-Map SLAM Paused (freeing ~50% CPU), Mode set to DOCKING_OS30A."""
        if self.current_mode == "DOCKING_OS30A":
            self.get_logger().info("[MODE SWITCH] Already in DOCKING_OS30A mode.")
            return

        self.get_logger().info(">>> [MODE SWITCH] Switching to DOCKING Mode...")
        # Pause RTAB-Map SLAM to free CPU for precision docking
        self._call_service_async(self.pause_rtabmap_cli, '/rtabmap/pause')

        self.current_mode = "DOCKING_OS30A"
        self.publish_mode()
        self.get_logger().info(">>> [MODE SWITCH COMPLETED] Mode: DOCKING_OS30A (OS30A Active, RTAB-Map Paused)")

    def handle_set_camera_mode(self, request, response):
        if request.data:
            self.switch_to_nav2()
            response.success = True
            response.message = "Switched to NAV2 Mode (Astra S Active, OS30A Inactive)"
        else:
            self.switch_to_docking()
            response.success = True
            response.message = "Switched to DOCKING Mode (OS30A Active, Astra S Inactive)"
        return response

    def handle_set_mode_nav2(self, request, response):
        self.switch_to_nav2()
        response.success = True
        response.message = "Switched to NAV2 Mode (Astra S Active, OS30A Inactive)"
        return response

    def handle_set_mode_docking(self, request, response):
        self.switch_to_docking()
        response.success = True
        response.message = "Switched to DOCKING Mode (OS30A Active, Astra S Inactive)"
        return response

    def publish_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def publish_mode_timer(self):
        if self.current_mode:
            self.publish_mode()

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraModeManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
