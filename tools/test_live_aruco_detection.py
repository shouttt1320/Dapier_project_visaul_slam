#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
 [Live ArUco Marker Detection & Dual-Marker Diagnostics Tool]
  - Subscribes to live camera feed (/camera/color/image_decompressed)
  - Visualizes detected markers (ID, 3D Pose Axes, Distance)
  - Diagnoses Dual-Marker vs Single-Marker state with live span calculation
  - Shows rejected candidates in red to identify unclassified markers
  - Interactive keys:
      'q' or ESC : Exit
      'p'        : Toggle parameter preset (Tuned vs Default)
      's'        : Save snapshot image with debug annotations
==============================================================================
"""

import sys
import os
import time
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class LiveArucoDetector(Node):
    def __init__(self):
        super().__init__('live_aruco_detector')
        self.bridge = CvBridge()

        # Camera Intrinsics (Astra S 320x240 Calibrated)
        self.fx = 285.17
        self.fy = 285.17
        self.cx = 159.5
        self.cy = 119.5
        self.camera_matrix = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.marker_size = 0.060  # 60mm (0.06m)

        # ArUco Dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Detection parameter presets: Tuned vs Default
        self.use_tuned_params = True

        # Tuned parameters: Multi-scale window (min=5, max=45, step=8, const=8.0)
        # Prevents close-range window saturation when markers become large on screen
        self.params_tuned = self._create_detector_params(
            win_min=5, win_max=45, win_step=8, const=8.0, border_err=0.35, corr_rate=0.70
        )
        # Default parameters
        self.params_default = self._create_detector_params(
            win_min=3, win_max=23, win_step=4, const=7.0, border_err=0.25, corr_rate=0.60
        )

        # Image reception
        self.last_img = None
        self.img_count = 0
        self.fps = 0.0
        self._fps_time = time.time()
        self._frame_count = 0

        # Subscriptions
        self.create_subscription(
            Image, '/camera/color/image_decompressed', self._on_image, 10
        )
        self.create_subscription(
            CameraInfo, '/camera/color/decompressed/camera_info', self._on_cam_info, 10
        )

        self.get_logger().info("🔍 [Live Aruco Detector] Initialized. Subscribed to /camera/color/image_decompressed")

    def _create_detector_params(self, win_min=5, win_max=25, win_step=10, const=9.0, border_err=0.35, corr_rate=0.70):
        try:
            p = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            p = cv2.aruco.DetectorParameters()
        p.adaptiveThreshWinSizeMin = win_min
        p.adaptiveThreshWinSizeMax = win_max
        p.adaptiveThreshWinSizeStep = win_step
        p.adaptiveThreshConstant = const
        p.minMarkerPerimeterRate = 0.03
        p.maxErroneousBitsInBorderRate = border_err
        p.errorCorrectionRate = corr_rate
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return p

    def _on_cam_info(self, msg: CameraInfo):
        if len(msg.k) == 9 and msg.k[0] > 10.0:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            if len(msg.d) >= 4:
                self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

    def _on_image(self, msg: Image):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.img_count += 1
            self._frame_count += 1
            now = time.time()
            if now - self._fps_time >= 1.0:
                self.fps = self._frame_count / (now - self._fps_time)
                self._frame_count = 0
                self._fps_time = now
        except Exception as e:
            self.get_logger().error(f"Image decode error: {e}")

    def process_and_visualize(self):
        if self.last_img is None:
            return None

        img = self.last_img.copy()
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        active_params = self.params_tuned if self.use_tuned_params else self.params_default
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=active_params)

        # Canvas for display (upscaled 2x to 640x480 for clear text & visibility)
        vis = cv2.resize(img, (w * 2, h * 2), interpolation=cv2.INTER_NEAREST)

        detected_markers = []
        if ids is not None and len(ids) > 0:
            # Estimate 3D pose
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for idx, mid in enumerate(ids.flatten()):
                c = (corners[idx][0] * 2.0).astype(int)  # scaled for display
                t = tvecs[idx][0]
                r = rvecs[idx][0]
                dist_m = float(np.linalg.norm(t))

                center_x = int(np.mean(c[:, 0]))
                center_y = int(np.mean(c[:, 1]))

                detected_markers.append({
                    'id': int(mid),
                    'tvec': t,
                    'dist': dist_m,
                    'center': (center_x, center_y),
                    'corners_scaled': c
                })

                # Draw green polygon
                cv2.polylines(vis, [c], True, (0, 255, 0), 2)
                # Corner 0 marker (red dot for orientation)
                cv2.circle(vis, (c[0, 0], c[0, 1]), 4, (0, 0, 255), -1)

                # Label
                label = f"ID:{mid} ({dist_m*100:.1f}cm)"
                cv2.putText(vis, label, (center_x - 40, center_y - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

                # 3D axes
                try:
                    axis_len = 0.04
                    img_pts, _ = cv2.projectPoints(
                        np.array([[0,0,0], [axis_len,0,0], [0,axis_len,0], [0,0,axis_len]], dtype=np.float64),
                        r, t, self.camera_matrix, self.dist_coeffs
                    )
                    origin = (int(img_pts[0,0,0]*2), int(img_pts[0,0,1]*2))
                    pt_x = (int(img_pts[1,0,0]*2), int(img_pts[1,0,1]*2))
                    pt_y = (int(img_pts[2,0,0]*2), int(img_pts[2,0,1]*2))
                    pt_z = (int(img_pts[3,0,0]*2), int(img_pts[3,0,1]*2))
                    cv2.line(vis, origin, pt_x, (0, 0, 255), 2)  # X: Red
                    cv2.line(vis, origin, pt_y, (0, 255, 0), 2)  # Y: Green
                    cv2.line(vis, origin, pt_z, (255, 0, 0), 2)  # Z: Blue
                except Exception:
                    pass

        # Draw rejected candidates in faint red/dotted
        if rejected is not None:
            for r in rejected:
                r_pts = (r[0] * 2.0).astype(int)
                cv2.polylines(vis, [r_pts], True, (60, 60, 220), 1)

        # Sort detected markers left-to-right (by X coordinate in camera space)
        detected_markers.sort(key=lambda m: m['tvec'][0])

        # Header overlay
        header_color = (30, 30, 30)
        cv2.rectangle(vis, (0, 0), (w * 2, 50), header_color, -1)

        mode_str = "TUNED (min=5, const=9)" if self.use_tuned_params else "DEFAULT (min=3, const=7)"
        cv2.putText(vis, f"FPS: {self.fps:.1f} | Params: {mode_str} ['P' to Toggle]", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # Dual Marker Status Banner
        if len(detected_markers) >= 2:
            m_left = detected_markers[0]
            m_right = detected_markers[-1]
            p_l = m_left['tvec']
            p_r = m_right['tvec']
            span = float(np.linalg.norm(p_r - p_l))
            midpoint_3d = (p_l + p_r) / 2.0
            mid_dist = float(np.linalg.norm(midpoint_3d))

            # Angle from marker normal
            dx = float(p_r[0] - p_l[0])
            dz = float(p_r[2] - p_l[2])
            normal_angle_deg = math.degrees(math.atan2(dz, dx)) - 90.0

            # Line between markers
            cv2.line(vis, m_left['center'], m_right['center'], (255, 255, 0), 2)
            mid_center_px = (
                int((m_left['center'][0] + m_right['center'][0]) / 2),
                int((m_left['center'][1] + m_right['center'][1]) / 2)
            )
            cv2.circle(vis, mid_center_px, 5, (0, 255, 255), -1)

            banner_text = (
                f"✅ DUAL MARKER LOCKED: Left=ID{m_left['id']}, Right=ID{m_right['id']} | "
                f"Span: {span*100:.1f}cm | Dist: {mid_dist*100:.1f}cm | Yaw: {normal_angle_deg:+.1f}deg"
            )
            cv2.putText(vis, banner_text, (10, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
        elif len(detected_markers) == 1:
            mid = detected_markers[0]['id']
            dist = detected_markers[0]['dist']
            banner_text = f"⚠️ ONLY 1 MARKER DETECTED: ID={mid} ({dist*100:.1f}cm) -> 2nd marker missing!"
            cv2.putText(vis, banner_text, (10, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 2)
        else:
            banner_text = "❌ NO MARKERS DETECTED (Searching...)"
            cv2.putText(vis, banner_text, (10, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

        # Footer guide
        cv2.putText(vis, "Keys: [P] Toggle Params | [S] Save Snapshot | [Q/ESC] Quit",
                    (10, h * 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)

        return vis


def main():
    rclpy.init()
    node = LiveArucoDetector()

    window_name = "Live ArUco Dual-Marker Diagnostics"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    print("=" * 70)
    print(" 🚀 [Live ArUco Marker Detector Started]")
    print("  - Camera Topic: /camera/color/image_decompressed")
    print("  - Keys:")
    print("      'p'     : Toggle Tuned vs Default parameters")
    print("      's'     : Save snapshot image to workspace")
    print("      'q'/ESC : Quit")
    print("=" * 70)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            vis = node.process_and_visualize()
            if vis is not None:
                cv2.imshow(window_name, vis)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break
            elif key == ord('p'):
                node.use_tuned_params = not node.use_tuned_params
                print(f"🔄 Switched parameters to: {'TUNED (min=5, c=9)' if node.use_tuned_params else 'DEFAULT (min=3, c=7)'}")
            elif key == ord('s'):
                snap_path = f"/home/jjj/Documents/Dapier/Project/aruco_snapshot_{int(time.time())}.jpg"
                if vis is not None:
                    cv2.imwrite(snap_path, vis)
                    print(f"📸 Snapshot saved to: {snap_path}")

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
