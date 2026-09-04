#!/usr/bin/env python3
"""
YDLIDAR OS30A Real-Time Camera & Depth Map Viewer
Displays Camera Stream (Left) and Colorized Depth Map (Right) side-by-side.
"""

import sys
import time
import cv2
import numpy as np


def main():
    print("==================================================")
    print(" [YDLIDAR OS30A Live Viewer]")
    print(" Initializing camera and depth streams...")
    print("==================================================")

    # Device index: video4 for camera, video6 for depth
    dev_camera = 4
    dev_depth = 6

    cap_cam = cv2.VideoCapture(dev_camera, cv2.CAP_V4L2)
    cap_depth = cv2.VideoCapture(dev_depth, cv2.CAP_V4L2)
    cap_depth.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)

    if not cap_cam.isOpened():
        print(f"Error: Could not open /dev/video{dev_camera} for camera stream.")
        sys.exit(1)

    if not cap_depth.isOpened():
        print(f"Error: Could not open /dev/video{dev_depth} for depth stream.")
        cap_cam.release()
        sys.exit(1)

    window_name = "YDLIDAR OS30A: Camera (Left) & Depth Map (Right)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 460)

    print("\nViewer running. Press 'q' or ESC in the window to exit.")

    fps_start = time.time()
    frame_count = 0
    fps_text = "FPS: --"

    try:
        while True:
            ret_cam, frame_cam = cap_cam.read()
            ret_depth, raw_depth = cap_depth.read()

            if not ret_cam or not ret_depth:
                time.sleep(0.01)
                continue

            # Camera: Take left 640x460 view
            h, w = frame_cam.shape[:2]
            if w >= 1280:
                cam_view = frame_cam[:, :640]
            else:
                cam_view = frame_cam

            # Depth: Convert 2-channel uint8 to 16-bit millimeter depth
            depth16 = raw_depth[:, :, 0].astype(np.uint16) | (raw_depth[:, :, 1].astype(np.uint16) << 8)

            # Center distance calculation
            ch, cw = depth16.shape
            center_val = depth16[ch // 2 - 5:ch // 2 + 5, cw // 2 - 5:cw // 2 + 5]
            valid_center = center_val[center_val > 0]
            if len(valid_center) > 0:
                center_dist_m = np.median(valid_center) / 1000.0
                dist_text = f"Center: {center_dist_m:.2f} m"
            else:
                dist_text = "Center: Out of range"

            # Normalize depth to 200mm ~ 2500mm (OS30A spec range)
            depth_clipped = np.clip(depth16, 200, 2500)
            depth_norm = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            depth_color[depth16 == 0] = [0, 0, 0]  # Black for no return / out of range

            # Draw center crosshair and info on depth
            cv2.drawMarker(depth_color, (cw // 2, ch // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.drawMarker(cam_view, (cw // 2, ch // 2), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

            # Calculate FPS
            frame_count += 1
            if frame_count >= 15:
                now = time.time()
                fps = frame_count / (now - fps_start)
                fps_text = f"FPS: {fps:.1f}"
                fps_start = now
                frame_count = 0

            # Overlay text
            cv2.putText(cam_view, "Camera Stream", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cam_view, fps_text, (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.putText(depth_color, "Depth Colormap (0.2m ~ 2.5m)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(depth_color, dist_text, (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Horizontal concatenation: Left = Camera, Right = Depth
            combined = np.hstack((cam_view, depth_color))

            cv2.imshow(window_name, combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    finally:
        cap_cam.release()
        cap_depth.release()
        cv2.destroyAllWindows()
        print("\nViewer closed successfully.")


if __name__ == '__main__':
    main()
