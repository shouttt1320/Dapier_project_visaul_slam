#!/usr/bin/env python3
"""
[Target Object Image Capturer]
- Astra S 카메라 실시간 스트림(/camera/color/image_raw)을 보며 도킹 타겟 물체 사진을 캡처하는 도구
- 사용법:
  1. 카메라 앞에 도킹할 빨간 상자를 비춥니다.
  2. 초록색 가이드 사각형 안에 상자를 맞춥니다.
  3. [SPACE] 바를 누르면 target_images/red_box.jpg 로 자동 저장됩니다.
  4. [q] 또는 ESC를 누르면 종료됩니다.
"""

import os
import sys
import cv2
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class ImageCapturerNode(Node):
    def __init__(self):
        super().__init__('target_image_capturer')
        self.bridge = CvBridge()
        self.last_img = None

        self.create_subscription(Image, '/camera/color/image_raw', self._cb, 10)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self._comp_cb, 10)

    def _cb(self, msg: Image):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _comp_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.last_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception:
            pass


def main():
    rclpy.init()
    node = ImageCapturerNode()

    save_dir = os.path.join(PROJECT_ROOT, "target_images")
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, "red_box.jpg")

    window_name = "Target Object Capturer (Astra S)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)

    print("\n" + "=" * 70)
    print(" 📷 TARGET OBJECT IMAGE CAPTURER")
    print("=" * 70)
    print(" 1. Align the red target object inside the green guide box.")
    print(" 2. Press [SPACE] to capture and save to target_images/red_box.jpg.")
    print(" 3. Press [q] or [ESC] to quit.")
    print("=" * 70 + "\n")

    captured = False

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)

            if node.last_img is not None:
                frame = node.last_img.copy()
                h, w = frame.shape[:2]

                # Center Guide Box (140x140)
                box_size = 140
                gx1 = (w - box_size) // 2
                gy1 = (h - box_size) // 2
                gx2 = gx1 + box_size
                gy2 = gy1 + box_size

                cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), (0, 255, 0), 2)
                cv2.putText(frame, "ALIGN OBJECT HERE", (gx1, gy1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # Instruction Banner
                cv2.rectangle(frame, (0, h - 35), (w, h), (30, 30, 30), -1)
                cv2.putText(frame, "PRESS [SPACE] TO CAPTURE | [q] TO QUIT", (15, h - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

                if captured:
                    cv2.putText(frame, "SAVED SUCCESSFULLY!", (15, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

                cv2.imshow(window_name, frame)

                key = cv2.waitKey(1) & 0xFF
                if key == 32:  # SPACE
                    # Crop center region with margin
                    crop = node.last_img[max(0, gy1-10):min(h, gy2+10), max(0, gx1-10):min(w, gx2+10)]
                    cv2.imwrite(save_path, crop)
                    full_path = os.path.join(save_dir, "red_box_full.jpg")
                    cv2.imwrite(full_path, node.last_img)
                    print(f">> [CAPTURED] Target image saved to:\n   - {save_path}\n   - {full_path}")
                    captured = True
                elif key == ord('q') or key == 27:
                    break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
