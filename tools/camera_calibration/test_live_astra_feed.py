#!/usr/bin/env python3
import os
import sys
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ws_src = os.path.join(PROJECT_ROOT, "robot_ws", "src", "llm_semantic_docker")
sys.path.insert(0, ws_src)

import rclpy
from llm_semantic_docker.object_detector_3d import ObjectDetector3DNode

def main():
    rclpy.init()
    node = ObjectDetector3DNode(enable_terminal_ui=False)
    start = time.time()
    frame_count = 0

    print(">> 3초간 실제 Astra S 3D 비전 노드 실행 중...")
    while time.time() - start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.last_color_img is not None and node.last_depth_raw is not None:
            frame_count += 1

    print(f">> 실제 프레임 수신 및 처리 완료: {frame_count} 프레임 처리됨 (FPS: {node.current_fps:.1f})")
    print(f">> 타겟 락 상태: {node.is_target_locked}")
    if node.smoothed_center is not None:
        print(f">> 검출된 3D 포즈: {node.smoothed_center}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
