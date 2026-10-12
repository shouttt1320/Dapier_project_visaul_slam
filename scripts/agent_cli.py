#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[Qwen2.5-VL Agent Interactive CLI Client]
Interactive terminal console for sending commands to LLM Agent and Task Manager.
Runs independently in any terminal with zero log noise.
"""

import sys
import os
import time
import json
import readline
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool


class TermColor:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"


class AgentCLIClient(Node):
    def __init__(self):
        super().__init__('agent_cli_client')

        # Publishers
        self.cmd_pub = self.create_publisher(String, '/user_voice_command', 10)
        self.query_pub = self.create_publisher(String, '/qwen/detect_query', 10)
        self.stop_pub = self.create_publisher(Bool, '/safety/software_stop', 10)

        # Subscribers
        self.create_subscription(String, '/llm/agent_response', self._resp_cb, 10)
        self.create_subscription(String, '/qwen/target_bbox', self._bbox_cb, 10)
        self.create_subscription(String, '/task/command', self._task_cb, 10)
        self.create_subscription(Bool, '/safety/software_stop', self._stop_cb, 10)

    def _resp_cb(self, msg: String):
        print(f"\n{TermColor.GREEN}💬 [에이전트 응답] {msg.data}{TermColor.RESET}\n🤖 명령 입력 >> ", end="", flush=True)

    def _bbox_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("target_found"):
                box = data.get("pixel_box", data.get("bbox_2d"))
                label = data.get("label", "object")
                latency = data.get("latency_sec", 0.0) * 1000
                print(f"\n{TermColor.YELLOW}👁️ [시각 발견] '{label}' 좌표: {box} ({latency:.1f}ms){TermColor.RESET}\n🤖 명령 입력 >> ", end="", flush=True)
            else:
                print(f"\n{TermColor.RED}👁️ [시각 질의 실패] 물체를 찾지 못했습니다.{TermColor.RESET}\n🤖 명령 입력 >> ", end="", flush=True)
        except Exception:
            pass

    def _task_cb(self, msg: String):
        try:
            task = json.loads(msg.data)
            action = task.get("action", "UNKNOWN")
            print(f"\n{TermColor.BLUE}📋 [작업 등록] Action: {action}{TermColor.RESET}\n🤖 명령 입력 >> ", end="", flush=True)
        except Exception:
            pass

    def _stop_cb(self, msg: Bool):
        if msg.data:
            print(f"\n{TermColor.RED}🛑 [비상 정지] 로봇 긴급 정지 신호 수신!{TermColor.RESET}\n🤖 명령 입력 >> ", end="", flush=True)


def main():
    os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "101")
    rclpy.init()
    node = AgentCLIClient()

    # Spin ROS 2 callbacks in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    banner = f"""
{TermColor.CYAN}╔══════════════════════════════════════════════════════════════════════════╗
║               🎮 QWEN2.5-VL INTERACTIVE CLI CONSOLE                      ║
║  - 주행 명령: '책상으로 가줘', '앞으로 1m 이동해', '좌표 X 1.5, Y 0.5로 가' ║
║  - 도킹 명령: '빨간 상자가 있는 책상에 도킹해', '상자에 도킹해줘'          ║
║  - 시각 질의: '? 텀블러', '? 펜', '? 상자' (실시간 BBox 시각 검증)        ║
║  - 비상 정지: '정지', '멈춰', 'stop'                                      ║
╚══════════════════════════════════════════════════════════════════════════╝{TermColor.RESET}
"""
    print(banner, flush=True)

    try:
        while rclpy.ok():
            try:
                line = input(f"{TermColor.CYAN}🤖 명령 입력 >> {TermColor.RESET}").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n종료합니다...")
                break

            if not line:
                continue

            # 비상 정지 키워드
            if line in ['정지', '멈춰', 'stop', '비상정지', 'STOP']:
                print(f"{TermColor.RED}🚨 긴급 정지 신호 전송!{TermColor.RESET}")
                node.stop_pub.publish(Bool(data=True))
                continue

            # 시각 질의 (? 물체이름)
            if line.startswith("?"):
                query = line[1:].strip()
                if query:
                    print(f">> 🔎 물체 시각 검색 질의 전송: '{query}'...")
                    node.query_pub.publish(String(data=query))
                continue

            # 자연어 명령 전송
            print(f">> 🚀 음성/텍스트 명령 전송: '{line}'...")
            node.cmd_pub.publish(String(data=line))

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
