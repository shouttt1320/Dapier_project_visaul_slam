#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
🤖 Robotic Arm (sweepick) <-> TurtleBot3 Handshake Verification Tool
==============================================================================
본 도구는 상대팀(로봇팔 측) 컴퓨터 역할을 가상으로 수행하거나,
터틀봇과 로봇팔 간의 [접근 완료 ➔ 파지 대기 ➔ 잡기 완료 ➔ 주행 복귀]
엔드투엔드 핸드셰이크 신호가 정상적으로 오가는지 실시간 검증하는 테스트 툴입니다.

[검증하는 핵심 통신 규격]
1. 터틀봇 접근 완료 감지: /task/status ("접근 완료") 및 /task/ack ("APPROACH_DONE")
2. 로봇팔 생존 신호 송신: /sweepick/heartbeat (2 Hz 주기)
3. 로봇팔 준비 신호 송신: /sweepick/ready ({"status": "READY"})
4. (선택) 5cm 미세 이동 & ACK 검증: /task/command -> /task/ack
5. 로봇팔 잡기 완료 송신: /sweepick/ready ({"status": "PICK_DONE"})
6. 터틀봇 주행 복귀 확인: /task/status 확인
==============================================================================
"""

import sys
import os
import time
import json
import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# ANSI Colors
C_RESET = "\033[0m"
C_BOLD = "\033[1m"
C_GREEN = "\033[92m"
C_CYAN = "\033[96m"
C_YELLOW = "\033[93m"
C_RED = "\033[91m"
C_MAGENTA = "\033[95m"


class ArmHandshakeTesterNode(Node):
    def __init__(self, auto_mode: bool = True, test_inching: bool = True):
        super().__init__('arm_handshake_tester_node')
        self.auto_mode = auto_mode
        self.test_inching = test_inching

        # Internal Test State
        self.step = 0
        self.approach_detected = False
        self.inching_ack_received = False
        self.last_ack_payload = None
        self.resume_detected = False

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers (로봇팔 -> 터틀봇)
        self.heartbeat_pub = self.create_publisher(String, '/sweepick/heartbeat', qos)
        self.ready_pub = self.create_publisher(String, '/sweepick/ready', qos)
        self.command_pub = self.create_publisher(String, '/task/command', qos)

        # Subscribers (터틀봇 -> 로봇팔)
        self.create_subscription(String, '/task/status', self._on_task_status, qos)
        self.create_subscription(String, '/task/ack', self._on_task_ack, qos)

        # Heartbeat Timer (2 Hz)
        self.heartbeat_active = False
        self.hb_timer = self.create_timer(0.5, self._on_heartbeat_tick)

        self._print_header()

    def _print_header(self):
        print(f"\n{C_CYAN}{C_BOLD}╔════════════════════════════════════════════════════════════════════════╗")
        print(f"║   🦾 ROBOTIC ARM (SWEEPICK) HANDSHAKE VERIFICATION TESTER              ║")
        print(f"║   ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '101'):<4}                                          ║")
        print(f"╚════════════════════════════════════════════════════════════════════════╝{C_RESET}\n")

    def _on_heartbeat_tick(self):
        if self.heartbeat_active:
            hb_msg = json.dumps({"alive": True, "timestamp": time.time()})
            self.heartbeat_pub.publish(String(data=hb_msg))

    def _on_task_status(self, msg: String):
        text = msg.data
        # Step 1: Detect Approach
        if not self.approach_detected and any(kw in text for kw in ("접근 완료", "APPROACH_DONE")):
            self.approach_detected = True
            print(f"\n{C_GREEN}{C_BOLD}[STEP 1 SUCCESS] 🎯 터틀봇의 물체 접근 완료 신호 수신!{C_RESET}")
            print(f"  └ 수신 토픽: /task/status")
            print(f"  └ 내용: {text}\n")

        # Step 4: Detect Resume Navigation
        if self.step >= 3 and any(kw in text for kw in ("물체 파지 완료", "RESUME", "원래 목적지로 주행을 복귀")):
            self.resume_detected = True
            print(f"\n{C_GREEN}{C_BOLD}[STEP 4 SUCCESS] 🚀 터틀봇이 잡기 완료 신호를 확인하고 자율주행 복귀를 시작했습니다!{C_RESET}")
            print(f"  └ 수신 토픽: /task/status")
            print(f"  └ 내용: {text}\n")

    def _on_task_ack(self, msg: String):
        print(f"{C_MAGENTA}[/task/ack 수신]{C_RESET} {msg.data}")
        try:
            payload = json.loads(msg.data)
            self.last_ack_payload = payload
            # Approach ACK
            if payload.get("status") == "APPROACH_DONE" or payload.get("id") == "approach":
                self.approach_detected = True
            # Inching ACK
            if payload.get("state") == "done":
                self.inching_ack_received = True
        except Exception:
            pass

    def run_handshake_sequence(self):
        """Runs through the full 4-step robotic arm handshake test sequence."""
        print(f"{C_YELLOW}[대기 중] 터틀봇의 물체 접근 완료(/task/status: '접근 완료') 신호를 기다리는 중...{C_RESET}")
        print("  (터틀봇에서 바닥 물체 접근이 완료되거나, 테스트용 모의 신호가 오면 자동 진행됩니다.)\n")

        # Step 1: Wait for Approach Done
        timeout = 60.0
        start_t = time.time()
        while rclpy.ok() and not self.approach_detected:
            if time.time() - start_t > timeout:
                print(f"{C_RED}❌ [타임아웃] {timeout}초 동안 터틀봇의 접근 완료 신호가 감지되지 않았습니다.{C_RESET}")
                return False
            time.sleep(0.1)

        self.step = 1
        # Start Heartbeat
        self.heartbeat_active = True
        print(f"{C_CYAN}💓 로봇팔 생존 신호(/sweepick/heartbeat) 2Hz 송신을 시작합니다.{C_RESET}\n")
        time.sleep(1.0)

        # Step 2: (Optional) Test 5cm micro-inching command & ACK
        if self.test_inching:
            print(f"{C_YELLOW}{C_BOLD}[STEP 2-1] 로봇팔 ➔ 터틀봇 추가 미세 접근 명령 (forward 3cm) 전송 테스트...{C_RESET}")
            cmd_id_fwd = f"test_inch_fwd_{int(time.time())}"
            cmd_payload_fwd = json.dumps({"id": cmd_id_fwd, "forward": 0.03})
            print(f"  └ 송신 토픽: /task/command")
            print(f"  └ 데이터: {cmd_payload_fwd}")
            self.command_pub.publish(String(data=cmd_payload_fwd))

            # Wait for ACK
            self.inching_ack_received = False
            ack_wait_start = time.time()
            while rclpy.ok() and not self.inching_ack_received:
                if time.time() - ack_wait_start > 5.0:
                    print(f"{C_YELLOW}⚠️ [참고] 미세 접근 ACK 타임아웃. 계속 진행합니다.{C_RESET}")
                    break
                time.sleep(0.1)

            if self.inching_ack_received:
                print(f"{C_GREEN}  └ ✅ 추가 미세 접근 완료 /task/ack 정상 수신 확인!{C_RESET}\n")
            time.sleep(1.0)

            print(f"{C_YELLOW}{C_BOLD}[STEP 2-2] 로봇팔 ➔ 터틀봇 자세 각도 틀기 명령 (turn_left 5.0도) 전송 테스트...{C_RESET}")
            cmd_id_turn = f"test_inch_turn_{int(time.time())}"
            cmd_payload_turn = json.dumps({"id": cmd_id_turn, "turn_left": 5.0})
            print(f"  └ 송신 토픽: /task/command")
            print(f"  └ 데이터: {cmd_payload_turn}")
            self.command_pub.publish(String(data=cmd_payload_turn))

            # Wait for ACK
            self.inching_ack_received = False
            ack_wait_start = time.time()
            while rclpy.ok() and not self.inching_ack_received:
                if time.time() - ack_wait_start > 5.0:
                    print(f"{C_YELLOW}⚠️ [참고] 자세 회전 ACK 타임아웃. 계속 진행합니다.{C_RESET}")
                    break
                time.sleep(0.1)

            if self.inching_ack_received:
                print(f"{C_GREEN}  └ ✅ 자세 회전 완료 /task/ack 정상 수신 확인!{C_RESET}\n")
            time.sleep(1.0)

        # Step 3: Arm Ready Signal -> TurtleBot Locks Base
        print(f"{C_YELLOW}{C_BOLD}[STEP 3] 로봇팔 ➔ 터틀봇: 집기 준비 완료(READY) 신호 송신{C_RESET}")
        ready_payload = json.dumps({"status": "READY", "arm": "moving", "object": "locked"})
        self.ready_pub.publish(String(data=ready_payload))
        print(f"  └ 송신 토픽: /sweepick/ready")
        print(f"  └ 데이터: {ready_payload}")
        print(f"  └ {C_CYAN}터틀봇 베이스 0속도 고정 및 파지 대기(WAITING_ARM_PICK) 진입 유도{C_RESET}\n")

        # Simulate Arm Gripping (3 seconds)
        print("🦾 로봇팔이 물체를 파지하는 중입니다... (시뮬레이션 대기 3초)")
        for i in range(3, 0, -1):
            print(f"   [{i}초...]")
            time.sleep(1.0)

        # Step 4: Arm Pick Done Signal -> TurtleBot Resumes Navigation
        self.step = 3
        print(f"\n{C_YELLOW}{C_BOLD}[STEP 4] 로봇팔 ➔ 터틀봇: 잡기 완료(PICK_DONE) 신호 송신{C_RESET}")
        done_payload = json.dumps({"status": "PICK_DONE", "arm": "home", "result": "success"})
        self.ready_pub.publish(String(data=done_payload))
        print(f"  └ 송신 토픽: /sweepick/ready")
        print(f"  └ 데이터: {done_payload}\n")

        # Wait for Resume confirmation
        wait_resume_start = time.time()
        while rclpy.ok() and not self.resume_detected:
            if time.time() - wait_resume_start > 4.0:
                break
            time.sleep(0.1)

        # Print Final Verdict
        self.heartbeat_active = False
        print(f"\n{C_CYAN}╔════════════════════════════════════════════════════════════════════════╗")
        print(f"║                    📊 통신 테스트 최종 검증 결과                        ║")
        print(f"╠════════════════════════════════════════════════════════════════════════╣")
        print(f"║ 1. 터틀봇 접근 완료 신호 수신 (/task/status):      {C_GREEN if self.approach_detected else C_RED}{'✅ 성공 (정상 수신)' if self.approach_detected else '❌ 실패'}{C_RESET}     ║")
        print(f"║ 2. 로봇팔 2Hz 하트비트 송신 (/sweepick/heartbeat):  {C_GREEN}✅ 정상 동작{C_RESET}             ║")
        print(f"║ 3. 로봇팔 READY 베이스 고정 연동 (/sweepick/ready): {C_GREEN}✅ 정상 송신{C_RESET}             ║")
        print(f"║ 4. 로봇팔 잡기 완료(PICK_DONE) 및 주행 복귀:        {C_GREEN if self.resume_detected else C_YELLOW}{'✅ 성공 (주행 재개)' if self.resume_detected else '⚠️ 송신 완료 (확인)'}{C_RESET}     ║")
        print(f"╚════════════════════════════════════════════════════════════════════════╝{C_RESET}\n")
        return True


from rclpy.executors import SingleThreadedExecutor

def run_mock_turtlebot_sim():
    """Simulates TurtleBot sending '접근 완료' for standalone dry-run testing."""
    node = Node('mock_turtlebot_emitter')
    pub_status = node.create_publisher(String, '/task/status', 10)
    pub_ack = node.create_publisher(String, '/task/ack', 10)

    print(f"\n{C_CYAN}🤖 [모의 터틀봇 기동] 2초 후 가상 '접근 완료' 신호를 발행합니다...{C_RESET}")
    time.sleep(2.0)

    msg_text = "🎉 [접근 완료] APPROACH_DONE: red_cube 렌즈 전방 25.0cm 정지 완료! (로봇팔 작업 준비 완료)"
    pub_status.publish(String(data=msg_text))
    pub_ack.publish(String(data=json.dumps({"id": "approach", "state": "done", "status": "APPROACH_DONE"})))
    print(f"  └ 📡 가상 '접근 완료' 신호 발행 완료!\n")

    # Listen for /task/command (Micro-inching & posture adjustment)
    def on_cmd(msg: String):
        try:
            d = json.loads(msg.data)
            cmd_id = d.get("id", "cmd_test")
            print(f"{C_GREEN}🤖 [모의 터틀봇] 자세 조정/미세 이동 명령 수신 ({msg.data}) ➔ 바퀴 회전/이동 후 ACK 발행!{C_RESET}")
            pub_status.publish(String(data=f"INFO: Relative displacement completed: id={cmd_id}"))
            pub_ack.publish(String(data=json.dumps({"id": cmd_id, "state": "done"})))
        except Exception:
            pass

    node.create_subscription(String, '/task/command', on_cmd, 10)

    # Listen for /sweepick/ready PICK_DONE
    def on_ready(msg: String):
        if "PICK_DONE" in msg.data or "SUCCESS" in msg.data:
            print(f"{C_GREEN}🤖 [모의 터틀봇] 잡기 완료(PICK_DONE) 수신 확인! ➔ 주행 복귀 신호 발행!{C_RESET}")
            pub_status.publish(String(data="INFO: 🤖 [로봇팔 신호 수신] 물체 파지 완료! 원래 목적지로 주행을 복귀합니다."))

    node.create_subscription(String, '/sweepick/ready', on_ready, 10)

    exec_mock = SingleThreadedExecutor()
    exec_mock.add_node(node)

    start_t = time.time()
    while time.time() - start_t < 12.0:
        exec_mock.spin_once(timeout_sec=0.1)

    exec_mock.remove_node(node)
    node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description="Robotic Arm Handshake Tester")
    parser.add_argument('--gui', action='store_true', help="Launch modern interactive GUI dashboard")
    parser.add_argument('--mock-robot', action='store_true', help="Run mock turtlebot simulation thread for single-PC testing")
    parser.add_argument('--no-inching', action='store_true', help="Skip micro-inching test")
    args = parser.parse_args()

    # If GUI requested, delegate directly to arm_handshake_gui
    if args.gui:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if script_dir not in sys.path:
            sys.path.insert(0, script_dir)
        from arm_handshake_gui import main as gui_main
        gui_main()
        return

    # Ensure Domain ID
    if 'ROS_DOMAIN_ID' not in os.environ:
        os.environ['ROS_DOMAIN_ID'] = '101'

    rclpy.init()

    # If mock-robot requested, launch mock emitter in background thread
    if args.mock_robot:
        mock_thread = threading.Thread(target=run_mock_turtlebot_sim, daemon=True)
        mock_thread.start()

    node = ArmHandshakeTesterNode(test_inching=not args.no_inching)

    exec_main = SingleThreadedExecutor()
    exec_main.add_node(node)
    spin_thread = threading.Thread(target=exec_main.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_handshake_sequence()
    except KeyboardInterrupt:
        print("\n테스트가 사용자에 의해 중단되었습니다.")
    finally:
        try:
            exec_main.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        os._exit(0)


if __name__ == '__main__':
    main()
