#!/usr/bin/env python3
"""
Robotic Arm & TurtleBot Handshake Interactive GUI Tester
=========================================================
실제 물체 인식/접근 없이도 클릭 한 번으로:
  1. 가상 '접근 완료(APPROACH_DONE)' 신호 송신 (터틀봇 상태 전환)
  2. 로봇팔 자세 틀기(turn_left, turn_right) 및 추가 접근(forward, backward) 전송
  3. 로봇팔 READY (베이스 고정) 및 PICK_DONE (주행 복귀) 신호 송수신
  4. 모든 토픽(/task/status, /task/command, /task/ack, /sweepick/ready, /sweepick/heartbeat) 실시간 모니터링
"""

import os
import sys
import time
import json
import queue
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


class RosHandshakeBridge(Node):
    def __init__(self, event_queue: queue.Queue):
        super().__init__('arm_handshake_gui_bridge')
        self.event_queue = event_queue

        # Publishers
        self.task_cmd_pub = self.create_publisher(String, '/task/command', 10)
        self.task_status_pub = self.create_publisher(String, '/task/status', 10)
        self.task_ack_pub = self.create_publisher(String, '/task/ack', 10)
        self.sweepick_ready_pub = self.create_publisher(String, '/sweepick/ready', 10)
        self.sweepick_hb_pub = self.create_publisher(String, '/sweepick/heartbeat', 10)

        # Subscribers
        self.create_subscription(String, '/task/status', self._on_status, 10)
        self.create_subscription(String, '/task/command', self._on_command, 10)
        self.create_subscription(String, '/task/ack', self._on_ack, 10)
        self.create_subscription(String, '/sweepick/ready', self._on_sweepick_ready, 10)
        self.create_subscription(String, '/sweepick/heartbeat', self._on_sweepick_hb, 10)
        self.create_subscription(String, '/task/queue_status', self._on_queue_status, 10)

        # Heartbeat generator & Mock TurtleBot mode (Allows full testing when TurtleBot is powered off)
        self.heartbeat_enabled = False
        self.hb_count = 0
        self.hb_timer = self.create_timer(0.5, self._hb_tick)  # 2 Hz
        self.mock_turtlebot_mode = True  # Automatically responds to arm commands when TurtleBot is OFF

    def _hb_tick(self):
        if self.heartbeat_enabled:
            self.hb_count += 1
            payload = json.dumps({"source": "arm_gui_tester", "seq": self.hb_count, "stamp": time.time()})
            self.sweepick_hb_pub.publish(String(data=payload))
            self.event_queue.put(("HEARTBEAT_SENT", payload))

    def _on_status(self, msg: String):
        self.event_queue.put(("STATUS", msg.data))

    def _on_command(self, msg: String):
        self.event_queue.put(("COMMAND", msg.data))
        # If TurtleBot hardware is OFF, GUI acts as TurtleBot and automatically replies with ACK!
        if self.mock_turtlebot_mode:
            try:
                d = json.loads(msg.data)
                cmd_id = d.get("id", f"cmd_{int(time.time()*1000)%100000}")
                action = str(d.get("action", "")).upper()
                if any(k in d for k in ("forward", "backward", "turn_left", "turn_right", "left", "right")) or action == "NAVIGATE_RELATIVE":
                    # Simulated 0.3s wheel rotation delay, then emit ACK
                    threading.Timer(0.3, lambda: self._emit_mock_ack(cmd_id)).start()
            except Exception:
                pass

    def _emit_mock_ack(self, cmd_id: str):
        ack_payload = json.dumps({"id": cmd_id, "state": "done"})
        self.task_ack_pub.publish(String(data=ack_payload))
        self.task_status_pub.publish(String(data=f"INFO: [모의 터틀봇] 자세/거리 이동 완료: id={cmd_id}"))

    def _on_ack(self, msg: String):
        self.event_queue.put(("ACK", msg.data))

    def _on_sweepick_ready(self, msg: String):
        self.event_queue.put(("SWEEPICK_READY", msg.data))
        if self.mock_turtlebot_mode:
            try:
                d = json.loads(msg.data)
                status = str(d.get("status", "")).upper()
                if status in ("PICK_DONE", "SUCCESS"):
                    threading.Timer(0.3, lambda: self.task_status_pub.publish(
                        String(data="INFO: 🤖 [모의 터틀봇] 로봇팔 집기 완료 수신! 원래 목적지로 주행을 복귀합니다.")
                    )).start()
            except Exception:
                pass

    def _on_sweepick_hb(self, msg: String):
        self.event_queue.put(("SWEEPICK_HB", msg.data))

    def _on_queue_status(self, msg: String):
        self.event_queue.put(("QUEUE_STATUS", msg.data))

    # Send Actions
    def trigger_mock_approach_done(self):
        """Sends TRIGGER_APPROACH_DONE to TurtleBot task_manager and broadcasts APPROACH_DONE."""
        # 1. Instruct TurtleBot task_manager to transition to WAITING_ARM_PICK immediately
        cmd_payload = json.dumps({"action": "TRIGGER_APPROACH_DONE", "id": f"gui_app_{int(time.time())}"})
        self.task_cmd_pub.publish(String(data=cmd_payload))

        # 2. Also directly publish status and ack so external nodes (192.168.0.35) detect it immediately
        status_text = "🎉 [접근 완료] APPROACH_DONE: (GUI 가상) 물체 전방 25.0cm 정지 완료! (로봇팔 작업 준비 완료)"
        self.task_status_pub.publish(String(data=status_text))
        self.task_ack_pub.publish(String(data=json.dumps({"id": "approach", "state": "done", "status": "APPROACH_DONE"})))

    def send_relative_inching(self, forward_m: float = 0.0, turn_deg: float = 0.0):
        cmd_id = f"gui_inch_{int(time.time() * 1000) % 100000}"
        payload = {"id": cmd_id}
        if abs(forward_m) > 0.0001:
            if forward_m >= 0:
                payload["forward"] = round(forward_m, 3)
            else:
                payload["backward"] = round(abs(forward_m), 3)
        if abs(turn_deg) > 0.0001:
            if turn_deg >= 0:
                payload["turn_left"] = round(turn_deg, 1)
            else:
                payload["turn_right"] = round(abs(turn_deg), 1)

        payload_str = json.dumps(payload)
        self.task_cmd_pub.publish(String(data=payload_str))
        return cmd_id, payload_str

    def send_arm_ready(self):
        payload = json.dumps({"status": "READY", "arm": "moving", "object": "locked"})
        self.sweepick_ready_pub.publish(String(data=payload))
        return payload

    def send_arm_pick_done(self):
        payload = json.dumps({"status": "PICK_DONE", "arm": "home", "result": "success"})
        self.sweepick_ready_pub.publish(String(data=payload))
        return payload

    def send_stop(self):
        payload = json.dumps({"action": "STOP", "reason": "GUI Emergency Stop"})
        self.task_cmd_pub.publish(String(data=payload))
        return payload


class HandshakeDashboard(tk.Tk):
    def __init__(self, bridge_node: RosHandshakeBridge, event_queue: queue.Queue):
        super().__init__()
        self.bridge = bridge_node
        self.event_queue = event_queue

        self.title("🦾 TurtleBot & Robotic Arm Handshake Tester (Domain: 101)")
        self.geometry("980x780")
        self.minsize(880, 680)

        # Dark Slate Theme Colors
        self.c_bg = "#1e1e2e"
        self.c_card = "#252538"
        self.c_card_border = "#3e3e5e"
        self.c_text = "#cdd6f4"
        self.c_text_dim = "#a6adc8"
        self.c_accent = "#89b4fa"
        self.c_green = "#a6e3a1"
        self.c_red = "#f38ba8"
        self.c_yellow = "#f9e2af"
        self.c_purple = "#cba6f7"

        self.configure(bg=self.c_bg)

        # State tracking
        self.last_hb_time = 0.0
        self.is_approach_done = False
        self.is_arm_ready = False
        self.is_pick_done = False

        self._build_ui()
        self._check_event_queue()

    def _build_ui(self):
        # 1. Header Frame
        header = tk.Frame(self, bg=self.c_card, highlightbackground=self.c_card_border, highlightthickness=1)
        header.pack(fill="x", padx=14, pady=(12, 6))

        title_lbl = tk.Label(
            header,
            text="🦾 로봇팔(192.168.0.35) ⟷ 터틀봇 통신 & 미세제어 통합 테스터",
            font=("Pretendard", 15, "bold"),
            fg=self.c_accent,
            bg=self.c_card
        )
        title_lbl.pack(side="left", padx=16, pady=10)

        info_lbl = tk.Label(
            header,
            text=f"ROS_DOMAIN_ID: 101  |  RMW: FastRTPS  |  Subnet UDP Discovery",
            font=("Pretendard", 10, "bold"),
            fg=self.c_yellow,
            bg=self.c_card
        )
        info_lbl.pack(side="right", padx=16, pady=10)

        # 2. Live Status Badges Card
        status_card = tk.Frame(self, bg=self.c_card, highlightbackground=self.c_card_border, highlightthickness=1)
        status_card.pack(fill="x", padx=14, pady=6)

        badges_grid = tk.Frame(status_card, bg=self.c_card)
        badges_grid.pack(fill="x", padx=12, pady=10)

        # Badge 1: Approach Done
        self.badge_approach = self._create_badge(badges_grid, "1. 물체 접근 완료", "대기 중 (미수신)", self.c_red)
        self.badge_approach.grid(row=0, column=0, padx=8, pady=4, sticky="nsew")

        # Badge 2: Arm Heartbeat
        self.badge_hb = self._create_badge(badges_grid, "2. 로봇팔 생존(Heartbeat)", "끊김 (0 Hz)", self.c_red)
        self.badge_hb.grid(row=0, column=1, padx=8, pady=4, sticky="nsew")

        # Badge 3: Arm Ready (Base Locked)
        self.badge_ready = self._create_badge(badges_grid, "3. 로봇팔 집기준비(READY)", "대기 중", self.c_text_dim)
        self.badge_ready.grid(row=0, column=2, padx=8, pady=4, sticky="nsew")

        # Badge 4: Pick Done (Resumed)
        self.badge_done = self._create_badge(badges_grid, "4. 집기 완료(PICK_DONE)", "대기 중", self.c_text_dim)
        self.badge_done.grid(row=0, column=3, padx=8, pady=4, sticky="nsew")

        for col in range(4):
            badges_grid.grid_columnconfigure(col, weight=1)

        # 3. Middle Area (Controls)
        ctrl_frame = tk.Frame(self, bg=self.c_bg)
        ctrl_frame.pack(fill="x", padx=14, pady=6)

        # Column Left: TurtleBot Virtual Controls
        tb_group = tk.LabelFrame(
            ctrl_frame,
            text=" 🤖 터틀봇 시뮬레이션 제어 (물체 없이 즉시 테스트) ",
            font=("Pretendard", 11, "bold"),
            fg=self.c_green,
            bg=self.c_card,
            highlightbackground=self.c_card_border,
            highlightthickness=1,
            labelanchor="nw"
        )
        tb_group.pack(side="left", fill="both", expand=True, padx=(0, 6), pady=4)

        tb_desc = tk.Label(
            tb_group,
            text="카메라로 물건을 찾지 않고, 원클릭으로 '접근 완료' 상태에\n진입하여 로봇팔의 자세/파지 신호를 수락하도록 만듭니다.",
            font=("Pretendard", 9),
            fg=self.c_text_dim,
            bg=self.c_card,
            justify="left"
        )
        tb_desc.pack(anchor="w", padx=12, pady=(8, 4))

        self.var_mock_tb = tk.BooleanVar(value=True)
        chk_mock = tk.Checkbutton(
            tb_group,
            text="🟢 모의 터틀봇 자동 응답 (터틀봇 전원 OFF 시 필수)",
            variable=self.var_mock_tb,
            font=("Pretendard", 9, "bold"),
            fg=self.c_green,
            bg=self.c_card,
            activebackground=self.c_card,
            activeforeground=self.c_green,
            selectcolor="#181825",
            command=self._on_toggle_mock_mode
        )
        chk_mock.pack(anchor="w", padx=12, pady=(0, 6))

        btn_app = tk.Button(
            tb_group,
            text="🎯 [원클릭] 가상 접근 완료(APPROACH_DONE) 신호 발송",
            font=("Pretendard", 11, "bold"),
            bg="#2e7d32",
            fg="white",
            activebackground="#388e3c",
            activeforeground="white",
            relief="flat",
            cursor="hand2",
            pady=8,
            command=self._on_click_trigger_approach
        )
        btn_app.pack(fill="x", padx=12, pady=6)

        btn_stop = tk.Button(
            tb_group,
            text="🛑 터틀봇 소프트웨어 비상 정지 (STOP)",
            font=("Pretendard", 10, "bold"),
            bg="#c62828",
            fg="white",
            activebackground="#d32f2f",
            activeforeground="white",
            relief="flat",
            cursor="hand2",
            pady=6,
            command=self._on_click_stop
        )
        btn_stop.pack(fill="x", padx=12, pady=(4, 10))

        # Column Right: Robotic Arm Controls
        arm_group = tk.LabelFrame(
            ctrl_frame,
            text=" 🦾 로봇팔 PC(192.168.0.35) 동작 시뮬레이션 ",
            font=("Pretendard", 11, "bold"),
            fg=self.c_purple,
            bg=self.c_card,
            highlightbackground=self.c_card_border,
            highlightthickness=1,
            labelanchor="nw"
        )
        arm_group.pack(side="right", fill="both", expand=True, padx=(6, 0), pady=4)

        # Heartbeat Toggle
        hb_row = tk.Frame(arm_group, bg=self.c_card)
        hb_row.pack(fill="x", padx=12, pady=(8, 4))

        self.btn_hb_toggle = tk.Button(
            hb_row,
            text="💓 로봇팔 2Hz 하트비트: 꺼짐 [클릭하여 켜기]",
            font=("Pretendard", 9, "bold"),
            bg="#3e3e5e",
            fg=self.c_yellow,
            relief="flat",
            cursor="hand2",
            pady=4,
            command=self._on_toggle_heartbeat
        )
        self.btn_hb_toggle.pack(fill="x")

        # Inching Buttons (Turn & Forward)
        inch_lbl = tk.Label(
            arm_group,
            text="▼ 터틀봇 자세 틀기 & 추가 접근 명령 전송 (최대 10cm·30° 안전제어):",
            font=("Pretendard", 9, "bold"),
            fg=self.c_accent,
            bg=self.c_card
        )
        inch_lbl.pack(anchor="w", padx=12, pady=(6, 4))

        grid_inch = tk.Frame(arm_group, bg=self.c_card)
        grid_inch.pack(fill="x", padx=12, pady=4)

        btn_fwd = tk.Button(
            grid_inch, text="⬆️ 전진 3cm", font=("Pretendard", 9, "bold"),
            bg="#33334d", fg=self.c_text, relief="flat", cursor="hand2", pady=4,
            command=lambda: self._on_click_inch(fwd=0.03, turn=0.0)
        )
        btn_fwd.grid(row=0, column=1, padx=3, pady=2, sticky="nsew")

        btn_bwd = tk.Button(
            grid_inch, text="⬇️ 후진 3cm", font=("Pretendard", 9, "bold"),
            bg="#33334d", fg=self.c_text, relief="flat", cursor="hand2", pady=4,
            command=lambda: self._on_click_inch(fwd=-0.03, turn=0.0)
        )
        btn_bwd.grid(row=1, column=1, padx=3, pady=2, sticky="nsew")

        btn_left = tk.Button(
            grid_inch, text="⟲ 좌회전 5°", font=("Pretendard", 9, "bold"),
            bg="#33334d", fg=self.c_text, relief="flat", cursor="hand2", pady=4,
            command=lambda: self._on_click_inch(fwd=0.0, turn=5.0)
        )
        btn_left.grid(row=0, column=0, padx=3, pady=2, sticky="nsew")

        btn_right = tk.Button(
            grid_inch, text="⟳ 우회전 5°", font=("Pretendard", 9, "bold"),
            bg="#33334d", fg=self.c_text, relief="flat", cursor="hand2", pady=4,
            command=lambda: self._on_click_inch(fwd=0.0, turn=-5.0)
        )
        btn_right.grid(row=0, column=2, padx=3, pady=2, sticky="nsew")

        for col in range(3):
            grid_inch.grid_columnconfigure(col, weight=1)

        # Ready & Done Row
        handshake_row = tk.Frame(arm_group, bg=self.c_card)
        handshake_row.pack(fill="x", padx=12, pady=(6, 10))

        btn_ready = tk.Button(
            handshake_row,
            text="🔒 READY (집기 시작 ➔ 베이스 잠금)",
            font=("Pretendard", 9, "bold"),
            bg="#1565c0",
            fg="white",
            relief="flat",
            cursor="hand2",
            pady=6,
            command=self._on_click_ready
        )
        btn_ready.pack(side="left", fill="x", expand=True, padx=(0, 3))

        btn_pick_done = tk.Button(
            handshake_row,
            text="🎉 PICK_DONE (잡기 완료 ➔ 주행 복귀)",
            font=("Pretendard", 9, "bold"),
            bg="#ad1457",
            fg="white",
            relief="flat",
            cursor="hand2",
            pady=6,
            command=self._on_click_pick_done
        )
        btn_pick_done.pack(side="right", fill="x", expand=True, padx=(3, 0))

        # 4. Live Packet Monitor & Terminal Log
        log_card = tk.Frame(self, bg=self.c_card, highlightbackground=self.c_card_border, highlightthickness=1)
        log_card.pack(fill="both", expand=True, padx=14, pady=(6, 12))

        log_header = tk.Frame(log_card, bg=self.c_card)
        log_header.pack(fill="x", padx=12, pady=(8, 4))

        log_title = tk.Label(
            log_header,
            text="📡 실시간 ROS 2 토픽 송수신 패킷 모니터",
            font=("Pretendard", 11, "bold"),
            fg=self.c_accent,
            bg=self.c_card
        )
        log_title.pack(side="left")

        btn_clear = tk.Button(
            log_header,
            text="🧹 로그 지우기",
            font=("Pretendard", 9),
            bg="#3e3e5e",
            fg=self.c_text,
            relief="flat",
            cursor="hand2",
            command=self._on_clear_log
        )
        btn_clear.pack(side="right")

        self.txt_log = scrolledtext.ScrolledText(
            log_card,
            wrap="word",
            bg="#11111b",
            fg="#cdd6f4",
            insertbackground="white",
            font=("JetBrains Mono", 9),
            highlightthickness=0,
            borderwidth=0
        )
        self.txt_log.pack(fill="both", expand=True, padx=12, pady=(4, 10))

        # Text color tags
        self.txt_log.tag_config("TIME", foreground="#6c7086")
        self.txt_log.tag_config("STATUS", foreground="#89dceb")
        self.txt_log.tag_config("COMMAND", foreground="#f9e2af")
        self.txt_log.tag_config("ACK", foreground="#a6e3a1")
        self.txt_log.tag_config("READY", foreground="#cba6f7")
        self.txt_log.tag_config("SUCCESS", foreground="#a6e3a1", font=("JetBrains Mono", 9, "bold"))
        self.txt_log.tag_config("WARN", foreground="#fab387")
        self.txt_log.tag_config("SYSTEM", foreground="#f38ba8")
        self.txt_log.tag_config("QUEUE", foreground="#b4befe")

        self._log("SYSTEM", "🌟 테스터 초기화 완료. 터틀봇 노드(/task_manager) 및 로봇팔 PC(/sweepick/...)를 모니터링합니다.")

    def _create_badge(self, parent, title: str, init_val: str, init_color: str):
        box = tk.Frame(parent, bg="#181825", highlightbackground="#313244", highlightthickness=1)
        lbl_title = tk.Label(box, text=title, font=("Pretendard", 9), fg=self.c_text_dim, bg="#181825")
        lbl_title.pack(anchor="w", padx=10, pady=(6, 2))

        lbl_val = tk.Label(box, text=init_val, font=("Pretendard", 10, "bold"), fg=init_color, bg="#181825")
        lbl_val.pack(anchor="w", padx=10, pady=(0, 6))

        box.lbl_val = lbl_val
        box.set_state = lambda text, color: lbl_val.configure(text=text, fg=color)
        return box

    def _log(self, tag: str, message: str):
        now_str = time.strftime("[%H:%M:%S] ")
        self.txt_log.insert(tk.END, now_str, "TIME")
        self.txt_log.insert(tk.END, f"[{tag:<7}] ", tag)
        self.txt_log.insert(tk.END, message + "\n")
        self.txt_log.see(tk.END)

    def _on_clear_log(self):
        self.txt_log.delete("1.0", tk.END)

    # User Interactions
    def _on_toggle_mock_mode(self):
        self.bridge.mock_turtlebot_mode = self.var_mock_tb.get()
        if self.bridge.mock_turtlebot_mode:
            self._log("SYSTEM", "🤖 [모의 터틀봇 활성] 터틀봇 전원이 꺼져 있어도 로봇팔 제어 명령 수신 시 자동으로 ACK를 회신합니다.")
        else:
            self._log("SYSTEM", "🤖 [모의 터틀봇 비활성] 실제 터틀봇 하드웨어의 실행 및 응답을 기다립니다.")

    def _on_click_trigger_approach(self):
        self.bridge.trigger_mock_approach_done()
        self._log("COMMAND", "🎯 터틀봇 가상 접근완료 트리거 (TRIGGER_APPROACH_DONE) 및 /task/status 신호 발행 완료!")
        self.badge_approach.set_state("✅ 접근 완료 (신호 발행)", self.c_green)
        self.is_approach_done = True

    def _on_click_stop(self):
        payload = self.bridge.send_stop()
        self._log("COMMAND", f"🛑 비상 정지 명령 전송: {payload}")

    def _on_toggle_heartbeat(self):
        self.bridge.heartbeat_enabled = not self.bridge.heartbeat_enabled
        if self.bridge.heartbeat_enabled:
            self.btn_hb_toggle.configure(text="💓 로봇팔 2Hz 하트비트: 동작 중 [클릭하여 끄기]", bg="#2e7d32", fg="white")
            self._log("READY", "💓 로봇팔 생존 신호(/sweepick/heartbeat) 2Hz 발행 시작")
        else:
            self.btn_hb_toggle.configure(text="💓 로봇팔 2Hz 하트비트: 꺼짐 [클릭하여 켜기]", bg="#3e3e5e", fg=self.c_yellow)
            self._log("READY", "🛑 로봇팔 생존 신호(/sweepick/heartbeat) 발행 중단")

    def _on_click_inch(self, fwd: float = 0.0, turn: float = 0.0):
        cmd_id, payload = self.bridge.send_relative_inching(fwd, turn)
        desc = ""
        if abs(fwd) > 0.0001:
            desc += f"{'전진' if fwd > 0 else '후진'} {abs(fwd)*100:.0f}cm "
        if abs(turn) > 0.0001:
            desc += f"{'좌회전' if turn > 0 else '우회전'} {abs(turn):.1f}° "
        self._log("COMMAND", f"📤 미세조정 전송 [{desc.strip()}] -> {payload}")

    def _on_click_ready(self):
        payload = self.bridge.send_arm_ready()
        self._log("READY", f"📤 집기 준비(READY) 전송 ➔ 베이스 정지 잠금 유도: {payload}")
        self.badge_ready.set_state("🦾 집기 중 (베이스 잠김)", self.c_accent)
        self.is_arm_ready = True

    def _on_click_pick_done(self):
        payload = self.bridge.send_arm_pick_done()
        self._log("READY", f"📤 집기 완료(PICK_DONE) 전송 ➔ 주행 복귀 유도: {payload}")
        self.badge_done.set_state("🎉 완료 (복귀 신호 송신)", self.c_green)
        self.is_pick_done = True

    # Background ROS 2 Event Queue Handler
    def _check_event_queue(self):
        try:
            while not self.event_queue.empty():
                evt_type, data = self.event_queue.get_nowait()

                if evt_type == "STATUS":
                    self._log("STATUS", f"📥 /task/status: {data}")
                    if "접근 완료" in data or "APPROACH_DONE" in data:
                        self.badge_approach.set_state("✅ 접근 완료 감지", self.c_green)
                        self.is_approach_done = True
                    elif "로봇팔 신호 수신" in data or "주행 복귀" in data or "주행을 재개" in data:
                        self.badge_done.set_state("🚀 주행 재개 확인!", self.c_green)

                elif evt_type == "COMMAND":
                    self._log("COMMAND", f"📥 /task/command 수신: {data}")

                elif evt_type == "ACK":
                    self._log("ACK", f"📥 /task/ack 수신: {data}")
                    try:
                        d = json.loads(data)
                        if d.get("state") == "done":
                            self._log("SUCCESS", f"  └ 🎯 명령 완료 ACK 확인 (id: {d.get('id')})")
                    except Exception:
                        pass

                elif evt_type == "SWEEPICK_READY":
                    self._log("READY", f"📥 /sweepick/ready: {data}")
                    try:
                        d = json.loads(data)
                        st = str(d.get("status", "")).upper()
                        if st == "READY":
                            self.badge_ready.set_state("🦾 집기 중 (READY)", self.c_accent)
                        elif st in ("PICK_DONE", "SUCCESS"):
                            self.badge_done.set_state("🎉 PICK_DONE 수신됨", self.c_green)
                        elif st in ("PICK_NOT_DONE", "BUS_LOST", "ERROR", "FAILED"):
                            self.badge_done.set_state(f"🚨 {st} (베이스 잠금)", self.c_red)
                            self._log("WARN", f"⚠️ 로봇팔 집기 중단({st}) 수신! 베이스 잠금 유지 (출발 안 함)")
                    except Exception:
                        pass

                elif evt_type == "QUEUE_STATUS":
                    self._log("QUEUE", f"📋 /task/queue_status: {data}")

                elif evt_type in ("SWEEPICK_HB", "HEARTBEAT_SENT"):
                    self.last_hb_time = time.time()
                    self.badge_hb.set_state("💓 2Hz 수신 중", self.c_green)

        except Exception as e:
            print(f"Error handling event: {e}")

        # Check Heartbeat timeout (1.5s)
        if time.time() - self.last_hb_time > 1.5 and self.last_hb_time > 0:
            self.badge_hb.set_state("⚠️ 하트비트 끊김", self.c_yellow)

        self.after(50, self._check_event_queue)


def main():
    if 'ROS_DOMAIN_ID' not in os.environ:
        os.environ['ROS_DOMAIN_ID'] = '101'

    rclpy.init()
    event_queue = queue.Queue()
    bridge_node = RosHandshakeBridge(event_queue)

    # Run ROS 2 Executor in background daemon thread
    executor = SingleThreadedExecutor()
    executor.add_node(bridge_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Run Tkinter GUI
    app = HandshakeDashboard(bridge_node, event_queue)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.remove_node(bridge_node)
            bridge_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        os._exit(0)


if __name__ == '__main__':
    main()
