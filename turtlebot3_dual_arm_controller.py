import omni
import omni.ui as ui
import numpy as np
from isaacsim.core.experimental.prims import Articulation

# ==============================================================================
# TurtleBot3 Waffle Pi + Dual SO-101 양팔로봇 통합 제어기
# ==============================================================================
class DualArmMobileRobotController:
    def __init__(self, prim_path="/World/turtlebot3_dual_so101"):
        self.prim_path = prim_path
        self.robot = Articulation(prim_path)
        
        # 14개 관절 이름 매핑
        # [0]: wheel_left, [1]: wheel_right
        # [2:8]: left_arm (pan, lift, elbow, wrist_flex, wrist_roll, gripper)
        # [8:14]: right_arm (pan, lift, elbow, wrist_flex, wrist_roll, gripper)
        self._configure_gains()
        
        self.current_targets = [0.0] * 14
        self._build_ui()
        print(">> [READY] 이동형 양팔로봇(TurtleBot3 + Dual SO-101) 제어기 준비 완료!")

    def _configure_gains(self):
        """14개 관절별 최적 물리 게인(물리엔진 튜닝) 주입"""
        # 휠 2개는 속도 제어/주행용, 로봇팔 12개는 처짐 방지용 고토크 위치 제어
        stiffness = [
            0.0, 0.0,                                    # Wheels (Left, Right)
            800.0, 1000.0, 800.0, 600.0, 400.0, 400.0,   # Left Arm (6-DOF)
            800.0, 1000.0, 800.0, 600.0, 400.0, 400.0    # Right Arm (6-DOF)
        ]
        damping = [
            100.0, 100.0,                                # Wheels
            80.0, 100.0, 80.0, 60.0, 40.0, 40.0,         # Left Arm
            80.0, 100.0, 80.0, 60.0, 40.0, 40.0          # Right Arm
        ]
        try:
            self.robot.set_dof_gains(stiffnesses=stiffness, dampings=damping)
            print(">> [GAINS] 14개 조인트 파워 설정 완료!")
        except Exception as e:
            print(f">> [WARN] 게인 설정 알림: {e}")

    def _build_ui(self):
        self.window = ui.Window("Dual-Arm Mobile Robot Controller", width=420, height=380)
        with self.window.frame:
            with ui.VStack(spacing=8):
                ui.Label("🤖 TurtleBot3 + Dual SO-101 통합 제어", height=24, style={"color": 0xFF00D2FF, "font_size": 16})
                ui.Label("모바일 주행과 양팔 협동 모션을 동시에 제어합니다.", height=20, style={"color": 0xFFAAAAAA})
                ui.Spacer(height=4)
                
                # 1. 주행 제어 패널
                ui.Label("🚗 모바일 베이스 주행 (Wheels)", height=20, style={"color": 0xFF22FF88, "font_size": 13})
                with ui.HStack(height=30):
                    ui.Button("▲ 전진 (Forward)", clicked_fn=self._drive_forward)
                    ui.Button("▼ 후진 (Backward)", clicked_fn=self._drive_backward)
                with ui.HStack(height=30):
                    ui.Button("◀ 좌회전 (Turn Left)", clicked_fn=self._turn_left)
                    ui.Button("▶ 우회전 (Turn Right)", clicked_fn=self._turn_right)
                with ui.HStack(height=26):
                    ui.Button("■ 주행 정지 (Stop)", clicked_fn=self._stop_wheels)
                
                ui.Spacer(height=8)
                
                # 2. 양팔 협동 모션 프리셋
                ui.Label("🦾 양팔 협동 모션 (Bimanual Motions)", height=20, style={"color": 0xFFFFB300, "font_size": 13})
                with ui.HStack(height=32):
                    ui.Button("🏠 홈 자세 (Home)", clicked_fn=self._pose_home)
                    ui.Button("📦 큰 상자 맞잡기 (Hug/Grasp)", clicked_fn=self._pose_box_hug)
                with ui.HStack(height=32):
                    ui.Button("👋 한손 인사 (Wave)", clicked_fn=self._pose_wave)
                    ui.Button("⚡ 작업 준비 자세 (Ready)", clicked_fn=self._pose_ready)

    # --- 주행 명령 ---
    def _drive_forward(self):
        # 양쪽 바퀴 동일 회전 (전진)
        self.robot.set_dof_velocity_targets([[4.0, 4.0] + [0.0]*12])
        print(">> [DRIVE] 전진")

    def _drive_backward(self):
        self.robot.set_dof_velocity_targets([[-4.0, -4.0] + [0.0]*12])
        print(">> [DRIVE] 후진")

    def _turn_left(self):
        # 제자리 좌회전
        self.robot.set_dof_velocity_targets([[-3.0, 3.0] + [0.0]*12])
        print(">> [DRIVE] 좌회전")

    def _turn_right(self):
        # 제자리 우회전
        self.robot.set_dof_velocity_targets([[3.0, -3.0] + [0.0]*12])
        print(">> [DRIVE] 우회전")

    def _stop_wheels(self):
        self.robot.set_dof_velocity_targets([[0.0, 0.0] + [0.0]*12])
        print(">> [DRIVE] 정지")

    # --- 양팔 자세 프리셋 ---
    def _pose_home(self):
        # 14개 관절 모두 0
        self.current_targets = [0.0] * 14
        self.robot.set_dof_position_targets([self.current_targets])
        print(">> [MOTION] 홈 자세 복귀")

    def _pose_ready(self):
        # 양팔을 살짝 들어 앞을 주시하는 준비 자세
        # Left Arm: [0.2, -0.4, 0.6, 0.4, 0.0, 0.0]
        # Right Arm: [-0.2, -0.4, 0.6, 0.4, 0.0, 0.0]
        targets = [0.0, 0.0] + [0.2, -0.4, 0.6, 0.4, 0.0, 0.0] + [-0.2, -0.4, 0.6, 0.4, 0.0, 0.0]
        self.robot.set_dof_position_targets([targets])
        print(">> [MOTION] 양팔 작업 준비 자세")

    def _pose_box_hug(self):
        # 양팔을 앞으로 뻗어 중앙의 상자를 양쪽에서 꽉 감싸쥐는 협동 자세
        # 좌우 pan을 안쪽으로 모음 (Left: 0.5 rad, Right: -0.5 rad)
        targets = [0.0, 0.0] + [0.5, -0.6, 0.8, 0.5, 0.0, 0.7] + [-0.5, -0.6, 0.8, 0.5, 0.0, 0.7]
        self.robot.set_dof_position_targets([targets])
        print(">> [MOTION] 큰 물체 양팔 포옹/맞잡기 자세 (Bimanual Hug)")

    def _pose_wave(self):
        # 오른팔을 높이 들어 흔들기
        targets = [0.0, 0.0] + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + [-0.3, -1.0, 0.2, -0.6, 0.8, 0.0]
        self.robot.set_dof_position_targets([targets])
        print(">> [MOTION] 오른팔 인사하기")

# 인스턴스 실행
dual_controller = DualArmMobileRobotController()
