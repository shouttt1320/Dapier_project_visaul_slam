import omni
import omni.ui as ui
import numpy as np
from scipy.optimize import minimize
from pxr import UsdGeom, Gf, UsdShade, Sdf, Usd
from isaacsim.core.experimental.prims import Articulation

# ==============================================================================
# SO-101 자체 내장 역기구학 (IK) 엔진 (외부 라이브러리 설치 불필요, 100% 자체 동작)
# ==============================================================================
def _rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def _rpy_matrix(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    R = np.zeros((4, 4))
    R[0, 0] = cy*cp
    R[0, 1] = cy*sp*sr - sy*cr
    R[0, 2] = cy*sp*cr + sy*sr
    R[1, 0] = sy*cp
    R[1, 1] = sy*sp*sr + cy*cr
    R[1, 2] = sy*sp*cr - cy*sr
    R[2, 0] = -sp
    R[2, 1] = cp*sr
    R[2, 2] = cp*cr
    R[3, 3] = 1.0
    return R

def _trans(xyz):
    T = np.eye(4)
    T[:3, 3] = xyz
    return T

# URDF 기반 정밀 순기구학 (FK)
def so101_forward_kinematics(q):
    q0, q1, q2, q3, q4 = q
    T = _trans([0.0388353, 0.0, 0.0624]) @ _rpy_matrix(3.14159, 0, -3.14159) @ _rot_z(q0)
    T = T @ _trans([-0.0303992, -0.0182778, -0.0542]) @ _rpy_matrix(-1.5708, -1.5708, 0) @ _rot_z(q1)
    T = T @ _trans([-0.11257, -0.028, 0.0]) @ _rpy_matrix(0, 0, 1.5708) @ _rot_z(q2)
    T = T @ _trans([-0.1349, 0.0052, 0.0]) @ _rpy_matrix(0, 0, -1.5708) @ _rot_z(q3)
    T = T @ _trans([0.0, -0.0611, 0.0181]) @ _rpy_matrix(1.5708, 0.04868, 3.14159) @ _rot_z(q4)
    T = T @ _trans([-0.0079, -0.000218, -0.098127]) @ _rpy_matrix(0, 3.14159, 0)
    return T[:3, 3]

# 관절 가동 범위 (라디안)
JOINT_BOUNDS = [
    (-np.deg2rad(110), np.deg2rad(110)),
    (-np.deg2rad(70), np.deg2rad(90)),
    (-np.deg2rad(90), np.deg2rad(90)),
    (-np.deg2rad(90), np.deg2rad(90)),
    (-np.deg2rad(110), np.deg2rad(110)),
]

def solve_so101_ik(target_pos, q_init=None):
    if q_init is None:
        q_init = [0.0, -0.5, 0.8, 0.5, 0.0]
    target = np.array(target_pos)
    
    # 팬 각도는 방위각으로 초기값 지정 (수렴 가속)
    pan_guess = np.arctan2(target[1], target[0])
    q_init[0] = np.clip(pan_guess, JOINT_BOUNDS[0][0], JOINT_BOUNDS[0][1])
    
    def loss(q):
        tip = so101_forward_kinematics(q)
        pos_err = np.sum((tip - target)**2)
        # 이전 자세 유지 가중치 (급격한 관절 회전 방지)
        reg = 0.01 * np.sum((q - q_init)**2)
        return pos_err + reg

    res = minimize(loss, q_init, method='SLSQP', bounds=JOINT_BOUNDS, tol=1e-4)
    return res.x


# ==============================================================================
# 실시간 마우스 기즈모 추종 및 GUI 컨트롤러
# ==============================================================================
class SO101MouseIKController:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.robot = Articulation("/so101_new_calib")
        
        # 튼튼한 모터 게인 (처짐 방지)
        stiffness = [800.0, 1000.0, 800.0, 600.0, 400.0, 400.0]
        damping = [80.0, 100.0, 80.0, 60.0, 40.0, 40.0]
        self.robot.set_dof_gains(stiffnesses=stiffness, dampings=damping)
        
        self.target_prim_path = "/World/IK_Target"
        self._setup_target_marker()
        
        self.gripper_open = True
        self.tracking_enabled = True
        self.current_q = [0.0, -0.5, 0.8, 0.5, 0.0]
        self.last_full_targets = [0.0] * 6
        
        # 실시간 렌더링 루프 추종 구독
        self.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="so101_ik_pure_update"
        )
        self._build_ui()
        print(">> [성공] SO-101 마우스 실시간 IK 컨트롤러가 시작되었습니다!")

    def _setup_target_marker(self):
        """3D 뷰포트에서 마우스로 잡고 움직일 형광 녹색 타겟 구체 생성"""
        prim = self.stage.GetPrimAtPath(self.target_prim_path)
        if not prim.IsValid():
            sphere = UsdGeom.Sphere.Define(self.stage, self.target_prim_path)
            sphere.GetRadiusAttr().Set(0.015)  # 1.5cm 크기
            xform = UsdGeom.Xformable(sphere)
            xform.AddTranslateOp().Set(Gf.Vec3d(0.18, 0.0, 0.08))
            
            # 형광 녹색 반투명 재질
            mat_path = "/World/IK_Target/Material"
            material = UsdShade.Material.Define(self.stage, mat_path)
            pbr = UsdShade.Shader.Define(self.stage, mat_path + "/Shader")
            pbr.CreateIdAttr("UsdPreviewSurface")
            pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.1, 1.0, 0.4))
            pbr.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.65)
            material.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI(sphere).Bind(material)
        self.target_prim = self.stage.GetPrimAtPath(self.target_prim_path)

    def _get_target_pos(self):
        xformable = UsdGeom.Xformable(self.target_prim)
        world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = world_transform.ExtractTranslation()
        return [float(pos[0]), float(pos[1]), float(pos[2])]

    def _on_update(self, event):
        if not self.tracking_enabled or not self.target_prim.IsValid():
            return
        try:
            target_pos = self._get_target_pos()
            # 순수 SciPy IK 계산
            q_solved = solve_so101_ik(target_pos, q_init=list(self.current_q))
            self.current_q = list(q_solved)
            
            grip_val = 0.0 if self.gripper_open else 0.7
            self.last_full_targets = list(self.current_q) + [grip_val]
            
            # 로봇팔 관절 실시간 전달
            self.robot.set_dof_position_targets([self.last_full_targets])
        except Exception:
            pass

    def _build_ui(self):
        self.window = ui.Window("SO-101 실시간 마우스 IK", width=340, height=230)
        with self.window.frame:
            with ui.VStack(spacing=8):
                ui.Label("🎯 마우스 드래그 추종 제어기", height=24, style={"color": 0xFF00FF88, "font_size": 15})
                ui.Label("뷰포트에서 녹색 구체(/World/IK_Target)를\n선택하고 W키로 잡고 끌면 로봇이 따라옵니다!", height=36)
                
                with ui.HStack(height=32):
                    ui.Button("🖐 그리퍼 열기", clicked_fn=self._open_grip)
                    ui.Button("✊ 그리퍼 닫기", clicked_fn=self._close_grip)
                    
                ui.Spacer(height=4)
                with ui.HStack(height=32):
                    ui.Button("📋 현재 위치 / 각도 출력", clicked_fn=self._print_pose)
                    
                with ui.HStack(height=26):
                    self.track_btn = ui.Button("⏸ 추종 일시중지", clicked_fn=self._toggle_tracking)

    def _open_grip(self):
        self.gripper_open = True
        print("[IK] 그리퍼 열림 (Open)")

    def _close_grip(self):
        self.gripper_open = False
        print("[IK] 그리퍼 닫힘 (Close)")

    def _toggle_tracking(self):
        self.tracking_enabled = not self.tracking_enabled
        self.track_btn.text = "▶ 추종 다시 시작" if not self.tracking_enabled else "⏸ 추종 일시중지"

    def _print_pose(self):
        pos = self._get_target_pos()
        deg = [round(float(np.rad2deg(a)), 1) for a in self.last_full_targets]
        print("\n" + "="*50)
        print(f"🎯 타겟 3D 위치 [X, Y, Z]: {[round(p, 4) for p in pos]}")
        print(f"🤖 계산된 관절 각도 (Degree): {deg}")
        print(f"💾 파이썬 코드용 Radian: {[round(float(a), 3) for a in self.last_full_targets]}")
        print("="*50 + "\n")

# 실행
so101_ik = SO101MouseIKController()
