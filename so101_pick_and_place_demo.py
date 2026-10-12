import asyncio
import omni
import omni.ui as ui
import numpy as np
from pxr import UsdGeom, Gf, UsdShade, Sdf, UsdPhysics
from isaacsim.core.experimental.prims import Articulation

class SO101PickAndPlaceDemo:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.robot_path = "/so101_new_calib"
        self.cube_path = "/World/TargetCube"
        self.robot = Articulation(self.robot_path)
        
        # 1. 모터 파워 최적화 (단단한 지지력)
        stiffness = [900.0, 1100.0, 900.0, 700.0, 500.0, 500.0]
        damping = [80.0, 100.0, 80.0, 60.0, 40.0, 40.0]
        self.robot.set_dof_gains(stiffnesses=stiffness, dampings=damping)
        
        # 2. 전용 상자 준비 (한 변 2.5cm, 가벼운 20g)
        self._setup_target_cube()
        
        self.is_running = False
        self._build_ui()
        print(">> [READY] SO-101 Pick & Place Demo 준비 완료!")

    def _setup_target_cube(self):
        """그리퍼에 꼭 맞는 2.5cm 물리 상자 자동 배치"""
        cube_prim = self.stage.GetPrimAtPath(self.cube_path)
        if not cube_prim.IsValid():
            cube = UsdGeom.Cube.Define(self.stage, self.cube_path)
            cube.GetSizeAttr().Set(0.025)  # 2.5cm 정육면체
            xform = UsdGeom.Xformable(cube)
            xform.AddTranslateOp().Set(Gf.Vec3d(0.17, 0.0, 0.015))
            
            # 물리 속성 (충돌체 + 강체 + 가벼운 무게 20g)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
            mass_api.CreateMassAttr(0.02)
            
            # 산뜻한 오렌지색 PBR 재질
            vmat = UsdShade.Material.Define(self.stage, "/World/TargetCube/VisualMat")
            pbr = UsdShade.Shader.Define(self.stage, "/World/TargetCube/VisualMat/Pbr")
            pbr.CreateIdAttr("UsdPreviewSurface")
            pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.45, 0.05))
            pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
            vmat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(vmat)

    def _reset_positions(self):
        """로봇과 상자를 초기 위치로 즉시 리셋"""
        cube_prim = self.stage.GetPrimAtPath(self.cube_path)
        if cube_prim.IsValid():
            xformable = UsdGeom.Xformable(cube_prim)
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(0.17, 0.0, 0.015))
        home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.robot.set_dof_position_targets([home_pose])
        self.status_label.text = "Status: 초기화 완료 (Ready)"
        print(">> [RESET] 초기화 완료!")

    async def _interpolate_motion(self, start_q, target_q, duration=1.0, steps=40):
        """실제 로봇처럼 부드러운 곡선으로 관절 보간 이동"""
        for s in range(1, steps + 1):
            alpha = s / steps
            smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha)
            interpolated = (1.0 - smooth_alpha) * np.array(start_q) + smooth_alpha * np.array(target_q)
            self.robot.set_dof_position_targets([interpolated.tolist()])
            await asyncio.sleep(duration / steps)

    async def _run_demo_sequence(self):
        if self.is_running:
            return
        self.is_running = True
        print("\n>> 🚀 Pick & Place 시퀀스 시작!")

        # 6개 관절 각도 정의 [pan, lift, elbow, wrist_flex, wrist_roll, gripper]
        p_home        = [0.0,   0.0,   0.0,   0.0,  0.0, 0.0]
        p_ready       = [0.0,  -0.3,   0.5,   0.4,  0.0, 0.0] # 상자 위 접근
        p_grasp_open  = [0.0,  -0.62,  0.84,  0.68, 0.0, 0.0] # 집기 위치로 하강
        p_grasp_close = [0.0,  -0.62,  0.84,  0.68, 0.0, 0.7] # 꽉 집기 (Grip)
        p_lift        = [0.0,  -0.28,  0.42,  0.28, 0.0, 0.7] # 위로 들어올리기
        p_transfer    = [1.05, -0.28,  0.42,  0.28, 0.0, 0.7] # 옆으로 60도 회전
        p_place_down  = [1.05, -0.62,  0.84,  0.68, 0.0, 0.7] # 바닥으로 하강
        p_place_open  = [1.05, -0.62,  0.84,  0.68, 0.0, 0.0] # 놓기 (Release)
        p_retract     = [1.05, -0.28,  0.42,  0.28, 0.0, 0.0] # 상승 복귀
        
        try:
            # 1. 접근 및 하강
            self.status_label.text = "1/7 상자 위로 이동..."
            await self._interpolate_motion(p_home, p_ready, duration=1.2)
            await asyncio.sleep(0.2)
            
            self.status_label.text = "2/7 상자 위치로 하강..."
            await self._interpolate_motion(p_ready, p_grasp_open, duration=1.0)
            await asyncio.sleep(0.3)
            
            # 2. 파지 (집기)
            self.status_label.text = "3/7 상자 파지 (Grip!)"
            await self._interpolate_motion(p_grasp_open, p_grasp_close, duration=0.8, steps=25)
            await asyncio.sleep(0.5)
            
            # 3. 들어올리기
            self.status_label.text = "4/7 상자 들어올리기 (Lift)..."
            await self._interpolate_motion(p_grasp_close, p_lift, duration=1.2)
            await asyncio.sleep(0.3)
            
            # 4. 옆으로 회전 이동
            self.status_label.text = "5/7 목표 위치로 선회 이동..."
            await self._interpolate_motion(p_lift, p_transfer, duration=1.8)
            await asyncio.sleep(0.3)
            
            # 5. 내려놓기
            self.status_label.text = "6/7 바닥에 안착 및 놓기 (Release)..."
            await self._interpolate_motion(p_transfer, p_place_down, duration=1.0)
            await asyncio.sleep(0.2)
            await self._interpolate_motion(p_place_down, p_place_open, duration=0.6, steps=20)
            await asyncio.sleep(0.4)
            
            # 6. 홈 복귀
            self.status_label.text = "7/7 초기 자세 복귀 (Home)..."
            await self._interpolate_motion(p_place_open, p_retract, duration=1.0)
            await self._interpolate_motion(p_retract, p_home, duration=1.5)
            
            self.status_label.text = "Status: 데모 완료! 🎉"
            print(">> 🎉 Pick & Place 데모 성공 완료!\n")
        finally:
            self.is_running = False

    def _build_ui(self):
        self.window = ui.Window("SO-101 Pick & Place Demo", width=340, height=210)
        with self.window.frame:
            with ui.VStack(spacing=10):
                ui.Label("SO-101 Pick & Place Demo", height=24, style={"color": 0xFF00D2FF, "font_size": 16})
                self.status_label = ui.Label("Status: 대기 중 (Ready)", height=22, style={"color": 0xFFE0E0E0})
                
                ui.Spacer(height=6)
                with ui.HStack(height=36):
                    ui.Button("▶ 데모 시작 (Start)", clicked_fn=lambda: asyncio.ensure_future(self._run_demo_sequence()))
                    ui.Button("🔄 위치 리셋 (Reset)", clicked_fn=self._reset_positions)
                    
                ui.Spacer(height=4)
                ui.Label("Play(▶) 상태에서 [데모 시작]을 누르세요.", height=20, style={"color": 0xFF999999})

pnp_demo = SO101PickAndPlaceDemo()
