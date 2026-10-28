#!/usr/bin/env python3
"""
Generate a professional, modern 16:9 widescreen PowerPoint presentation (.pptx)
for the TurtleBot3 RGB-D Visual SLAM & Semantic Docker project.
"""

import os
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

def create_deck():
    prs = Presentation()
    # 16:9 Widescreen standard dimensions
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)
    blank_layout = prs.slide_layouts[6] # completely blank layout

    # Color Palette: Modern Deep Slate & Cyan Theme
    C_BG = RGBColor(15, 23, 42)          # #0F172A (Deep Slate Navy)
    C_CARD = RGBColor(30, 41, 59)        # #1E293B (Card Dark Gray/Blue)
    C_CARD_BORDER = RGBColor(51, 65, 85) # #334155
    C_PRIMARY = RGBColor(56, 189, 248)   # #38BDF8 (Sky Blue / Cyan)
    C_SECONDARY = RGBColor(129, 140, 248)# #818CF8 (Indigo Accent)
    C_ACCENT = RGBColor(245, 158, 11)    # #F59E0B (Amber Gold)
    C_GREEN = RGBColor(52, 211, 153)     # #34D399 (Emerald Green)
    C_TEXT_LIGHT = RGBColor(248, 250, 252) # #F8FAFC (White/Off-white)
    C_TEXT_MUTED = RGBColor(148, 163, 184) # #94A3B8 (Slate Gray)
    C_CARD_HEADER = RGBColor(15, 23, 42)

    def set_slide_background(slide):
        bg_shape = slide.shapes.add_shape(
            MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height
        )
        bg_shape.fill.solid()
        bg_shape.fill.fore_color.rgb = C_BG
        bg_shape.line.color.rgb = C_BG
        # Send to back if needed, but since it's added first, it's naturally at the back

    def add_header(slide, tag_text, title_text, subtitle_text=None):
        # Category Tag Badge
        badge = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(0.45), Inches(2.2), Inches(0.35))
        badge.fill.solid()
        badge.fill.fore_color.rgb = C_CARD
        badge.line.color.rgb = C_PRIMARY
        badge.line.width = Pt(1)
        tf_b = badge.text_frame
        tf_b.word_wrap = True
        tf_b.vertical_anchor = MSO_ANCHOR.MIDDLE
        p_b = tf_b.paragraphs[0]
        p_b.alignment = PP_ALIGN.CENTER
        run_b = p_b.add_run()
        run_b.text = tag_text
        run_b.font.size = Pt(11)
        run_b.font.bold = True
        run_b.font.color.rgb = C_PRIMARY

        # Title
        tb = slide.shapes.add_textbox(Inches(0.8), Inches(0.85), Inches(11.7), Inches(0.8))
        tf = tb.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_top = tf.margin_right = tf.margin_bottom = 0
        p = tf.paragraphs[0]
        run = p.add_run()
        run.text = title_text
        run.font.size = Pt(24)
        run.font.bold = True
        run.font.color.rgb = C_TEXT_LIGHT

        if subtitle_text:
            p2 = tf.add_paragraph()
            p2.space_before = Pt(4)
            run2 = p2.add_run()
            run2.text = subtitle_text
            run2.font.size = Pt(12)
            run2.font.color.rgb = C_TEXT_MUTED

    # -------------------------------------------------------------
    # SLIDE 0: TITLE SLIDE
    # -------------------------------------------------------------
    s0 = prs.slides.add_slide(blank_layout)
    set_slide_background(s0)

    # Accent decorative line
    dec = s0.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(1.2), Inches(1.8), Inches(1.0), Inches(0.08))
    dec.fill.solid()
    dec.fill.fore_color.rgb = C_PRIMARY
    dec.line.color.rgb = C_PRIMARY

    # Title Textbox
    tb_title = s0.shapes.add_textbox(Inches(1.2), Inches(2.1), Inches(10.8), Inches(2.6))
    tf_t = tb_title.text_frame
    tf_t.word_wrap = True
    p0 = tf_t.paragraphs[0]
    r0 = p0.add_run()
    r0.text = "RGB-D 기반 3D Visual SLAM 및\n지능형 자율 모바일 로봇 시스템"
    r0.font.size = Pt(36)
    r0.font.bold = True
    r0.font.color.rgb = C_TEXT_LIGHT

    p0_sub = tf_t.add_paragraph()
    p0_sub.space_before = Pt(16)
    r0_sub = p0_sub.add_run()
    r0_sub.text = "2D LiDAR 한계 극복을 위한 Pure 3D VSLAM · Dual ArUco 정밀 도킹 · YOLOv8 바닥 물체 서보잉"
    r0_sub.font.size = Pt(16)
    r0_sub.font.color.rgb = C_PRIMARY

    # Info Card Container
    card_info = s0.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(1.2), Inches(5.1), Inches(10.9), Inches(1.5))
    card_info.fill.solid()
    card_info.fill.fore_color.rgb = C_CARD
    card_info.line.color.rgb = C_CARD_BORDER
    card_info.line.width = Pt(1.5)

    tf_ci = card_info.text_frame
    tf_ci.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf_ci.margin_left = Inches(0.4)
    p_ci1 = tf_ci.paragraphs[0]
    r_ci1 = p_ci1.add_run()
    r_ci1.text = "• 하드웨어 플랫폼 : "
    r_ci1.font.bold = True
    r_ci1.font.size = Pt(13)
    r_ci1.font.color.rgb = C_PRIMARY
    r_ci1_val = p_ci1.add_run()
    r_ci1_val.text = "TurtleBot3 Waffle Pi (Raspberry Pi 4B + OpenCR 1.0 + Orbbec Astra S RGB-D)\n"
    r_ci1_val.font.size = Pt(13)
    r_ci1_val.font.color.rgb = C_TEXT_LIGHT

    r_ci2 = p_ci1.add_run()
    r_ci2.text = "• 핵심 소프트웨어 : "
    r_ci2.font.bold = True
    r_ci2.font.size = Pt(13)
    r_ci2.font.color.rgb = C_PRIMARY
    r_ci2_val = p_ci1.add_run()
    r_ci2_val.text = "ROS 2 Jazzy, RTAB-Map 3D, Robot Localization EKF, Nav2, YOLOv8 20Hz, Qwen2.5-VL"
    r_ci2_val.font.size = Pt(13)
    r_ci2_val.font.color.rgb = C_TEXT_LIGHT

    # Speaker Notes
    s0.notes_slide.notes_text_frame.text = (
        "[발표 대본 - 표지]\n"
        "안녕하십니까. 오늘 발표를 맡은 [발표자 이름]입니다. 이번 프로젝트는 고가의 2D 라이다 센서 없이, "
        "단일 RGB-D 깊이 카메라와 IMU, 휠 오도메트리만을 활용하여 실내 3차원 공간을 복원하고 "
        "정밀 자율주행과 도킹, 그리고 바닥 물체 접근까지 통합 수행하는 순수 3D Visual SLAM 시스템 구축기입니다."
    )

    # -------------------------------------------------------------
    # SLIDE 1: WHY VISUAL SLAM (2D LiDAR vs 3D VSLAM)
    # -------------------------------------------------------------
    s1 = prs.slides.add_slide(blank_layout)
    set_slide_background(s1)
    add_header(s1, "SYSTEM MOTIVATION", "1. 왜 2D LiDAR 대신 RGB-D Visual SLAM인가?", 
               "단일 평면만 스캔하는 2D 라이다의 물리적 한계를 극복하고, 입체 공간 인식과 단일 센서 통합을 달성")

    # Table on the Left: 7.2" width
    t_left = Inches(0.8)
    t_top = Inches(1.8)
    t_width = Inches(7.4)
    t_height = Inches(4.8)

    table_shape = s1.shapes.add_table(6, 3, t_left, t_top, t_width, t_height)
    tbl = table_shape.table
    tbl.columns[0].width = Inches(1.8)
    tbl.columns[1].width = Inches(2.6)
    tbl.columns[2].width = Inches(3.0)

    headers = ["비교 항목", "기존 2D LiDAR SLAM", "구현된 3D Visual SLAM"]
    for j, h in enumerate(headers):
        cell = tbl.cell(0, j)
        cell.fill.solid()
        cell.fill.fore_color.rgb = RGBColor(30, 58, 138) if j == 2 else C_CARD
        p = cell.text_frame.paragraphs[0]
        p.alignment = PP_ALIGN.CENTER
        r = p.add_run()
        r.text = h
        r.font.bold = True
        r.font.size = Pt(12)
        r.font.color.rgb = C_PRIMARY if j == 2 else C_TEXT_LIGHT

    rows_data = [
        ("스캔 방식", "단일 지면 높이 2D 평면 스캔", "3D 입체 공간 전체(Depth) 복원"),
        ("장애물 한계", "책상 상판, 턱, 튀어나온 물체 미감지", "0.15~0.65m 필터로 책상 밑 통과 가능"),
        ("환경 정보량", "단순 기하학적 거리 점만 획득", "형상(3D 점군) + 텍스처(색상) 동시 취득"),
        ("루프 클로저", "복도 등 대칭 구조에서 오매칭 빈번", "DBoW2 시각 외형(ORB)으로 완벽 식별"),
        ("센서 확장성", "라이다 외에 비전 카메라 별도 추가", "Astra S 1대로 SLAM+도킹+AI 통합")
    ]

    for i, row in enumerate(rows_data):
        for j, val in enumerate(row):
            cell = tbl.cell(i+1, j)
            cell.fill.solid()
            cell.fill.fore_color.rgb = RGBColor(22, 33, 54) if (i % 2 == 0) else C_CARD
            p = cell.text_frame.paragraphs[0]
            if j == 0:
                p.alignment = PP_ALIGN.CENTER
            r = p.add_run()
            r.text = val
            r.font.size = Pt(11)
            if j == 2:
                r.font.bold = True
                r.font.color.rgb = C_PRIMARY
            else:
                r.font.color.rgb = C_TEXT_LIGHT

    # Right Side Feature Cards
    rc_left = Inches(8.5)
    rc_top = Inches(1.8)
    rc_width = Inches(4.0)

    # Card 1: 3D Headroom
    card1 = s1.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, rc_left, rc_top, rc_width, Inches(2.25))
    card1.fill.solid()
    card1.fill.fore_color.rgb = C_CARD
    card1.line.color.rgb = C_CARD_BORDER
    tf_c1 = card1.text_frame
    tf_c1.margin_left = tf_c1.margin_right = tf_c1.margin_top = Inches(0.25)
    p_c1_t = tf_c1.paragraphs[0]
    r_c1_t = p_c1_t.add_run()
    r_c1_t.text = "🎯 3D 높이 제어 및 책상 밑 주행"
    r_c1_t.font.bold = True
    r_c1_t.font.size = Pt(13)
    r_c1_t.font.color.rgb = C_ACCENT

    p_c1_b = tf_c1.add_paragraph()
    p_c1_b.space_before = Pt(8)
    r_c1_b = p_c1_b.add_run()
    r_c1_b.text = (
        "• 2D 라이다는 로봇 통과 높이를 구분하지 못해 책상을 거대한 벽으로 인식함.\n"
        "• 본 시스템은 Grid/MaxObstacleHeight=0.65m 설정을 통해 75cm 책상 상판은 무시하고, "
        "다리 사이 공간을 안전하게 통과하는 유연한 경로 개척 성공."
    )
    r_c1_b.font.size = Pt(11)
    r_c1_b.font.color.rgb = C_TEXT_LIGHT

    # Card 2: Single Sensor Architecture
    card2 = s1.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, rc_left, Inches(4.35), rc_width, Inches(2.25))
    card2.fill.solid()
    card2.fill.fore_color.rgb = C_CARD
    card2.line.color.rgb = C_CARD_BORDER
    tf_c2 = card2.text_frame
    tf_c2.margin_left = tf_c2.margin_right = tf_c2.margin_top = Inches(0.25)
    p_c2_t = tf_c2.paragraphs[0]
    r_c2_t = p_c2_t.add_run()
    r_c2_t.text = "💡 센서 통합을 통한 극대화된 경제성"
    r_c2_t.font.bold = True
    r_c2_t.font.size = Pt(13)
    r_c2_t.font.color.rgb = C_GREEN

    p_c2_b = tf_c2.add_paragraph()
    p_c2_b.space_before = Pt(8)
    r_c2_b = p_c2_b.add_run()
    r_c2_b.text = (
        "• 기존 방식: 2D 라이다(주행) + 별도 카메라(도킹/AI) 장착 필요.\n"
        "• 본 프로젝트: 저비용 Astra S RGB-D 단 1대로 [3D 슬램 맵핑] + [ArUco 초정밀 도킹] + [YOLO 바닥 물체 검출]을 일원화하여 로봇 하드웨어 단가 및 전력 소모 대폭 절감."
    )
    r_c2_b.font.size = Pt(11)
    r_c2_b.font.color.rgb = C_TEXT_LIGHT

    # Speaker Notes
    s1.notes_slide.notes_text_frame.text = (
        "[발표 대본 - 슬라이드 1]\n"
        "왜 2D 라이다 대신 3D 비주얼 슬램을 선택했는가에 대한 내용입니다. "
        "기존 2D 라이다는 정해진 높이의 2D 평면만 스캔하기 때문에, 책상 상판이나 턱 같은 입체 장애물을 전혀 보지 못합니다. "
        "반면 저희 시스템은 3차원 깊이 정보를 바탕으로 유효 높이를 0.65m로 필터링하여, "
        "책상 상판은 무시하고 다리 사이 공간으로 안전하게 책상 밑 주행을 수행할 수 있습니다. "
        "또한 주행용 센서와 AI 인식용 카메라를 따로 달지 않고, 카메라 하나로 모든 작업을 통합하여 경제성과 효율성을 극대화했습니다."
    )

    # -------------------------------------------------------------
    # SLIDE 2: VSLAM CORE ALGORITHMS
    # -------------------------------------------------------------
    s2 = prs.slides.add_slide(blank_layout)
    set_slide_background(s2)
    add_header(s2, "CORE ALGORITHMS", "2. Visual SLAM 위치 추정 핵심 알고리즘",
               "PnP-RANSAC 3D 역투영 기하학과 전역 포즈 그래프 최적화, 50Hz 로컬 + 10Hz 전역 이중 보정 루프")

    card_w = Inches(2.7)
    card_h = Inches(4.8)
    spacing = Inches(0.25)
    c_left = Inches(0.8)

    # Step 1: ORB & Backprojection
    c1 = s2.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, c_left + 0 * (card_w + spacing), Inches(1.8), card_w, card_h)
    c1.fill.solid(); c1.fill.fore_color.rgb = C_CARD; c1.line.color.rgb = C_CARD_BORDER
    tf1 = c1.text_frame; tf1.margin_left = tf1.margin_right = tf1.margin_top = Inches(0.2)
    p = tf1.paragraphs[0]; r = p.add_run(); r.text = "STEP 1\n3D 점군 역투영\n(Back-projection)"; r.font.bold = True; r.font.size = Pt(13); r.font.color.rgb = C_PRIMARY
    p = tf1.add_paragraph(); p.space_before = Pt(12)
    r = p.add_run()
    r.text = (
        "• 특징점 추출: ORB(Oriented FAST + BRIEF) 프레임당 최대 1,500개 검출.\n\n"
        "• 실측 Depth 결합:\n"
        "  단안 카메라의 스케일 모호성을 해결하기 위해, 16-bit Depth 영상에서 실측 거리 Z를 조회하여 3D 복원:\n\n"
        "  Xc = (u - cx) * Z / fx\n"
        "  Yc = (v - cy) * Z / fy\n"
        "  Zc = Z\n\n"
        "• 유효거리(0.4~2.5m) 밖 노이즈는 자동 배제."
    )
    r.font.size = Pt(10.5); r.font.color.rgb = C_TEXT_LIGHT

    # Step 2: PnP-RANSAC
    c2 = s2.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, c_left + 1 * (card_w + spacing), Inches(1.8), card_w, card_h)
    c2.fill.solid(); c2.fill.fore_color.rgb = C_CARD; c2.line.color.rgb = C_CARD_BORDER
    tf2 = c2.text_frame; tf2.margin_left = tf2.margin_right = tf2.margin_top = Inches(0.2)
    p = tf2.paragraphs[0]; r = p.add_run(); r.text = "STEP 2\nPnP & RANSAC\n(상대 변환 추정)"; r.font.bold = True; r.font.size = Pt(13); r.font.color.rgb = C_PRIMARY
    p = tf2.add_paragraph(); p.space_before = Pt(12)
    r = p.add_run()
    r.text = (
        "• PnP / SVD 정합:\n"
        "  3D 공간점과 2D 특징점 대응 쌍을 바탕으로 EPnP 및 Kabsch SVD 알고리즘을 통해 회전(R)과 이동(t) 도출.\n\n"
        "• RANSAC 아웃라이어 제거:\n"
        "  반복 벽면 패턴 및 동적 장애물 오매칭 배제. 인라이어 >= 20개 이상 시에만 유효 변환 확정.\n\n"
        "• 비선형 최적화(LM):\n"
        "  재투영 오차(Reprojection Error)를 최소화하여 mm 단위 초정밀 정렬."
    )
    r.font.size = Pt(10.5); r.font.color.rgb = C_TEXT_LIGHT

    # Step 3: DBoW2 & Graph Opt
    c3 = s2.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, c_left + 2 * (card_w + spacing), Inches(1.8), card_w, card_h)
    c3.fill.solid(); c3.fill.fore_color.rgb = C_CARD; c3.line.color.rgb = C_CARD_BORDER
    tf3 = c3.text_frame; tf3.margin_left = tf3.margin_right = tf3.margin_top = Inches(0.2)
    p = tf3.paragraphs[0]; r = p.add_run(); r.text = "STEP 3\n루프 클로저 & 그래프 최적화"; r.font.bold = True; r.font.size = Pt(13); r.font.color.rgb = C_PRIMARY
    p = tf3.add_paragraph(); p.space_before = Pt(12)
    r = p.add_run()
    r.text = (
        "• DBoW2 시각 단어 가방:\n"
        "  과거 방문 키프레임의 어휘 사전을 역인덱스(Inverted Index)로 10ms 이내 초고속 후보 검색.\n\n"
        "• 포즈 그래프(Pose Graph):\n"
        "  - 노드: 로봇의 시점별 3D 절대 자세\n"
        "  - 엣지: EKF 오도메트리 제약 + 시각 루프 클로저 제약\n\n"
        "• g2o / GTSAM 최적화:\n"
        "  루프 감지 즉시 전체 오차 행렬을 일괄 완화하여 누적 드리프트를 0cm로 영점 수렴."
    )
    r.font.size = Pt(10.5); r.font.color.rgb = C_TEXT_LIGHT

    # Step 4: Dual-Loop TF
    c4 = s2.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, c_left + 3 * (card_w + spacing), Inches(1.8), card_w, card_h)
    c4.fill.solid(); c4.fill.fore_color.rgb = C_CARD; c4.line.color.rgb = C_CARD_BORDER
    tf4 = c4.text_frame; tf4.margin_left = tf4.margin_right = tf4.margin_top = Inches(0.2)
    p = tf4.paragraphs[0]; r = p.add_run(); r.text = "STEP 4\n50Hz + 10Hz 이중 루프\n(TF 브로드캐스트)"; r.font.bold = True; r.font.size = Pt(13); r.font.color.rgb = C_ACCENT
    p = tf4.add_paragraph(); p.space_before = Pt(12)
    r = p.add_run()
    r.text = (
        "• 1계층 로컬 EKF (50Hz / 20ms):\n"
        "  IMU(100Hz) + 엔코더(30Hz) 융합으로 무지연 연속 속도 제어.\n"
        "  TF: odom -> base_footprint\n\n"
        "• 2계층 전역 SLAM (10Hz / 100ms):\n"
        "  3D 비전 정합으로 EKF 바퀴 슬립 드리프트를 실시간 리셋.\n"
        "  TF: map -> odom 발행\n\n"
        "• 분리 설계의 장점:\n"
        "  비전 보정 시에도 로봇이 튕기는 충격(Jump) 없이 부드러운 자율주행 유지."
    )
    r.font.size = Pt(10.5); r.font.color.rgb = C_TEXT_LIGHT

    # Speaker Notes
    s2.notes_slide.notes_text_frame.text = (
        "[발표 대본 - 슬라이드 2]\n"
        "Visual SLAM의 위치 추정 파이프라인은 크게 4단계로 동작합니다. "
        "먼저 ORB로 1,500개의 특징점을 뽑은 뒤, Astra S 카메라의 16비트 실측 Depth 값을 결합해 실제 물리 세계의 3차원 공간 좌표로 역투영합니다. "
        "이후 PnP-RANSAC 알고리즘과 레벤버그-마쿼트 비선형 최적화로 직전 프레임과의 3차원 움직임을 정확히 계산합니다. "
        "로봇이 출발점으로 복귀할 때는 DBoW2 시각 단어 가방 모델이 루프를 감지하고, g2o 그래프 최적화 엔진이 주행 중 누적되었던 모든 드리프트를 0cm로 보정합니다. "
        "최종적으로 로컬 50Hz EKF와 전역 10Hz 비전 SLAM이 이중 루프로 분리되어, 제어기는 부드럽게 유지하면서 지도상의 위치 오차는 완벽히 제거합니다."
    )

    # -------------------------------------------------------------
    # SLIDE 3: IMPLEMENTATION 1 (3D VSLAM & DISTRIBUTED NETWORK)
    # -------------------------------------------------------------
    s3 = prs.slides.add_slide(blank_layout)
    set_slide_background(s3)
    add_header(s3, "IMPLEMENTATION 1", "3. 3D Visual SLAM 구현 및 분산 네트워크 최적화",
               "무선 대역폭 한계와 시스템 시계 편차를 엔지니어링으로 극복한 안정적 3D 복원 및 2D 점유격자 지도 투영")

    col_w = Inches(5.6)
    col_h = Inches(4.8)

    # Left Column: Network Bottleneck Solved
    c_l3 = s3.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.8), col_w, col_h)
    c_l3.fill.solid(); c_l3.fill.fore_color.rgb = C_CARD; c_l3.line.color.rgb = C_CARD_BORDER
    tf_l3 = c_l3.text_frame; tf_l3.margin_left = tf_l3.margin_right = tf_l3.margin_top = Inches(0.3)
    p = tf_l3.paragraphs[0]; r = p.add_run(); r.text = "📡 Wi-Fi 대역폭 병목 & 시계 동기화 극복"; r.font.bold = True; r.font.size = Pt(14); r.font.color.rgb = C_PRIMARY
    p = tf_l3.add_paragraph(); p.space_before = Pt(14)
    r = p.add_run()
    r.text = (
        "1. 초경량 압축 전송 (지연시간 3초 ➔ 30ms 단축)\n"
        "   - 문제: 640x480 RGB + 16-bit Depth 원본(30~50MB/s)은 Wi-Fi 포화로 극심한 패킷 드롭 및 3초 딜레이 유발.\n"
        "   - 해결: 라즈베리파이에서 JPEG(RGB)+무손실 PNG(Depth) 하드웨어 압축 수행 후 BEST_EFFORT QoS로 송신. 지연시간을 0.03초 이내로 단축.\n\n"
        "2. 락스텝 동기화 & 버퍼 플러시 (compressed_bridge_node)\n"
        "   - 컬러와 깊이의 타임스탬프 차이 |t_color - t_depth| <= 0.07s 검증.\n"
        "   - 처리 즉시 과거 프레임을 버퍼에서 폐기하여 회전 시 모션 블러 및 잔상(Ghosting) 원천 차단.\n\n"
        "3. 원격 나노초 시계 동기화 (5초 프리징 100% 영구 해결)\n"
        "   - 문제: 파이와 PC 간 2.95초 시계 편차로 RTAB-Map이 센서 타임아웃(5초 프리징) 발생.\n"
        "   - 해결: SSH로 PC 나노초 시각 자동 주입(sudo date -s). 시간 편차를 5ms 이내로 영점 조준."
    )
    r.font.size = Pt(11); r.font.color.rgb = C_TEXT_LIGHT

    # Right Column: 3D to 2D Occupancy Grid
    c_r3 = s3.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(6.9), Inches(1.8), col_w, col_h)
    c_r3.fill.solid(); c_r3.fill.fore_color.rgb = C_CARD; c_r3.line.color.rgb = C_CARD_BORDER
    tf_r3 = c_r3.text_frame; tf_r3.margin_left = tf_r3.margin_right = tf_r3.margin_top = Inches(0.3)
    p = tf_r3.paragraphs[0]; r = p.add_run(); r.text = "🗺️ 3D Depth ➔ 2D 점유격자 지도 실시간 투영"; r.font.bold = True; r.font.size = Pt(14); r.font.color.rgb = C_GREEN
    p = tf_r3.add_paragraph(); p.space_before = Pt(14)
    r = p.add_run()
    r.text = (
        "1. 라이다 없는 순수 비주얼 내비게이션 환경 구축\n"
        "   - Grid/FromDepth: true, Grid/Sensor: 1 설정으로 3D 점군을 Nav2 주행용 2D /map 점유격자로 실시간 역투영.\n\n"
        "2. 정밀 지면 분리 및 높이 필터링 파라미터\n"
        "   - Grid/CellSize: 0.05m (5cm 정밀 격자 해상도)\n"
        "   - Grid/MinGroundHeight: 0.05m (바닥 광택 반사 노이즈 차단)\n"
        "   - Grid/MaxObstacleHeight: 0.65m (천장 및 75cm 책상 상판은 무시하여 책상 밑 통과 주행 보장)\n\n"
        "3. 광선 추적 (Ray Tracing) 동적 장애물 제거\n"
        "   - 사람이 지나가거나 물체가 이동할 때, 카메라 시선 경로 상의 이전 장애물 셀을 자동으로 주행 가능(Free Space) 영역으로 즉시 복원."
    )
    r.font.size = Pt(11); r.font.color.rgb = C_TEXT_LIGHT

    # Speaker Notes
    s3.notes_slide.notes_text_frame.text = (
        "[발표 대본 - 슬라이드 3]\n"
        "실제 구현 과정에서 맞닥뜨린 가장 큰 난제는 Wi-Fi 대역폭 한계와 시스템 시계 편차였습니다. "
        "파이에서 초당 수십 메가바이트의 원본 영상을 보내면 3초 이상의 렉이 걸렸는데, "
        "JPEG과 PNG 압축 및 BEST_EFFORT QoS를 적용해 지연시간을 30ms 이내로 단축시켰습니다. "
        "또한 두 컴퓨터 간의 2.95초 시계 편차로 인해 지도가 5초마다 멈추던 문제를 SSH 원격 나노초 동기화로 5ms 이내로 맞춰 완벽히 해결했습니다. "
        "그 결과 3D 점군으로부터 5cm 격자의 2D 주행 지도를 실시간으로 매끄럽게 투영할 수 있게 되었습니다."
    )

    # -------------------------------------------------------------
    # SLIDE 4: IMPLEMENTATION 2 (DUAL ARUCO DOCKING)
    # -------------------------------------------------------------
    s4 = prs.slides.add_slide(blank_layout)
    set_slide_background(s4)
    add_header(s4, "IMPLEMENTATION 2", "4. Dual ArUco 마커 기반 초정밀 충전 도킹 시스템",
               "Visual SLAM 대역 주행 후 오차 ±1cm, ±1° 이내로 충전 스테이션에 정밀 안착 및 물리 접촉 안전 차단")

    # 3 Cards Layout
    c_w4 = Inches(3.7)
    c_h4 = Inches(4.8)
    sp4 = Inches(0.3)

    # Card 1: Dual ArUco Geometry
    c1_4 = s4.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8) + 0*(c_w4+sp4), Inches(1.8), c_w4, c_h4)
    c1_4.fill.solid(); c1_4.fill.fore_color.rgb = C_CARD; c1_4.line.color.rgb = C_CARD_BORDER
    tf = c1_4.text_frame; tf.margin_left = tf.margin_right = tf.margin_top = Inches(0.25)
    p = tf.paragraphs[0]; r = p.add_run(); r.text = "📐 Dual ArUco 삼각 측량"; r.font.bold = True; r.font.size = Pt(13); r.font.color.rgb = C_PRIMARY
    p = tf.add_paragraph(); p.space_before = Pt(12)
    r = p.add_run()
    r.text = (
        "• 듀얼 마커 배치 (DICT_4X4_50):\n"
        "  Left #7, Right #19 마커를 150mm 간격으로 대칭 배치.\n\n"
        "• 헤딩각 및 거리 고정밀 산출:\n"
        "  단일 마커의 불안정한 Yaw각 떨림을 극복하기 위해, 두 마커의 3D 공간 중심점 간 벡터를 구해 법선 각도를 삼각 측량.\n\n"
        "• 신뢰도 검증:\n"
        "  두 마커가 동시 검출될 때만 정밀 유도 알고리즘 진입."
    )
    r.font.size = Pt(11); r.font.color.rgb = C_TEXT_LIGHT

    # Card 2: PND Guidance Law
    c2_4 = s4.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8) + 1*(c_w4+sp4), Inches(1.8), c_w4, c_h4)
    c2_4.fill.solid(); c2_4.fill.fore_color.rgb = C_CARD; c2_4.line.color.rgb = C_CARD_BORDER
    tf = c2_4.text_frame; tf.margin_left = tf.margin_right = tf.margin_top = Inches(0.25)
    p = tf.paragraphs[0]; r = p.add_run(); r.text = "🎯 PND 정밀 제어 곡선"; r.font.bold = True; r.font.size = Pt(13); r.font.color.rgb = C_PRIMARY
    p = tf.add_paragraph(); p.space_before = Pt(12)
    r = p.add_run()
    r.text = (
        "• 3단계 가변 감속 프로파일:\n"
        "  1) 진입 정렬 (d > 0.8m): 헤딩각 편차 우선 회전.\n"
        "  2) 정밀 유도 (0.25m < d <= 0.8m): 비례 항법 유도(PND)로 횡방향 오차(Ey)를 0으로 수렴.\n"
        "  3) 최종 압착 (d <= 0.25m): 0.03m/s 초저속 정밀 크리핑 전진.\n\n"
        "• 최종 도킹 정밀도:\n"
        "  - 거리 편차: ±1cm 이내\n"
        "  - 각도 편차: ±1.0° 이내 안착"
    )
    r.font.size = Pt(11); r.font.color.rgb = C_TEXT_LIGHT

    # Card 3: Hardware Safety & 3D Gizmo
    c3_4 = s4.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8) + 2*(c_w4+sp4), Inches(1.8), c_w4, c_h4)
    c3_4.fill.solid(); c3_4.fill.fore_color.rgb = C_CARD; c3_4.line.color.rgb = C_CARD_BORDER
    tf = c3_4.text_frame; tf.margin_left = tf.margin_right = tf.margin_top = Inches(0.25)
    p = tf.paragraphs[0]; r = p.add_run(); r.text = "🛡️ 범퍼 센서 & 3D 기즈모 티칭"; r.font.bold = True; r.font.size = Pt(13); r.font.color.rgb = C_GREEN
    p = tf.add_paragraph(); p.space_before = Pt(12)
    r = p.add_run()
    r.text = (
        "• 물리 범퍼 마이크로스위치 안전 차단:\n"
        "  최종 5cm 구간에서 카메라가 마커를 가리더라도, 범퍼 접촉 즉시 GPIO 인터럽트 발생 ➔ 모터 긴급 정지 및 도킹 성공 체결.\n\n"
        "• RViz 3D 기즈모 인터랙티브 티칭:\n"
        "  teach_docking_station.sh를 통해 RViz 상에서 도킹 스테이션 위치를 3D 마커 핸들로 드래그하여 직관적으로 티칭 및 YAML 자동 저장."
    )
    r.font.size = Pt(11); r.font.color.rgb = C_TEXT_LIGHT

    # Speaker Notes
    s4.notes_slide.notes_text_frame.text = (
        "[발표 대본 - 슬라이드 4]\n"
        "슬램으로 도킹장 앞까지 온 로봇은 듀얼 ArUco 마커 기반 정밀 도킹 모드로 진입합니다. "
        "단일 마커는 각도가 떨리는 단점이 있어, 7번과 19번 두 마커의 중심선 벡터를 삼각측량하여 정확한 자세를 계산했습니다. "
        "비례 항법 유도(PND) 제어로 좌우 편차를 줄이며 부드러운 S자 곡선으로 접근하고, 최종 25cm부터는 초저속으로 압착합니다. "
        "마지막 접촉 순간에는 전면 하드웨어 범퍼 스위치가 눌리면서 1cm, 1도 이내의 정밀도로 안전하게 도킹을 완료합니다."
    )

    # -------------------------------------------------------------
    # SLIDE 5: IMPLEMENTATION 3 (YOLO & LLM FLOOR APPROACH)
    # -------------------------------------------------------------
    s5 = prs.slides.add_slide(blank_layout)
    set_slide_background(s5)
    add_header(s5, "IMPLEMENTATION 3", "5. Qwen2.5-VL & YOLOv8 바닥 타겟 물체 접근 시스템",
               "자율 순찰 중 실시간 발견 인터럽트, 하이브리드 지면 기하학 거리 추정, 카메라 사각지대 극복 정밀 안착")

    col_w5 = Inches(5.6)
    col_h5 = Inches(4.8)

    # Left Column: Hybrid Depth Perception
    c_l5 = s5.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.8), col_w5, col_h5)
    c_l5.fill.solid(); c_l5.fill.fore_color.rgb = C_CARD; c_l5.line.color.rgb = C_CARD_BORDER
    tf = c_l5.text_frame; tf.margin_left = tf.margin_right = tf.margin_top = Inches(0.3)
    p = tf.paragraphs[0]; r = p.add_run(); r.text = "👁️ 3단계 하이브리드 3D 거리 측정 기하학"; r.font.bold = True; r.font.size = Pt(14); r.font.color.rgb = C_PRIMARY
    p = tf.add_paragraph(); p.space_before = Pt(14)
    r = p.add_run()
    r.text = (
        "1. 원거리 (> 1.2m) : 단안 지면 핀홀 기하학\n"
        "   - 카메라 물리 장착 높이(H=14.7cm)와 캘리브레이션된 피치각(+4.2°) 삼각비 역투영.\n"
        "   - 바운딩 박스 최하단 지면 접촉점(v_bottom)을 이용해 센서 한계거리 밖에서도 거리 계산.\n\n"
        "2. 중근거리 (0.4m ~ 1.2m) : Astra S 3D Depth 융합\n"
        "   - 박스 중앙 ROI 뎁스 중앙값(Median)을 실측하여 오차 없는 mm급 거리 피드백.\n\n"
        "3. 초근거리 (< 0.4m) : 블라인드존 오도메트리 데드레커닝\n"
        "   - 문제: 40cm 미만 접근 시 물체가 카메라 아래 시야 밖으로 사라짐.\n"
        "   - 해결: 시야 상실 직전의 상대 좌표를 EKF 오도메트리 변위 적분으로 공간 기억하여 25cm 목표 위치에 오차 없이 부드럽게 감속 정지."
    )
    r.font.size = Pt(11); r.font.color.rgb = C_TEXT_LIGHT

    # Right Column: Patrol Intercept & Toolchain
    c_r5 = s5.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(6.9), Inches(1.8), col_w5, col_h5)
    c_r5.fill.solid(); c_r5.fill.fore_color.rgb = C_CARD; c_r5.line.color.rgb = C_CARD_BORDER
    tf = c_r5.text_frame; tf.margin_left = tf.margin_right = tf.margin_top = Inches(0.3)
    p = tf.paragraphs[0]; r = p.add_run(); r.text = "🎯 순찰 중 자동 발견 인터럽트 & 툴체인"; r.font.bold = True; r.font.size = Pt(14); r.font.color.rgb = C_ACCENT
    p = tf.add_paragraph(); p.space_before = Pt(14)
    r = p.add_run()
    r.text = (
        "1. 순찰 중 자동 발견 및 팝업 인터럽트 (Patrol Intercept)\n"
        "   - Nav2 웨이포인트 주행 중 전방 0.45~2.2m 내에서 타겟(큐브/공/막대)이 3프레임 연속 검출되면 즉시 주행을 일시정지.\n"
        "   - GUI 팝업창에서 사용자 확인 시 25cm 정밀 접근, 취소 시 주행 재개.\n\n"
        "2. PyQt5 미션 컨트롤 다크테마 콕핏 GUI\n"
        "   - 실시간 20Hz 타겟 락온 박스, 헤딩각 오차 표시, 원클릭 타겟 프리셋 버튼 제공.\n\n"
        "3. 원클릭 4단계 YOLO 파인튜닝 툴체인\n"
        "   - tools/yolo_training/ 도구를 통해 [데이터 수집 ➔ 자동 라벨링 ➔ GPU 학습 ➔ 무동작 3D 검증] 원클릭 수행.\n"
        "   - floor_target_objects.yaml 선언적 등록으로 코드 수정 없이 새 물체 즉시 적용."
    )
    r.font.size = Pt(11); r.font.color.rgb = C_TEXT_LIGHT

    # Speaker Notes
    s5.notes_slide.notes_text_frame.text = (
        "[발표 대본 - 슬라이드 5]\n"
        "마지막으로 바닥에 놓인 타겟 물체에 정밀 접근하는 비전 서보잉 시스템입니다. "
        "로봇이 웨이포인트를 순찰하다가 바닥의 물체를 발견하면 자동으로 주행을 멈추고 팝업으로 사용자에게 접근 여부를 묻습니다. "
        "거리 측정 시 1.2m 밖에서는 카메라의 물리 높이와 피치각을 이용한 핀홀 기하학을 쓰고, "
        "중간 거리에서는 3D 뎁스 센서를 쓰며, 카메라 바로 앞 40cm 미만의 사각지대로 물체가 사라지면 "
        "오도메트리 공간 기억(Dead-reckoning)을 활용해 25cm 스탠드오프 지점에 정확히 멈춥니다. "
        "새로운 물체 등록도 원클릭 툴체인으로 완비되어 있습니다."
    )

    # -------------------------------------------------------------
    # SLIDE 6: CONCLUSION & IMPACT
    # -------------------------------------------------------------
    s6 = prs.slides.add_slide(blank_layout)
    set_slide_background(s6)
    add_header(s6, "SUMMARY & VALUE", "6. 종합 성과 및 결론 (Conclusion)",
               "고가의 2D 라이다 없이, 단일 깊이 카메라로 공간 복원부터 정밀 작업까지 완결한 로봇 솔루션")

    # 4 Highlight Metric Cards
    m_w = Inches(2.7)
    m_h = Inches(2.2)
    m_top = Inches(1.8)

    metrics = [
        ("30ms 이하", "네트워크 지연시간", "Raw 대비 Wi-Fi 트래픽 대폭 절감", C_PRIMARY),
        ("±1cm / ±1°", "도킹 체결 정밀도", "Dual ArUco + PND 항법 유도", C_GREEN),
        ("0.65m 제어", "헤드룸 통과 경로 개척", "75cm 책상 밑 입체 자율주행 달성", C_ACCENT),
        ("25cm 정밀 안착", "바닥 물체 접근 거리", "사각지대 오도메트리 데드레커닝", C_SECONDARY)
    ]

    for i, (val, title, desc, col) in enumerate(metrics):
        box = s6.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, c_left + i*(m_w+spacing), m_top, m_w, m_h)
        box.fill.solid(); box.fill.fore_color.rgb = C_CARD; box.line.color.rgb = col; box.line.width = Pt(1.5)
        tf = box.text_frame; tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        p = tf.paragraphs[0]; p.alignment = PP_ALIGN.CENTER
        r = p.add_run(); r.text = val; r.font.bold = True; r.font.size = Pt(22); r.font.color.rgb = col
        p2 = tf.add_paragraph(); p2.alignment = PP_ALIGN.CENTER; p2.space_before = Pt(6)
        r2 = p2.add_run(); r2.text = title; r2.font.bold = True; r2.font.size = Pt(12); r2.font.color.rgb = C_TEXT_LIGHT
        p3 = tf.add_paragraph(); p3.alignment = PP_ALIGN.CENTER; p3.space_before = Pt(4)
        r3 = p3.add_run(); r3.text = desc; r3.font.size = Pt(10); r3.font.color.rgb = C_TEXT_MUTED

    # Bottom Takeaway Box
    bot_box = s6.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(4.3), Inches(11.7), Inches(2.3))
    bot_box.fill.solid(); bot_box.fill.fore_color.rgb = C_CARD; bot_box.line.color.rgb = C_CARD_BORDER
    tf_b = bot_box.text_frame; tf_b.margin_left = tf_b.margin_top = Inches(0.3)
    p = tf_b.paragraphs[0]; r = p.add_run(); r.text = "🚀 프로젝트 핵심 기여 및 결론"; r.font.bold = True; r.font.size = Pt(14); r.font.color.rgb = C_PRIMARY
    p = tf_b.add_paragraph(); p.space_before = Pt(10)
    r = p.add_run()
    r.text = (
        "1. 순수 비전 기반 3D 공간 자율주행의 완벽한 실증:\n"
        "   - 고가 2D 라이다를 전면 배제하고 저비용 Astra S RGB-D 카메라 1대로 3D 공간 복원, 2D 점유격자 투영, 자율주행을 안정적으로 완성.\n\n"
        "2. 실전 분산 로봇 환경의 병목 엔지니어링 완결:\n"
        "   - 라즈베리파이-PC 간 Wi-Fi 병목을 초경량 락스텝 압축으로 해결하고, 원격 나노초 시간 동기화로 SLAM 타임아웃 문제를 원천 봉쇄.\n\n"
        "3. SLAM ➔ 정밀 도킹 ➔ AI 물체 작업으로 이어지는 엔드투엔드 파이프라인 구현:\n"
        "   - 단순 주행에 그치지 않고, cm 단위 초정밀 충전 도킹과 자율 순찰 중 바닥 물체 발견 & 정밀 접근까지 하나의 시스템으로 완결."
    )
    r.font.size = Pt(11); r.font.color.rgb = C_TEXT_LIGHT

    # Speaker Notes
    s6.notes_slide.notes_text_frame.text = (
        "[발표 대본 - 슬라이드 6]\n"
        "결론적으로 이번 프로젝트는 2D 라이다에 의존하던 모바일 로봇의 한계를 깨고, "
        "단일 RGB-D 카메라로 3D 공간 맵핑, 책상 밑 통과 주행, 1cm급 정밀 도킹, 그리고 AI 바닥 물체 서보잉까지 "
        "모두 성공적으로 통합한 상용 수준의 엔드투엔드 자율주행 솔루션입니다. "
        "이상으로 발표를 마치겠습니다. 경청해 주셔서 감사합니다. 질문 있으시면 답변드리겠습니다."
    )

    out_path = "/home/jjj/Documents/Dapier/Project/Visual_SLAM_Presentation.pptx"
    prs.save(out_path)
    print(f"Successfully generated PowerPoint presentation at: {out_path}")

if __name__ == "__main__":
    create_deck()
