#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# YOLO Auto-Annotator & Interactive High-Precision Dataset Labeler
# - Automatically detects 'red_cube' (Class 0) and 'blue_ball' (Class 1)
# - Large 3X Zoom Viewport (960x720) for easy view of tiny distant cubes
# - Robust Mouse Drawing: BOTH Click-and-Drag AND Two-Click Corner supported!
# - Right-Click to Delete any individual box instantly
# - Undo [Z] / Clear [D] / Class Switch [0], [1], [TAB]
# - Automatically splits into YOLO format: images/train, images/val, labels/train, labels/val
# ==============================================================================

import os
import sys
import glob
import shutil
import random
import argparse
from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import cv2

try:
    from ultralytics import YOLOWorld
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

CLASSES = ["red_cube", "blue_ball", "white_tissue"]
CLASS_COLORS = [
    (0, 0, 255),     # 0: red_cube (Red)
    (255, 120, 0),   # 1: blue_ball (Blue/Cyan)
    (255, 255, 50),  # 2: white_tissue (Bright Mint/Yellow-Green outline)
    (0, 255, 0),     # 3: Green
    (255, 0, 255),   # 4: Magenta
]
YOLO_WEIGHTS_PATH = os.path.join(PROJECT_ROOT, "yolov8s-worldv2.pt")
DISPLAY_SCALE = 1.0  # Dynamic adaptive scaling applied in InteractiveLabeler


class AutoAnnotator:
    def __init__(self, data_root: str, use_yolo_world: bool = True):
        self.data_root = os.path.abspath(data_root)
        self.use_yolo_world = use_yolo_world and YOLO_AVAILABLE and os.path.exists(YOLO_WEIGHTS_PATH)

        self.yolo_model = None
        if self.use_yolo_world:
            print("🚀 [YOLO-World 로드 중] 다중 클래스 보조 검출기 활성화...")
            self.yolo_model = YOLOWorld(YOLO_WEIGHTS_PATH)
            self.yolo_model.set_classes([
                "red cube", "blue ball", "sports ball",
                "crumpled tissue", "crumpled paper", "white tissue",
                "tissue paper", "tissue", "paper ball", "wad of paper",
                "toilet paper", "tissue clump"
            ])
            dummy = np.zeros((240, 320, 3), dtype=np.uint8)
            self.yolo_model.predict(dummy, device=0, verbose=False)
            print(f"✅ YOLO-World 보조 검출기 준비 완료 (클래스: {CLASSES})")

    # ==========================================================================
    # Auto-Detection Logic (Color-based Detection for Black/Red/Blue + YOLO fallback)
    # ==========================================================================
    def detect_objects(self, bgr_img: np.ndarray, img_path: Optional[str] = None) -> List[Dict[str, Any]]:
        H, W = bgr_img.shape[:2]
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        detections = []
        parent_dir = os.path.basename(os.path.dirname(img_path)).lower() if img_path else ""
        k_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        # ----------------------------------------------------------------------
        # (Black Objects excluded to prevent dark floor/shadow false positives)
        # ----------------------------------------------------------------------

        # ----------------------------------------------------------------------
        # 2. Red Cube (Strict Saturation >= 85 to prevent floor tile false-positives)
        # ----------------------------------------------------------------------
        if parent_dir == "red_cube" or not parent_dir:
            mask_r1 = cv2.inRange(hsv, (0, 85, 40), (12, 255, 255))
            mask_r2 = cv2.inRange(hsv, (168, 85, 40), (180, 255, 255))
            mask_r = mask_r1 | mask_r2

            # Floor gate: Exclude top 22%
            mask_r[:int(H * 0.22), :] = 0
            mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, k_open)
            mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_CLOSE, k_open)

            cnts_r, _ = cv2.findContours(mask_r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_cands = []
            for c in cnts_r:
                area = cv2.contourArea(c)
                if area >= 12:
                    x, y, w, h = cv2.boundingRect(c)
                    if not (y + h >= H - 2 and h <= 8):
                        aspect = w / float(h)
                        if 0.35 <= aspect <= 2.8 and "red_cube" in CLASSES:
                            red_cands.append({
                                "cls_id": CLASSES.index("red_cube"),
                                "cls_name": "red_cube",
                                "bbox": (x, y, w, h),
                                "area": area,
                                "conf": min(0.99, 0.50 + (area / 1000.0))
                            })
            red_cands = sorted(red_cands, key=lambda d: d["area"], reverse=True)
            if red_cands:
                detections.append(red_cands[0])
                if len(red_cands) > 1 and red_cands[1]["area"] > red_cands[0]["area"] * 0.35:
                    detections.append(red_cands[1])

        # ----------------------------------------------------------------------
        # 3. Blue Ball (HSV: H 95~135, S >= 80, V >= 50)
        # ----------------------------------------------------------------------
        if parent_dir == "blue_ball" or not parent_dir:
            mask_b = cv2.inRange(hsv, (95, 80, 50), (135, 255, 255))
            mask_b[:int(H * 0.25), :] = 0
            mask_b = cv2.morphologyEx(mask_b, cv2.MORPH_OPEN, k_open)
            mask_b = cv2.morphologyEx(mask_b, cv2.MORPH_CLOSE, k_open)

            cnts_b, _ = cv2.findContours(mask_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            b_cands = []
            for c in cnts_b:
                area = cv2.contourArea(c)
                if area >= 20:
                    x, y, w, h = cv2.boundingRect(c)
                    if 0.45 <= w / float(h) <= 2.2 and "blue_ball" in CLASSES:
                        b_cands.append({
                            "cls_id": CLASSES.index("blue_ball"),
                            "cls_name": "blue_ball",
                            "bbox": (x, y, w, h),
                            "area": area,
                            "conf": 0.95
                        })
            b_cands = sorted(b_cands, key=lambda d: d["area"], reverse=True)
            if b_cands:
                detections.append(b_cands[0])

        # ----------------------------------------------------------------------
        # 4. White Tissue Clump (HSV: Low Saturation S <= 62, High Value V >= 165)
        # ----------------------------------------------------------------------
        if parent_dir in ("white_tissue", "tissue") or not parent_dir:
            mask_w = cv2.inRange(hsv, np.array([0, 0, 165], dtype=np.uint8), np.array([180, 62, 255], dtype=np.uint8))
            # Floor gate: Exclude top 20% horizon, extreme border noise
            mask_w[:int(H * 0.20), :] = 0
            mask_w[H - 3:, :] = 0
            mask_w[:, :3] = 0
            mask_w[:, W - 3:] = 0

            mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, k_open)
            mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_CLOSE, k_open)

            cnts_w, _ = cv2.findContours(mask_w, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            w_cands = []
            for c in cnts_w:
                area = cv2.contourArea(c)
                if area >= 20:
                    x, y, w, h = cv2.boundingRect(c)
                    # Filter out edge noise or whole-image glare
                    if not (y + h >= H - 2 and h <= 8) and w < W * 0.80 and h < H * 0.80:
                        aspect = w / float(h)
                        # Tissue clump is roughly compact or irregular (0.35 <= aspect <= 3.0)
                        if 0.30 <= aspect <= 3.20 and "white_tissue" in CLASSES:
                            w_cands.append({
                                "cls_id": CLASSES.index("white_tissue"),
                                "cls_name": "white_tissue",
                                "bbox": (x, y, w, h),
                                "area": area,
                                "conf": 0.92
                            })
            w_cands = sorted(w_cands, key=lambda d: d["area"], reverse=True)
            if w_cands:
                detections.append(w_cands[0])
                if len(w_cands) > 1 and w_cands[1]["area"] > w_cands[0]["area"] * 0.40:
                    detections.append(w_cands[1])

        # ----------------------------------------------------------------------
        # 5. YOLO-World Fallback (if no detections found by color)
        # ----------------------------------------------------------------------
        if len(detections) == 0 and self.yolo_model is not None:
            results = self.yolo_model.predict(bgr_img, conf=0.18, device=0, verbose=False)
            if results and len(results[0].boxes) > 0:
                yolo_classes = results[0].names
                for b in results[0].boxes:
                    xyxy = b.xyxy[0].cpu().numpy().astype(int)
                    conf = float(b.conf[0])
                    raw_cls_id = int(b.cls[0])
                    detected_lbl = yolo_classes.get(raw_cls_id, "").lower()

                    target_cls_id = None
                    if "red" in detected_lbl and "red_cube" in CLASSES:
                        target_cls_id = CLASSES.index("red_cube")
                    elif ("ball" in detected_lbl or "blue" in detected_lbl) and "blue_ball" in CLASSES:
                        target_cls_id = CLASSES.index("blue_ball")
                    elif any(k in detected_lbl for k in ("tissue", "paper", "clump", "wad", "toilet")) and "white_tissue" in CLASSES:
                        target_cls_id = CLASSES.index("white_tissue")

                    if target_cls_id is not None:
                        x1, y1, x2, y2 = xyxy
                        x1, y1 = max(0, x1), max(0, y1)
                        x2, y2 = min(W, x2), min(H, y2)
                        bw, bh = x2 - x1, y2 - y1
                        if bw >= 5 and bh >= 5 and y2 > int(H * 0.20):
                            detections.append({
                                "cls_id": target_cls_id,
                                "cls_name": CLASSES[target_cls_id],
                                "bbox": (x1, y1, bw, bh),
                                "area": bw * bh,
                                "conf": conf
                            })

        return detections


# ==============================================================================
# Interactive GUI / Mouse Box Drawer with 3X Zoom & Two-Click Support
# ==============================================================================
class InteractiveLabeler:
    def __init__(self, annotator: AutoAnnotator, image_paths: List[str], initial_class: int = 0):
        self.annotator = annotator
        self.image_paths = sorted(image_paths)
        self.curr_idx = 0
        self.labels_dict: Dict[str, List[Dict[str, Any]]] = {}

        # Scale factor for display
        self.scale = DISPLAY_SCALE

        # Mouse state
        self.drawing = False
        self.click_point: Optional[Tuple[int, int]] = None  # for 2-click mode
        self.mouse_x = 0
        self.mouse_y = 0
        self.start_x = 0
        self.start_y = 0
        self.manual_class = initial_class

    def _disp_to_orig(self, dx: int, dy: int, orig_w: int, orig_h: int) -> Tuple[int, int]:
        ox = int(round(dx / self.scale))
        oy = int(round(dy / self.scale))
        return max(0, min(orig_w - 1, ox)), max(0, min(orig_h - 1, oy))

    def mouse_callback(self, event, x, y, flags, param):
        self.mouse_x = x
        self.mouse_y = y

        if self.curr_idx >= len(self.image_paths):
            return

        cur_path = self.image_paths[self.curr_idx]
        cur_boxes = self.labels_dict.setdefault(cur_path, [])

        # ----------------------------------------------------------------------
        # 1. Right Click: Delete Box Under Cursor
        # ----------------------------------------------------------------------
        if event == cv2.EVENT_RBUTTONDOWN:
            ox = int(x / self.scale)
            oy = int(y / self.scale)
            # Find box containing (ox, oy) or closest
            deleted = False
            for idx in reversed(range(len(cur_boxes))):
                bx, by, bw, bh = cur_boxes[idx]["bbox"]
                if bx <= ox <= bx + bw and by <= oy <= by + bh:
                    removed = cur_boxes.pop(idx)
                    print(f"🗑️ [우클릭 삭제] {removed['cls_name']} 박스 제거됨: {removed['bbox']}")
                    deleted = True
                    break
            if not deleted and cur_boxes:
                # If clicked near a box (within 15px)
                for idx in reversed(range(len(cur_boxes))):
                    bx, by, bw, bh = cur_boxes[idx]["bbox"]
                    if (bx - 10) <= ox <= (bx + bw + 10) and (by - 10) <= oy <= (by + bh + 10):
                        removed = cur_boxes.pop(idx)
                        print(f"🗑️ [우클릭 삭제] 근처 {removed['cls_name']} 박스 제거됨: {removed['bbox']}")
                        break
            return

        # ----------------------------------------------------------------------
        # 2. Left Button Down: Start Drag or First Click
        # ----------------------------------------------------------------------
        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.drawing:
                # First click / Drag start
                self.drawing = True
                self.start_x = x
                self.start_y = y
                self.click_point = (x, y)
            else:
                # Second click in 2-click mode!
                self._finish_box(x, y, cur_path, cur_boxes)

        elif event == cv2.EVENT_LBUTTONUP:
            if self.drawing:
                # Check if moved significantly (drag-and-release mode)
                dist = np.hypot(x - self.start_x, y - self.start_y)
                if dist >= 8:  # User dragged!
                    self._finish_box(x, y, cur_path, cur_boxes)
                # Else: keep self.drawing = True for 2-click mode!

    def _finish_box(self, x: int, y: int, cur_path: str, cur_boxes: List[Dict[str, Any]]):
        self.drawing = False
        dx1, dx2 = min(self.start_x, x), max(self.start_x, x)
        dy1, dy2 = min(self.start_y, y), max(self.start_y, y)

        ox1 = int(round(dx1 / self.scale))
        oy1 = int(round(dy1 / self.scale))
        ox2 = int(round(dx2 / self.scale))
        oy2 = int(round(dy2 / self.scale))

        bw = ox2 - ox1
        bh = oy2 - oy1

        if bw >= 3 and bh >= 3:
            cls_name = CLASSES[self.manual_class]
            cur_boxes.append({
                "cls_id": self.manual_class,
                "cls_name": cls_name,
                "bbox": (ox1, oy1, bw, bh),
                "conf": 1.00
            })
            print(f"✏️ [박스 생성 성공] {cls_name}: (x={ox1}, y={oy1}, w={bw}, h={bh})")
        else:
            print("⚠️ 박스 크기가 너무 작아서 취소되었습니다.")
        self.click_point = None

    def run(self, auto_all: bool = False):
        win_name = "YOLO High-Precision Labeler (3X Zoom)"
        cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(win_name, self.mouse_callback)

        total = len(self.image_paths)
        print("\n" + "=" * 75)
        print(" 🎯 [YOLO 고정밀 라벨러 시작 - 3배 확대 창 (960x720)]")
        print(f"   - 총 이미지 수   : {total}장")
        print("   - [마우스 드래그] : 박스 그리기 (클릭 드래그 or 2번 클릭 모두 지원!)")
        print("   - [마우스 우클릭] : 커서가 위치한 잘못된 박스 즉시 삭제")
        print("   - [ENTER / SPACE]: 현재 라벨 승인하고 다음 이미지로")
        print("   - [Z] / [U]       : 방금 추가한 박스 실행취소(Undo)")
        print("   - [D]             : 현재 이미지 박스 전체 삭제")
        key_hints = ", ".join([f"[{i}]: {c}" for i, c in enumerate(CLASSES)])
        print(f"   - 클래스 선택 키  : {key_hints} (또는 [TAB]으로 순환)")
        print("   - [A]             : 남은 모든 이미지 자동 일괄 승인 (Auto-All)")
        print("   - [B]             : 이전 이미지로 돌아가기")
        print("   - [Q] / ESC       : 저장 후 종료")
        print("=" * 75 + "\n")

        auto_mode = auto_all

        while self.curr_idx < total:
            img_path = self.image_paths[self.curr_idx]
            raw_img = cv2.imread(img_path)
            if raw_img is None:
                self.curr_idx += 1
                continue

            orig_h, orig_w = raw_img.shape[:2]

            # Auto-calculate screen scale so window fits comfortably on screen
            target_max_w = 960.0
            target_max_h = 720.0
            if orig_w <= 360 and orig_h <= 270:
                self.scale = 3.0  # For small 320x240 camera frames
            elif orig_w <= 640 and orig_h <= 480:
                self.scale = 1.25 # For 640x480
            else:
                # For high-res images (e.g. phone 1080x1080) scale down to fit screen
                self.scale = min(1.0, target_max_w / float(orig_w), target_max_h / float(orig_h))

            disp_w = int(orig_w * self.scale)
            disp_h = int(orig_h * self.scale)

            # Auto-assign active draw class based on parent folder (e.g. black_cube, black_rod)
            parent_dir = os.path.basename(os.path.dirname(img_path))
            if parent_dir in CLASSES:
                self.manual_class = CLASSES.index(parent_dir)

            # Auto-detect on first visit
            if img_path not in self.labels_dict:
                self.labels_dict[img_path] = self.annotator.detect_objects(raw_img, img_path=img_path)

            if auto_mode:
                self.curr_idx += 1
                continue

            # Render Loop
            while True:
                # 3X Nearest neighbor scaling for clear pixel borders
                disp = cv2.resize(raw_img, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)

                curr_boxes = self.labels_dict.get(img_path, [])

                # Draw existing boxes in high resolution
                for b in curr_boxes:
                    cls_id = b["cls_id"]
                    ox, oy, ow, oh = b["bbox"]
                    conf = b.get("conf", 1.0)

                    dx1 = int(ox * self.scale)
                    dy1 = int(oy * self.scale)
                    dx2 = int((ox + ow) * self.scale)
                    dy2 = int((oy + oh) * self.scale)

                    color = CLASS_COLORS[cls_id % len(CLASS_COLORS)]
                    cv2.rectangle(disp, (dx1, dy1), (dx2, dy2), color, 2)

                    cls_name = CLASSES[cls_id] if cls_id < len(CLASSES) else f"cls_{cls_id}"
                    tag = f"{cls_name} {conf:.2f}"
                    cv2.rectangle(disp, (dx1, max(0, dy1 - 22)), (dx1 + len(tag) * 9 + 8, dy1), (20, 20, 20), -1)
                    cv2.putText(disp, tag, (dx1 + 4, max(14, dy1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

                # Draw currently active rubber-band box
                if self.drawing:
                    dx1, dx2 = min(self.start_x, self.mouse_x), max(self.start_x, self.mouse_x)
                    dy1, dy2 = min(self.start_y, self.mouse_y), max(self.start_y, self.mouse_y)

                    active_color = CLASS_COLORS[self.manual_class % len(CLASS_COLORS)]
                    cv2.rectangle(disp, (dx1, dy1), (dx2, dy2), active_color, 2)
                    # Crosshair
                    cv2.line(disp, (self.mouse_x - 12, self.mouse_y), (self.mouse_x + 12, self.mouse_y), (0, 255, 255), 1)
                    cv2.line(disp, (self.mouse_x, self.mouse_y - 12), (self.mouse_x, self.mouse_y + 12), (0, 255, 255), 1)

                    tag_draw = f"+ {CLASSES[self.manual_class]} (Click/Release to finish)"
                    cv2.putText(disp, tag_draw, (dx1, max(18, dy1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)

                # Top Status Banner
                overlay = disp.copy()
                cv2.rectangle(overlay, (0, 0), (disp_w, 48), (20, 25, 35), -1)
                disp = cv2.addWeighted(overlay, 0.82, disp, 0.18, 0)

                fname = os.path.basename(img_path)
                curr_class_name = CLASSES[self.manual_class]
                class_color = CLASS_COLORS[self.manual_class % len(CLASS_COLORS)]

                cv2.putText(disp, f"[{self.curr_idx + 1}/{total}] {fname} ({parent_dir}) | Boxes: {len(curr_boxes)}",
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 1)

                hint_str = " / ".join([f"[{i}]:{c}" for i, c in enumerate(CLASSES)])
                cv2.putText(disp, f"Active Draw: [{curr_class_name.upper()}]  ({hint_str} or [TAB])",
                            (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.44, class_color, 1)

                # Bottom Guide
                cv2.putText(disp, "[Drag/2-Click] Draw | [R-Click] Del | [Z] Undo | [D] Clear | [R] Re-detect | [ENTER] Next",
                            (10, disp_h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (230, 230, 230), 1)

                cv2.imshow(win_name, disp)
                key = cv2.waitKey(20) & 0xFF

                if key in (13, 32):  # ENTER or SPACE
                    self.curr_idx += 1
                    break
                elif key == ord('a'):
                    print("\n>> ⚡ [Auto-All 모드 전환] 남은 모든 이미지를 자동 라벨링하여 승인합니다...")
                    auto_mode = True
                    self.curr_idx += 1
                    break
                elif ord('0') <= key <= ord('9'):
                    cls_num = key - ord('0')
                    if cls_num < len(CLASSES):
                        self.manual_class = cls_num
                        print(f">> 활성 클래스: [{cls_num}] {CLASSES[self.manual_class]}")
                elif key == 9:  # TAB
                    self.manual_class = (self.manual_class + 1) % len(CLASSES)
                    print(f">> 클래스 전환: [{self.manual_class}] {CLASSES[self.manual_class]}")
                elif key in (ord('z'), ord('u')):
                    if curr_boxes:
                        popped = curr_boxes.pop()
                        print(f"↩️ [실행취소] {popped['cls_name']} 박스 취소됨")
                elif key == ord('r'):
                    self.labels_dict[img_path] = self.annotator.detect_objects(raw_img, img_path=img_path)
                    print(f">> 🔄 [{fname}] 색상 기반 자동 검출 다시 실행 완료 (검출 수: {len(self.labels_dict[img_path])}개)")
                elif key == ord('d'):
                    self.labels_dict[img_path] = []
                    print(">> 🗑️ 현재 이미지 박스 전체 삭제")
                elif key == ord('s'):
                    self.labels_dict.pop(img_path, None)
                    print(">> ⏭️ 건너뛰기")
                    self.curr_idx += 1
                    break
                elif key == ord('b'):
                    self.curr_idx = max(0, self.curr_idx - 1)
                    break
                elif key in (ord('q'), 27):
                    print("\n>> 라벨링을 종료하고 저장합니다.")
                    self.curr_idx = total
                    break

        cv2.destroyAllWindows()


# ==============================================================================
# YOLO Dataset Formatter (Images, Labels, Train/Val Split)
# ==============================================================================
def export_yolo_dataset(output_dir: str, labels_dict: Dict[str, List[Dict[str, Any]]], val_ratio: float = 0.2):
    dataset_dir = os.path.abspath(output_dir)
    img_train_dir = os.path.join(dataset_dir, "images", "train")
    img_val_dir = os.path.join(dataset_dir, "images", "val")
    lbl_train_dir = os.path.join(dataset_dir, "labels", "train")
    lbl_val_dir = os.path.join(dataset_dir, "labels", "val")

    for d in [img_train_dir, img_val_dir, lbl_train_dir, lbl_val_dir]:
        os.makedirs(d, exist_ok=True)

    items = list(labels_dict.items())
    random.seed(42)
    random.shuffle(items)

    val_count = max(1, int(len(items) * val_ratio))
    val_items = set(p for p, _ in items[:val_count])

    stats = {i: 0 for i in range(len(CLASSES))}
    total_imgs = 0

    print("\n📦 [YOLO 데이터셋 변환 및 저장 중...]")

    for src_img_path, boxes in items:
        if not os.path.exists(src_img_path):
            continue

        im = cv2.imread(src_img_path)
        if im is None:
            continue
        H, W = im.shape[:2]

        is_val = src_img_path in val_items
        dest_img_dir = img_val_dir if is_val else img_train_dir
        dest_lbl_dir = lbl_val_dir if is_val else lbl_train_dir

        base_name = os.path.splitext(os.path.basename(src_img_path))[0]
        ext = os.path.splitext(src_img_path)[1]

        target_img_path = os.path.join(dest_img_dir, f"{base_name}{ext}")
        target_lbl_path = os.path.join(dest_lbl_dir, f"{base_name}.txt")

        shutil.copyfile(src_img_path, target_img_path)

        lines = []
        for b in boxes:
            cls_id = int(b["cls_id"])
            x, y, w, h = b["bbox"]

            x_center = (x + w * 0.5) / float(W)
            y_center = (y + h * 0.5) / float(H)
            norm_w = w / float(W)
            norm_h = h / float(H)

            lines.append(f"{cls_id} {x_center:.6f} {y_center:.6f} {norm_w:.6f} {norm_h:.6f}")
            stats[cls_id] = stats.get(cls_id, 0) + 1

        with open(target_lbl_path, "w") as f:
            f.write("\n".join(lines) + "\n")

        total_imgs += 1

    # Generate data.yaml dynamically from CLASSES
    names_yaml = "\n".join([f"  {i}: {c}" for i, c in enumerate(CLASSES)])
    yaml_content = f"""# YOLOv8 Fine-Tuning Dataset Configuration
path: {dataset_dir}
train: images/train
val: images/val

names:
{names_yaml}
"""
    yaml_path = os.path.join(dataset_dir, "data.yaml")
    with open(yaml_path, "w") as f:
        f.write(yaml_content)

    print("=" * 70)
    print("🎉 [YOLO 데이터셋 생성 완료]")
    print(f"   - 총 이미지 수  : {total_imgs}장 (Train: {total_imgs - val_count}장, Val: {val_count}장)")
    box_stats_str = ", ".join([f"[{i}] {CLASSES[i]}: {stats.get(i, 0)}개" for i in range(len(CLASSES))])
    print(f"   - 라벨 박스 통계: {box_stats_str}")
    print(f"   - data.yaml 경로: {yaml_path}")
    print("=" * 70)


# ==============================================================================
# Main
# ==============================================================================
def main():
    global CLASSES
    parser = argparse.ArgumentParser(description="YOLO Multi-Class Auto-Annotator")
    parser.add_argument('--input', type=str, default=os.path.join(PROJECT_ROOT, "dataset_capture", "images"),
                        help="Input images root folder containing subfolders or jpg files")
    parser.add_argument('--output', type=str, default=os.path.join(PROJECT_ROOT, "dataset_capture", "yolo_dataset"),
                        help="Output YOLO dataset folder (train/val split)")
    parser.add_argument('--classes', type=str, default="",
                        help="Custom comma-separated class list (e.g. red_cube,blue_ball,black_cube,black_rod,white_tissue)")
    parser.add_argument('--label', '--target', type=str, default="",
                        help="Filter to label only specific class/folder (e.g. white_tissue)")
    parser.add_argument('--auto', action='store_true',
                        help="Auto-approve all images without showing GUI")
    args, unknown = parser.parse_known_args()

    # If --classes specified, override CLASSES
    if args.classes.strip():
        CLASSES = [c.strip() for c in args.classes.split(",") if c.strip()]
    elif os.path.exists(args.input):
        # Auto-detect folder names in input directory
        found_subdirs = [
            d for d in os.listdir(args.input)
            if os.path.isdir(os.path.join(args.input, d)) and "yolo_dataset" not in d
        ]
        if found_subdirs:
            # Preserve existing order and append any new ones
            for s in sorted(found_subdirs):
                if s not in CLASSES:
                    CLASSES.append(s)

    img_files = []
    for ext in ("*.jpg", "*.jpeg", "*.png"):
        img_files.extend(glob.glob(os.path.join(args.input, "**", ext), recursive=True))

    img_files = [f for f in img_files if "yolo_dataset" not in f]

    # Filter by specific label if requested
    init_cls_idx = 0
    target_filter = args.label.strip().lower()
    input_base = os.path.basename(os.path.normpath(args.input)).lower()
    if not target_filter and input_base in [c.lower() for c in CLASSES]:
        target_filter = input_base

    if target_filter:
        for idx, c in enumerate(CLASSES):
            if c.lower() == target_filter:
                init_cls_idx = idx
                break
        img_files = [
            f for f in img_files
            if target_filter == os.path.basename(os.path.dirname(f)).lower() or target_filter in os.path.basename(f).lower()
        ]
        print(f"🎯 [단일 라벨 필터 적용] '{CLASSES[init_cls_idx]}' 관련 이미지 {len(img_files)}장만 엽니다.")

    if not img_files:
        print(f"❌ {args.input} 디렉토리에서 대상 이미지를 찾을 수 없습니다.")
        return

    print(f"📂 총 {len(img_files)}장의 수집된 이미지를 발견했습니다.")

    annotator = AutoAnnotator(args.input, use_yolo_world=True)
    labeler = InteractiveLabeler(annotator, img_files, initial_class=init_cls_idx)

    labeler.run(auto_all=args.auto)

    if labeler.labels_dict:
        export_yolo_dataset(args.output, labeler.labels_dict, val_ratio=0.2)
        print("\n🚀 [파인튜닝 학습 바로 시작 명령어]")
        print(f"yolo detect train data={args.output}/data.yaml model=yolov8s.pt epochs=50 imgsz=320 batch=16 device=0 project=robot_runs name=custom_floor_detector\n")


if __name__ == '__main__':
    main()
