#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# YOLOv8 Custom Floor Object Fine-Tuning Pipeline
# - Targets: red_cube (Class 0), blue_ball (Class 1)
# - GPU-accelerated on NVIDIA GeForce RTX 5050
# - Data Augmentation tuned for small/distant floor objects (Mosaic, Scale, HSV)
# - Exports best weights to custom_floor_detector_best.pt
# ==============================================================================

import os
import sys
import time
import shutil
import argparse
import torch
from ultralytics import YOLO


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DATA_YAML = os.path.join(PROJECT_ROOT, "dataset_capture", "yolo_dataset", "data.yaml")
OUTPUT_BEST_MODEL = os.path.join(PROJECT_ROOT, "custom_floor_detector_best.pt")
RUNS_DIR = os.path.join(PROJECT_ROOT, "robot_runs")


def train_yolo(epochs: int = 60, batch_size: int = 16, imgsz: int = 320, model_name: str = "yolov8s.pt"):
    print("=" * 75)
    print(" 🚀 [YOLOv8 커스텀 바닥 물체 파인튜닝 시작]")
    print(f"   - 데이터셋 설정 : {DATA_YAML}")
    print(f"   - 기본 백본 모델 : {model_name}")
    print(f"   - 학습 에포크    : {epochs} Epochs")
    print(f"   - 배치 크기      : {batch_size}")
    print(f"   - 입력 해상도    : {imgsz}x{imgsz}")

    if not os.path.exists(DATA_YAML):
        print(f"❌ 데이터셋 설정 파일을 찾을 수 없습니다: {DATA_YAML}")
        sys.exit(1)

    # Device check
    if torch.cuda.is_available():
        device_id = 0
        device_name = torch.cuda.get_device_name(0)
        print(f"   - 연산 가속 장치 : GPU (ID: {device_id}, {device_name})")
    else:
        device_id = "cpu"
        print("   - 연산 가속 장치 : CPU (경고: 속도가 느릴 수 있습니다)")
    print("=" * 75 + "\n")

    # Load pretrained YOLOv8 model
    model = YOLO(model_name)

    t_start = time.time()

    # Train with augmentations tuned for floor objects & distant small objects
    results = model.train(
        data=DATA_YAML,
        epochs=epochs,
        batch=batch_size,
        imgsz=imgsz,
        device=device_id,
        project=RUNS_DIR,
        name="custom_floor_detector",
        exist_ok=True,
        # Augmentations for distance & lighting robustness:
        mosaic=1.0,         # stitches 4 images to make objects small (simulates far distance)
        mixup=0.1,          # blending for occlusion tolerance
        scale=0.5,          # scale jitter (0.5x ~ 1.5x)
        degrees=10.0,       # small rotation
        hsv_h=0.015,        # slight hue variation
        hsv_s=0.7,          # saturation variation
        hsv_v=0.4,          # brightness variation
        patience=25,        # early stopping patience
        save=True,
        plots=True,
        verbose=True
    )

    elapsed_min = (time.time() - t_start) / 60.0

    # Best weights path from training run save_dir
    save_dir = str(results.save_dir) if hasattr(results, 'save_dir') else os.path.join(RUNS_DIR, "custom_floor_detector")
    best_pt = os.path.join(save_dir, "weights", "best.pt")
    if not os.path.exists(best_pt):
        # Fallback search
        alt_paths = [
            os.path.join(RUNS_DIR, "custom_floor_detector", "weights", "best.pt"),
            "runs/detect/robot_runs/custom_floor_detector/weights/best.pt",
            "robot_runs/custom_floor_detector/weights/best.pt"
        ]
        for p in alt_paths:
            if os.path.exists(p):
                best_pt = p
                break

    if os.path.exists(best_pt):
        shutil.copyfile(best_pt, OUTPUT_BEST_MODEL)
        print("\n" + "=" * 75)
        print(f"🎉 [학습 완료! 소요 시간: {elapsed_min:.1f}분]")
        print(f"   - 최적 가중치 저장됨: {OUTPUT_BEST_MODEL}")
        print(f"   - 훈련 그래프 및 평가 결과: robot_runs/custom_floor_detector/")
        print("=" * 75 + "\n")

        # Quick validation
        print("🔍 [검증 세트 최종 평가 수행 중...]")
        val_results = model.val(data=DATA_YAML, device=device_id, verbose=False)
        maps = val_results.results_dict
        print(f"   - mAP@50     : {maps.get('metrics/mAP50(B)', 0.0):.4f}")
        print(f"   - mAP@50-95  : {maps.get('metrics/mAP50-95(B)', 0.0):.4f}")
        print("=" * 75)
    else:
        print(f"⚠️ 학습이 종료되었으나 {best_pt}를 찾을 수 없습니다.")


def main():
    parser = argparse.ArgumentParser(description="Train custom YOLO for floor objects")
    parser.add_argument('--epochs', type=int, default=60, help="Training epochs (default: 60)")
    parser.add_argument('--batch', type=int, default=16, help="Batch size (default: 16)")
    parser.add_argument('--imgsz', type=int, default=320, help="Image size (default: 320)")
    parser.add_argument('--model', type=str, default="yolov8s.pt", help="Pretrained model (yolov8s.pt or yolov8n.pt)")
    args = parser.parse_args()

    train_yolo(epochs=args.epochs, batch_size=args.batch, imgsz=args.imgsz, model_name=args.model)


if __name__ == '__main__':
    main()
