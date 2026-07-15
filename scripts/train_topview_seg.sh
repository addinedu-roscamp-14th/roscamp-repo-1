#!/bin/bash

set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/poter_ws}"
DATA="${DATA:-$WORKSPACE/config/topview_dataset.yaml}"
MODEL="${MODEL:-$WORKSPACE/config/weights/best.pt}"
EPOCHS="${EPOCHS:-100}"
IMGSZ="${IMGSZ:-640}"
BATCH="${BATCH:-8}"
DEVICE="${DEVICE:-}"

if ! command -v yolo >/dev/null 2>&1; then
    echo "[오류] Ultralytics yolo 명령을 찾을 수 없습니다."
    exit 1
fi

if [ ! -f "$DATA" ]; then
    echo "[오류] 데이터셋 설정이 없습니다: $DATA"
    exit 1
fi

if [ ! -f "$MODEL" ]; then
    echo "[오류] 기본 모델이 없습니다: $MODEL"
    exit 1
fi

DEVICE_ARG=()
if [ -n "$DEVICE" ]; then
    DEVICE_ARG=("device=$DEVICE")
fi

exec yolo segment train \
    "model=$MODEL" \
    "data=$DATA" \
    "epochs=$EPOCHS" \
    "imgsz=$IMGSZ" \
    "batch=$BATCH" \
    "project=$WORKSPACE/runs/topview_seg" \
    "name=train" \
    "${DEVICE_ARG[@]}"
