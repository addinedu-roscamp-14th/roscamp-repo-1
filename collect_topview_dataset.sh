#!/bin/bash

set -eo pipefail

WORKSPACE="$HOME/poter_ws"
ROS_DOMAIN="${ROS_DOMAIN_ID:-77}"
CAMERA_NAME="${TOP_CAMERA_NAME:-ABKO APC925}"
CAMERA_DEVICE="${TOP_CAMERA_DEVICE:-auto}"
WIDTH="${TOP_CAMERA_WIDTH:-640}"
HEIGHT="${TOP_CAMERA_HEIGHT:-480}"
FPS="${TOP_CAMERA_FPS:-30}"
OUTPUT="${TOPVIEW_DATASET_OUTPUT:-$WORKSPACE/datasets/topview/unlabeled}"

source /opt/ros/jazzy/setup.bash
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID="$ROS_DOMAIN"

CAMERA_PID=""
cleanup() {
    if [ -n "$CAMERA_PID" ]; then
        kill "$CAMERA_PID" 2>/dev/null || true
        wait "$CAMERA_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[실행] 탑뷰 카메라를 시작합니다. (로봇팔은 실행하지 않음)"
ros2 run calibration topview_usb_camera \
    --ros-args \
    -p camera_name:="$CAMERA_NAME" \
    -p camera_device:="$CAMERA_DEVICE" \
    -p width:="$WIDTH" \
    -p height:="$HEIGHT" \
    -p fps:="$FPS" \
    -p display:=false &
CAMERA_PID=$!

sleep 2
if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
    echo "[오류] 탑뷰 카메라를 시작하지 못했습니다." >&2
    wait "$CAMERA_PID"
    exit 1
fi

echo "[수집] R: 영상 녹화 시작/중지, S: 사진 저장, Q/ESC: 종료"
python3 "$WORKSPACE/scripts/collect_topview_images.py" \
    --topic /top_camera/image_raw \
    --output "$OUTPUT" \
    --video-output "$WORKSPACE/datasets/topview/videos" \
    --fps "$FPS"
