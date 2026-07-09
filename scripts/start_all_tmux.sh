#!/bin/bash

# Start a tmux session that launches the ROS nodes and scripts described by the user.
# Usage: ./start_all_tmux.sh

set -e
SESSION="poter_ws"
TMUX_BIN=$(command -v tmux || true)
if [ -z "$TMUX_BIN" ]; then
  echo "tmux가 설치되어 있지 않습니다. 설치 후 다시 시도하세요: sudo apt install tmux"
  exit 1
fi

# Common environment commands
ENV_CMD='source /opt/ros/jazzy/setup.bash && source ~/YOLO/install/setup.bash && export ROS_DOMAIN_ID=12'

# Camera index (0 = USB2_0Camera, 1 = HD Webcam).
# Change by exporting CAMERA_IDX before running this script, e.g.
#   CAMERA_IDX=1 ./scripts/start_all_tmux.sh
CAMERA_IDX=${CAMERA_IDX:-0}

# Choose camera_info file based on CAMERA_IDX
if [ "$CAMERA_IDX" -eq 0 ]; then
  CAMERA_INFO_URL="file:///home/rsj/camera_calib/arm_camera_info.yaml"
else
  CAMERA_INFO_URL="file:///home/rsj/camera_calib/arm_camera_info_hd.yaml"
fi

# If a session already exists, ask and kill or attach
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "이미 tmux 세션 '$SESSION'이 존재합니다. 기존 세션을 종료하고 새로 만듭니다."
  tmux kill-session -t "$SESSION"
fi

# Create session and windows (only first 4)
# 1: 공통 환경 (쉘 유지)
# 2: 로봇팔 카메라 (video2)
# 3: 로봇팔 보정 (image_proc rectify)
# 4: 로봇팔 서버

# Window 1: env
tmux new-session -d -s "$SESSION" -n env bash -lc "$ENV_CMD; echo \"환경 로드 완료 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)\"; exec bash"

# Window 2: arm camera
tmux new-window -t "$SESSION" -n arm_camera bash -lc "$ENV_CMD; ros2 run camera_ros camera_node --ros-args -p camera:=$CAMERA_IDX -p width:=640 -p height:=480 -p format:=\"YUYV\" -p frame_id:=\"arm_camera_optical_frame\" -p camera_info_url:=\"$CAMERA_INFO_URL\"; exec bash"

# Window 3: arm rectify
tmux new-window -t "$SESSION" -n arm_rectify bash -lc "$ENV_CMD; ros2 run image_proc rectify_node --ros-args -r __ns:=/camera -r image:=image_raw -r camera_info:=camera_info -r image_rect:=image_rect; exec bash"

# Window 4: robot arm server
tmux new-window -t "$SESSION" -n arm_server bash -lc "$ENV_CMD; python3 ~/poter_ws/src/arm/arm/robot_arm_server.py; exec bash"

# Quick device check for serial port used by arm server (예: /dev/ttyUSB0)
if [ ! -e /dev/ttyUSB0 ]; then
  echo "경고: /dev/ttyUSB0 장치가 없습니다. 로봇암 연결을 확인하세요. (robot_arm_server에서 SerialException 발생 가능)"
fi

echo "tmux 세션 '$SESSION' 생성 완료. 접속하려면: tmux attach -t $SESSION"

echo "윈도우 목록:"
mt=$(tmux list-windows -t "$SESSION" 2>/dev/null || true)
echo "$mt"

exit 0
