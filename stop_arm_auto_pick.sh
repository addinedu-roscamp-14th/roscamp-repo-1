#!/bin/bash

# tmux 자동 집기 세션 이름입니다.
SESSION_NAME="poter_ws"

echo "=============================================="
echo " JetCobot 자동 집기 시스템 종료"
echo "=============================================="

# tmux 세션을 종료합니다.
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# ROS 2 카메라 실행 래퍼를 종료합니다.
pkill -9 -f "ros2 run camera_ros camera_node" 2>/dev/null || true

# 실제 camera_ros 카메라 노드를 종료합니다.
pkill -9 -f "/opt/ros/jazzy/lib/camera_ros/camera_node" 2>/dev/null || true

# 왜곡 보정 노드를 종료합니다.
pkill -9 -f "/opt/ros/jazzy/lib/image_proc/rectify_node" 2>/dev/null || true

# 로봇팔 서버를 종료합니다.
pkill -9 -f "robot_arm_server.py" 2>/dev/null || true

# 자동 집기 클라이언트를 종료합니다.
pkill -9 -f "arm_camera_rect_auto_pick_client.py" 2>/dev/null || true

# 프로세스 종료 처리를 잠시 기다립니다.
sleep 1

echo "[확인] 남아 있는 관련 프로세스를 검사합니다."

# 남은 관련 프로세스를 확인합니다.
REMAINING=$(
    ps aux |
    grep -E "camera_node|rectify_node|robot_arm_server|arm_camera_rect_auto_pick_client" |
    grep -v grep
)

# 남은 프로세스가 있는지 검사합니다.
if [ -n "$REMAINING" ]; then
    echo "[경고] 일부 프로세스가 남아 있습니다."
    echo "$REMAINING"
else
    echo "[완료] 자동 집기 시스템을 모두 종료했습니다."
fi
