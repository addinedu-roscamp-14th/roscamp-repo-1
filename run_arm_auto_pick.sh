#!/bin/bash

# 오류가 발생해도 전체 스크립트가 불필요하게 종료되지 않도록 설정
set -u

# tmux 세션 이름
SESSION_NAME="poter_ws"

# 모든 ROS 2 노드에서 사용할 도메인 번호
ROS_DOMAIN="77"

# 로봇팔 서버 파일 경로
SERVER_FILE="$HOME/poter_ws/src/arm/arm/robot_arm_server.py"

# 자동 집기 클라이언트 파일 경로
CLIENT_FILE="$HOME/poter_ws/src/arm/arm/arm_camera_rect_auto_pick_client.py"

# camera_ros의 카메라 번호입니다. 기본값은 자동 감지입니다.
# 주의: /dev/video 번호가 아니라 camera_ros 목록의 0, 1, 2 같은 순번입니다.
# 필요하면 CAMERA_IDX=1 ./run_arm_auto_pick.sh 처럼 직접 지정할 수 있습니다.
CAMERA_IDX="${CAMERA_IDX:-auto}"

detect_camera_idx() {
    local preferred_name="${CAMERA_NAME:-USB 2.0 Camera}"
    local detected=""

    if command -v v4l2-ctl >/dev/null 2>&1; then
        detected="$(v4l2-ctl --list-devices 2>/dev/null | awk -v name="$preferred_name" '
            /^[^[:space:]].*:$/ {
                if ($0 ~ name) {
                    print camera_idx
                    exit
                }
                camera_idx++
            }
        ')"
    fi

    if [ -z "$detected" ] && command -v v4l2-ctl >/dev/null 2>&1; then
        detected="$(v4l2-ctl --list-devices 2>/dev/null | awk '
            /^[^[:space:]].*:$/ {
                if ($0 !~ /HD Webcam/) {
                    print camera_idx
                    exit
                }
                camera_idx++
            }
        ')"
    fi

    if [ -z "$detected" ] && command -v v4l2-ctl >/dev/null 2>&1; then
        detected="$(v4l2-ctl --list-devices 2>/dev/null | awk '
            /^[^[:space:]].*:$/ {
                print camera_idx
                exit
            }
        ')"
    fi

    if [ -n "$detected" ]; then
        echo "$detected"
    else
        echo "0"
    fi
}

if [ "$CAMERA_IDX" = "auto" ]; then
    CAMERA_IDX="$(detect_camera_idx)"
fi

# 카메라 캘리브레이션 파일 경로입니다. CAMERA_INFO_FILE로 직접 지정할 수도 있습니다.
if [ -z "${CAMERA_INFO_FILE:-}" ]; then
    CAMERA_INFO_FILE="$HOME/camera_calib/arm_camera_info.yaml"
fi

echo "=============================================="
echo " JetCobot 자동 집기 시스템 실행"
echo " ROS_DOMAIN_ID=$ROS_DOMAIN"
echo " CAMERA_IDX=$CAMERA_IDX"
echo " CAMERA_INFO_FILE=$CAMERA_INFO_FILE"
echo "=============================================="

# tmux 설치 여부 확인
if ! command -v tmux >/dev/null 2>&1; then
    echo "[오류] tmux가 설치되어 있지 않습니다."
    echo "설치 명령어: sudo apt install tmux"
    exit 1
fi

# 로봇팔 서버 파일 확인
if [ ! -f "$SERVER_FILE" ]; then
    echo "[오류] 로봇팔 서버 파일을 찾을 수 없습니다."
    echo "$SERVER_FILE"
    exit 1
fi

# 자동 집기 클라이언트 파일 확인
if [ ! -f "$CLIENT_FILE" ]; then
    echo "[오류] 자동 집기 클라이언트 파일을 찾을 수 없습니다."
    echo "$CLIENT_FILE"
    exit 1
fi

# 카메라 캘리브레이션 파일 확인
if [ ! -f "$CAMERA_INFO_FILE" ]; then
    echo "[오류] 카메라 캘리브레이션 파일을 찾을 수 없습니다."
    echo "$CAMERA_INFO_FILE"
    exit 1
fi

echo "[정리] 기존 자동 집기 프로그램을 종료합니다."

# 기존 tmux 세션 종료
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# 실제 camera_ros 실행 파일 종료
pkill -9 -f "/opt/ros/jazzy/lib/camera_ros/camera_node" 2>/dev/null || true

# ros2 run 카메라 실행 래퍼 종료
pkill -9 -f "ros2 run camera_ros camera_node" 2>/dev/null || true

# 왜곡 보정 노드 종료
pkill -9 -f "/opt/ros/jazzy/lib/image_proc/rectify_node" 2>/dev/null || true

# 로봇팔 서버 종료
pkill -9 -f "robot_arm_server.py" 2>/dev/null || true

# 자동 집기 클라이언트 종료
pkill -9 -f "arm_camera_rect_auto_pick_client.py" 2>/dev/null || true

# 장치 해제 대기
sleep 2

echo "[실행] 카메라 노드를 시작합니다."

# 터미널 1: 카메라 노드 실행
tmux new-session -d \
    -s "$SESSION_NAME" \
    -n "1-camera" \
    "bash -lc '
        source /opt/ros/jazzy/setup.bash

        if [ -f \"\$HOME/YOLO/install/setup.bash\" ]; then
            source \"\$HOME/YOLO/install/setup.bash\"
        fi

        if [ -f \"\$HOME/poter_ws/install/setup.bash\" ]; then
            source \"\$HOME/poter_ws/install/setup.bash\"
        fi

        export ROS_DOMAIN_ID=$ROS_DOMAIN

        echo \"=======================================\"
        echo \"[터미널 1] 로봇팔 카메라 실행\"
        echo \"ROS_DOMAIN_ID=\$ROS_DOMAIN_ID\"
        echo \"CAMERA_IDX=$CAMERA_IDX\"
        echo \"CAMERA_INFO_FILE=$CAMERA_INFO_FILE\"
        echo \"=======================================\"

        exec ros2 run camera_ros camera_node \
            --ros-args \
            -p camera:=$CAMERA_IDX \
            -p width:=640 \
            -p height:=480 \
            -p format:=YUYV \
            -p frame_id:=arm_camera_optical_frame \
            -p camera_info_url:=file://$CAMERA_INFO_FILE
    '"

# 실행 중 오류가 발생해도 tmux 창을 유지합니다.
tmux set-window-option -g -t "$SESSION_NAME" remain-on-exit on
tmux set-window-option -g -t "$SESSION_NAME" automatic-rename off
tmux set-option -t "$SESSION_NAME" allow-rename off

echo "[실행] 왜곡 보정 노드를 준비합니다."

# 터미널 2: 실제 원본 영상 메시지가 들어올 때까지 대기 후 왜곡 보정 실행
tmux new-window \
    -t "$SESSION_NAME" \
    -n "2-rectify" \
    "bash -lc '
        source /opt/ros/jazzy/setup.bash

        if [ -f \"\$HOME/YOLO/install/setup.bash\" ]; then
            source \"\$HOME/YOLO/install/setup.bash\"
        fi

        if [ -f \"\$HOME/poter_ws/install/setup.bash\" ]; then
            source \"\$HOME/poter_ws/install/setup.bash\"
        fi

        export ROS_DOMAIN_ID=$ROS_DOMAIN

        echo \"=======================================\"
        echo \"[터미널 2] 카메라 원본 영상 대기\"
        echo \"ROS_DOMAIN_ID=\$ROS_DOMAIN_ID\"
        echo \"=======================================\"

        while true; do
            if timeout 3 ros2 topic echo \
                /camera/image_raw \
                sensor_msgs/msg/Image \
                --once >/dev/null 2>&1
            then
                echo \"[확인] /camera/image_raw 영상 수신 성공\"
                break
            fi

            echo \"[대기] /camera/image_raw 영상이 아직 없습니다.\"
            sleep 1
        done

        echo \"[실행] 왜곡 보정 노드 시작\"

        exec ros2 run image_proc rectify_node \
            --ros-args \
            -r __node:=arm_rectify_node \
            -r image:=/camera/image_raw \
            -r camera_info:=/camera/camera_info \
            -r image_rect:=/camera/image_rect
    '"

echo "[실행] 로봇팔 서버를 준비합니다."

# 터미널 3: 로봇팔 서버 실행
tmux new-window \
    -t "$SESSION_NAME" \
    -n "3-server" \
    "bash -lc '
        source /opt/ros/jazzy/setup.bash

        if [ -f \"\$HOME/YOLO/install/setup.bash\" ]; then
            source \"\$HOME/YOLO/install/setup.bash\"
        fi

        if [ -f \"\$HOME/poter_ws/install/setup.bash\" ]; then
            source \"\$HOME/poter_ws/install/setup.bash\"
        fi

        export ROS_DOMAIN_ID=$ROS_DOMAIN

        echo \"=======================================\"
        echo \"[터미널 3] 로봇팔 서버 실행\"
        echo \"ROS_DOMAIN_ID=\$ROS_DOMAIN_ID\"
        echo \"=======================================\"

        sleep 3

        exec python3 \
            \"\$HOME/poter_ws/src/arm/arm/robot_arm_server.py\"
    '"

echo "[실행] 자동 집기 클라이언트를 준비합니다."

# 터미널 4: 실제 왜곡 보정 영상이 나올 때까지 대기 후 클라이언트 실행
tmux new-window \
    -t "$SESSION_NAME" \
    -n "4-client" \
    "bash -lc '
        source /opt/ros/jazzy/setup.bash

        if [ -f \"\$HOME/YOLO/install/setup.bash\" ]; then
            source \"\$HOME/YOLO/install/setup.bash\"
        fi

        if [ -f \"\$HOME/poter_ws/install/setup.bash\" ]; then
            source \"\$HOME/poter_ws/install/setup.bash\"
        fi

        export ROS_DOMAIN_ID=$ROS_DOMAIN

        echo \"=======================================\"
        echo \"[터미널 4] 로봇팔 서버 준비 대기\"
        echo \"ROS_DOMAIN_ID=\$ROS_DOMAIN_ID\"
        echo \"=======================================\"

        # 로봇팔 서버의 TCP 포트가 열릴 때까지 기다립니다.
        SERVER_READY=0

        for TRY_COUNT in \$(seq 1 40); do

            # Bash의 TCP 연결 기능으로 127.0.0.1:15000을 확인합니다.
            if timeout 1 bash -c \
                \"</dev/tcp/127.0.0.1/15000\" \
                >/dev/null 2>&1
            then
                echo \"[확인] robot_arm_server TCP 15000 준비 완료\"
                SERVER_READY=1
                break
            fi

            echo \"[대기] robot_arm_server 준비 중... \${TRY_COUNT}/40\"
            sleep 0.5
        done

        # 서버가 준비되지 않으면 클라이언트를 실행하지 않습니다.
        if [ \"\$SERVER_READY\" -ne 1 ]; then
            echo \"[오류] robot_arm_server가 준비되지 않았습니다.\"
            echo \"[확인] 터미널 3의 오류 로그를 확인하세요.\"
            exec bash
        fi

        echo \"[실행] 자동 집기 클라이언트 시작\"

        exec python3 \
            \"\$HOME/poter_ws/src/arm/arm/arm_camera_rect_auto_pick_client.py\"
    '"

# 첫 번째 카메라 창 선택
tmux select-window -t "$SESSION_NAME:1-camera"

echo "=============================================="
echo " 자동 집기 시스템 실행 완료"
echo ""
echo " tmux 창 이동:"
echo " Ctrl+B를 누른 후 N : 다음 창"
echo " Ctrl+B를 누른 후 P : 이전 창"
echo " Ctrl+B를 누른 후 숫자 : 해당 창 이동"
echo ""
echo " tmux 화면에서 나오기:"
echo " Ctrl+B를 누른 후 D"
echo "=============================================="

# 실행 중인 tmux 세션에 접속합니다.
# 터미널이 없는 상태에서 실행되면 세션만 띄워두고 종료합니다.
if [ -t 0 ] && [ -t 1 ]; then
    exec tmux attach-session -t "$SESSION_NAME"
else
    echo "비대화형 실행이라 tmux에 자동 접속하지 않습니다."
    echo "접속 명령어: tmux attach-session -t $SESSION_NAME"
fi
