# UDP Package

UDP/GStreamer로 수신한 카메라 영상을 ROS 2 이미지 토픽으로 변환하는 fallback
패키지입니다. 카메라와 처리 노드가 같은 노트북에 있으면 UDP 없이 `v4l2_camera`를
사용합니다.

## 로컬 기본 방식

### 1. 원본 카메라

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video2 \
  -p image_size:="[640, 480]" \
  -p time_per_frame:="[1, 30]" \
  -p camera_info_url:=file:///home/jio/poter_ws/config/main_camera/camera_info.yaml \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

### 2. 왜곡 보정

```bash
ros2 run image_proc rectify_node --ros-args \
  -r image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  -r image_rect:=/camera/image_rect
```

YOLO와 calibration의 기본 입력 `/image_rect/compressed`가 자동으로 보이지 않으면
명시적으로 image transport를 실행합니다.

```bash
ros2 run image_transport republish raw compressed --ros-args \
  -r in:=/camera/image_rect \
  -r out:=/image_rect
```

확인:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/image_rect
ros2 topic hz /image_rect/compressed
```

## 원격 UDP Fallback

카메라가 연결된 송신 노트북과 영상을 처리할 수신 노트북에서 각각 실행합니다. 두 장비는
같은 네트워크에 있어야 합니다.

### 1. 송신 노트북

`<RECEIVER_IP>`를 수신 노트북의 실제 IP로 바꿉니다. `127.0.0.1`은 다른 노트북으로
전송되지 않습니다.

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video2 ! \
  image/jpeg,width=640,height=480,framerate=30/1 ! \
  jpegparse ! rtpjpegpay ! \
  udpsink host=<RECEIVER_IP> port=5000 sync=false async=false
```

### 2. 수신 노트북

워크스페이스 루트에서 실행합니다. `0.0.0.0`은 모든 로컬 네트워크 인터페이스에서 UDP를
수신합니다.

```bash
cd ~/poter_ws
source install/setup.bash
ros2 run udp udp_camera_node --ros-args \
  -p port:=5000 \
  -p bind_address:=0.0.0.0 \
  -p camera_info_yaml:=config/main_camera/camera_info.yaml
```

수신 노드는 다음 토픽을 발행합니다.

```text
/camera/image_raw
/camera/camera_info
```

### 3. 수신 영상 왜곡 보정

```bash
ros2 run image_proc rectify_node --ros-args \
  -r image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  -r image_rect:=/camera/image_rect
```

필요하면 위 로컬 방식과 동일하게 `/image_rect/compressed`를 발행합니다.

## 설정

```text
기본 UDP 포트: 5000
기본 bind 주소: 0.0.0.0
camera info: config/main_camera/camera_info.yaml
출력 이미지: /camera/image_raw
출력 camera info: /camera/camera_info
```

방화벽이 UDP 5000 포트를 차단하지 않는지 확인해야 합니다. UDP 영상 전송은 네트워크
상태에 따라 프레임 손실이 생길 수 있으므로 로컬 처리에서는 사용하지 않습니다.
