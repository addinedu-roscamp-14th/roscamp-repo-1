# UDP 패키지

이 패키지는 UDP/GStreamer로 들어오는 카메라 영상을 ROS2 이미지 토픽으로 바꾸는 fallback 패키지입니다.

현재 기본 카메라 경로는 UDP가 아니라 아래 ROS2 compressed image 흐름을 사용합니다.

```text
v4l2_camera
→ /camera/image_raw
→ /camera/image_raw/compressed
→ /camera/image_raw_relay
→ /camera/image_rect
```

UDP 방식은 compressed 경로가 불안정하거나 별도 GStreamer 전송이 필요할 때 사용합니다.

## 기본 카메라 실행 방식

UDP를 사용하지 않는 현재 기본 방식입니다.

### 1. 원본 카메라 실행

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video2 \
  -p image_size:="[640, 480]" \
  -p time_per_frame:="[1, 30]" \
  -p camera_info_url:=file:///home/jio/poter_ws/config/main_camera/camera_info.yaml \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

### 2. compressed 발행

```bash
ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/camera/image_raw \
  -r out:=/camera/image_raw
```

### 3. compressed를 raw relay로 복원

```bash
ros2 run image_transport republish compressed raw \
  --ros-args \
  -r in:=/camera/image_raw \
  -r out:=/camera/image_raw_relay
```

### 4. 왜곡 보정 이미지 생성

```bash
ros2 run image_proc rectify_node \
  --ros-args \
  -r image:=/camera/image_raw_relay \
  -r camera_info:=/camera/camera_info \
  -r image_rect:=/camera/image_rect
```

최종적으로 YOLO와 calibration은 아래 토픽을 사용합니다.

```text
/camera/image_rect
```

## 실행 순서

아래는 UDP fallback 방식입니다.

### 1. 카메라 영상 송신
아래 명령으로 카메라 영상을 UDP로 전송합니다.

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video2 ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegparse ! rtpjpegpay ! tee name=t \
  t. ! queue ! udpsink host=127.0.0.1 port=5000 
```

### 2. UDP 수신 노드 실행
`udp` 패키지의 노드를 실행합니다.

```bash
ros2 run udp udp_camera_node --ros-args \
  -p port:=5000 \
  -p bind_address:="127.0.0.1" \
  -p camera_info_yaml:="config/main_camera/camera_info.yaml"
```

### 3. 이미지 보정 노드 실행
필요하면 보정 노드를 따로 실행합니다.

```bash
ros2 run image_proc rectify_node \
  --ros-args \
  -r image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  -r image_rect:=/camera/image_rect
```

## 설정 파일 위치

- 카메라 보정 파일: `config/main_camera/camera_info.yaml`
- 추가 보정 파일: `config/main_camera/ost.yaml`
- YOLO 가중치 보관 폴더: `config/weights/`

## 참고

- 기본 포트는 `5000`입니다.
- 기본 바인드 주소는 `0.0.0.0`입니다.
- `camera_info_yaml`은 워크스페이스 기준 상대경로로 적어도 되고, 노드가 자동으로 찾아서 읽습니다.
