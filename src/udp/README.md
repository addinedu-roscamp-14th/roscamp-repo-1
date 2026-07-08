# UDP 패키지

이 패키지는 UDP로 들어오는 카메라 영상을 ROS2 이미지 토픽으로 바꾸는 역할을 합니다.

## 실행 순서

### 1. 카메라 영상 송신
아래 명령으로 카메라 영상을 UDP로 전송합니다.

```bash
gst-launch-1.0 -v v4l2src device=/dev/video2 ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegparse ! rtpjpegpay ! udpsink host=127.0.0.1 port=5000
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
