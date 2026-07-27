# dashboard

YOLO annotated 영상과 지도·차량·LiDAR 합성 화면을 HTTP MJPEG로 제공하는 ROS 2
패키지입니다. 기존 카메라, YOLO와 SLAM 실행 방법 및 토픽은 변경하지 않습니다.

- `/central/yolo/image_annotated`: 탑다운 YOLO 영상
- `/map`: SLAM 또는 Nav2의 실시간 `OccupancyGrid`
- `map -> base_footprint`: SLAM 지도 위 차량 위치와 헤딩
- `/scan`: SLAM 지도 위 실시간 LiDAR 스캔 포인트

## 설치

```bash
sudo apt update
sudo apt install python3-fastapi python3-uvicorn
```

시스템 패키지를 설치할 수 없으면 사용자 환경에 설치합니다.

```bash
/usr/bin/python3 -m pip install --user --break-system-packages fastapi uvicorn
```

```bash
cd ~/poter_ws
colcon build --symlink-install --packages-select dashboard
source install/setup.bash
```

## 실행

기존 카메라와 YOLO를 먼저 실행합니다. 차량의 SLAM Toolbox가 `/map`, `/scan`, `/odom`,
`/tf`, `/tf_static`을 발행하는 상태에서 노트북의 dashboard API 서버를 실행합니다.

```bash
ros2 launch slam slam_bringup.launch.xml
```

이 명령은 노트북에서 SLAM Toolbox를 실행하지 않고 `dashboard_stream_node`만
실행합니다. 다음 dashboard 전용 명령을 동시에 실행하면 TCP 8000번 포트 충돌이
발생합니다.

dashboard API만 단독으로 테스트할 때는 다음 명령을 대신 사용합니다.

```bash
ros2 launch dashboard dashboard_stream.launch.py
```

설정 파일 없이 dashboard 노드만 직접 테스트할 수도 있습니다.

```bash
ros2 run dashboard dashboard_stream_node
```

FPS와 JPEG 품질 변경:

```bash
ros2 run dashboard dashboard_stream_node --ros-args \
  -p web_fps:=20.0 \
  -p jpeg_quality:=70
```

## API

```text
http://localhost:8000/video
http://localhost:8000/health
http://localhost:8000/detections
http://localhost:8000/slam/view
http://localhost:8000/slam/video
http://localhost:8000/slam/map.png
http://localhost:8000/slam/map/metadata
http://localhost:8000/slam/health
```

같은 네트워크의 다른 장치에서는 노트북 IP를 사용합니다.

```text
http://<노트북-IP>:8000/video
http://<노트북-IP>:8000/slam/view
http://<노트북-IP>:8000/slam/video
http://<노트북-IP>:8000/slam/map.png
```

`/slam/video`는 지도 위에 차량 pose와 LiDAR를 합성해 MJPEG로 전송합니다. 브라우저에서
확인할 때는 `/slam/view`를 열고, HTTP 클라이언트에서 스트림을 직접 받을 때는
`/slam/video`를 사용합니다.

방화벽을 사용하는 경우 포트를 허용합니다.

```bash
sudo ufw allow 8000/tcp
```

## 파라미터

| 이름 | 기본값 | 기능 |
| --- | --- | --- |
| `input_topic` | `/central/yolo/image_annotated` | 입력 ROS 이미지 |
| `detection_topic` | `/central/yolo/detections` | 최신 YOLO 검출 JSON |
| `host` | `0.0.0.0` | API bind 주소 |
| `port` | `8000` | API TCP 포트 |
| `web_fps` | `15.0` | JPEG 인코딩 및 웹 전송 최대 FPS |
| `jpeg_quality` | `70` | JPEG 품질 1~100 |
| `output_width` | `640` | 웹 영상 너비, `0`이면 원본 유지 |
| `output_height` | `480` | 웹 영상 높이, `0`이면 원본 유지 |
| `stale_timeout_sec` | `2.0` | `/health`에서 stale로 판단할 시간 |
| `slam_map_topic` | `/map` | 실시간 SLAM 지도 토픽 |
| `slam_base_frame` | `base_footprint` | 지도에 표시할 차량 TF frame |
| `slam_scan_topic` | `/scan` | 지도에 표시할 LiDAR 토픽 |
| `slam_scan_max_age_sec` | `0.5` | 이 시간보다 오래된 스캔은 표시하지 않음 |
| `slam_live_fps` | `15.0` | SLAM 합성 MJPEG 전송 FPS |
| `slam_output_width` | `720` | SLAM 웹 영상 너비 |
| `slam_output_height` | `720` | SLAM 웹 영상 높이 |

기본값은 [dashboard.yaml](../../config/dashboard/dashboard.yaml)에서 관리합니다.

## 확인

```bash
ros2 topic hz /central/yolo/image_annotated
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map base_footprint
curl http://localhost:8000/health
curl http://localhost:8000/detections
curl http://localhost:8000/slam/health
```

`/health`가 `waiting_for_frame`이면 YOLO 출력 토픽을 확인합니다. `stale`이면 마지막
JPEG가 `stale_timeout_sec`보다 오래된 상태입니다.

`/detections`는 최신 YOLO JSON 하나만 반환합니다. 응답의 `status`가 `ok`이고
`age_sec`가 작아야 VLM 객체 접근에 사용됩니다. 과거 검출 결과는 큐에 쌓지 않습니다.

`/slam/health`가 `waiting_for_map`이면 `/map` 발행 여부를 확인합니다.
`robot_visible`이 `false`이면 `tf_error`를 확인하고 `map -> base_footprint` TF 연결을
점검합니다. 지도 자체는 TF가 없어도 전송됩니다.

`scan_points_visible`이 `0`이면 `scan_age_sec`와 `scan_tf_error`를 확인하고
`/scan`의 frame에서 `map`까지 TF가 연결되는지 점검합니다. LiDAR와 차량 좌표는
`/slam/map.png`의 720x720 픽셀 좌표로 제공되므로 웹 Canvas에 바로 그릴 수 있습니다.
