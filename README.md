# Port-ER Workspace

스마트 항만 관제를 위한 ROS2 워크스페이스입니다. 각 패키지는 역할별로 분리되어 있으며, 자세한 실행 방법은 각 패키지의 `README.md`에 정리합니다.

## Package Structure

```text
poter_ws/
├── config/
│   ├── SLAM/                 # SLAM map yaml/pgm
│   ├── central/              # calibration 결과 yaml
│   ├── main_camera/          # camera_info, calibration yaml
│   └── weights/              # YOLO weight
│
└── src/
    ├── udp/                  # UDP/GStreamer 카메라 입력 fallback
    ├── yolo/                 # YOLO 인식/시각화
    ├── calibration/          # 카메라 픽셀 ↔ SLAM map 캘리브레이션
    ├── central/              # 중앙 좌표 변환/차량 전달용 출력
    ├── drive/                # 차량 주행 제어 
    └── arm/                  # 로봇팔 제어 
```

## Package Summary

| Package | 역할 | 주요 노드/상태 |
| --- | --- | --- |
| `udp` | UDP/GStreamer 방식 카메라 입력 fallback. 현재 기본 카메라 경로는 `v4l2_camera + image_transport compressed` 사용 | `udp_camera_node` |
| `yolo` | 카메라 이미지에서 객체/영역을 인식하고 annotated image와 detection JSON 발행 | `yolo_node` |
| `calibration` | `/camera/image_rect` 픽셀 좌표와 SLAM `/map` 좌표를 homography로 캘리브레이션 | `direct_calibrator`, `calibration_verifier` |
| `central` | 캘리브레이션 결과를 사용해 카메라 픽셀 좌표를 `/map` 기준 좌표로 변환하고 JSON/PoseStamped 발행 | `camera_to_map_bridge` |
| `drive` | 차량 주행/Nav2 연동 담당 예정 | 뼈대 패키지 |
| `arm` | 로봇팔/크레인 제어 담당 예정 | 뼈대 패키지 |

## Data Flow

```text
v4l2_camera
  → /camera/image_raw
  → image_transport compressed
  → /camera/image_raw/compressed
  → image_transport raw relay
  → /camera/image_raw_relay
  → image rectification
  → /camera/image_rect
  → yolo / calibration
  → central
  → /central/target_map_pose
  → 차량 Nav2
```

## Camera Transport

현재 카메라 입력은 UDP 대신 ROS2 compressed image 경로를 기본으로 사용합니다. raw 이미지를 그대로 네트워크로 보내면 프레임이 끊길 수 있으므로, `/camera/image_raw`를 compressed로 압축한 뒤 수신 쪽에서 다시 raw로 복원합니다.

```text
/camera/image_raw
→ /camera/image_raw/compressed
→ /camera/image_raw_relay
→ /camera/image_rect
```

최종적으로 YOLO와 calibration은 왜곡 보정된 이미지를 사용합니다.

```text
/camera/image_rect
```

`udp` 패키지는 ROS compressed 경로가 불안정할 때 사용할 fallback입니다.

### Camera Commands

원본 카메라 실행:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video2 \
  -p image_size:="[640, 480]" \
  -p time_per_frame:="[1, 30]" \
  -p camera_info_url:=file:///home/jio/poter_ws/config/main_camera/camera_info.yaml \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

compressed 발행:

```bash
ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/camera/image_raw \
  -r out:=/camera/image_raw
```

compressed를 raw relay로 복원:

```bash
ros2 run image_transport republish compressed raw \
  --ros-args \
  -r in:=/camera/image_raw \
  -r out:=/camera/image_raw_relay
```

왜곡 보정 이미지 생성:

```bash
ros2 run image_proc rectify_node \
  --ros-args \
  -r image:=/camera/image_raw_relay \
  -r camera_info:=/camera/camera_info \
  -r image_rect:=/camera/image_rect
```

확인:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/image_raw/compressed
ros2 topic hz /camera/image_raw_relay
ros2 topic hz /camera/image_rect
```

## Important Files

```text
config/SLAM/current_map.yaml
config/SLAM/current_map.pgm
config/central/camera_map_calibration.yaml
config/main_camera/camera_info.yaml
config/weights/best.pt
```

## Rule

각 패키지에는 `README.md`를 두고, 해당 패키지의 노드 기능, 실행 방법, 주요 토픽을 간단하게 정리합니다.
