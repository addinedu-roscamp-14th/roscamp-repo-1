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
    ├── udp/                  # UDP 카메라 입력 → ROS Image
    ├── yolo/                 # YOLO 인식/시각화
    ├── calibration/          # 카메라 픽셀 ↔ SLAM map 캘리브레이션
    ├── central/              # 중앙 좌표 변환/차량 전달용 출력
    ├── drive/                # 차량 주행 제어 
    └── arm/                  # 로봇팔 제어 
```

## Package Summary

| Package | 역할 | 주요 노드/상태 |
| --- | --- | --- |
| `udp` | UDP로 들어오는 탑다운 카메라 영상을 ROS2 Image로 변환 | `udp_camera_node` |
| `yolo` | 카메라 이미지에서 객체/영역을 인식하고 annotated image와 detection JSON 발행 | `yolo_node` |
| `calibration` | `/camera/image_rect` 픽셀 좌표와 SLAM `/map` 좌표를 homography로 캘리브레이션 | `direct_calibrator`, `calibration_verifier` |
| `central` | 캘리브레이션 결과를 사용해 카메라 픽셀 좌표를 `/map` 기준 좌표로 변환하고 JSON/PoseStamped 발행 | `camera_to_map_bridge` |
| `drive` | 차량 주행/Nav2 연동 담당 예정 | 뼈대 패키지 |
| `arm` | 로봇팔/크레인 제어 담당 예정 | 뼈대 패키지 |

## Data Flow

```text
UDP camera
  → udp
  → /camera/image_raw
  → image rectification
  → /camera/image_rect
  → yolo / calibration
  → central
  → /central/target_map_pose
  → 차량 Nav2
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
