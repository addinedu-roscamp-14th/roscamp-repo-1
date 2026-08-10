# Arm Floor Calibration

통합된 `arm_pick_place` 패키지에서 다음 값을 실기체로 교시하는
캘리브레이션 기능입니다.

- station 0/1/2/3층과 AGV 0/1층의 `검출된 ArUco base XY -> 실제 controller XY` homography
- station 1/2/3층과 AGV 1층의 Pick 하강 절대 Z
- station 1/2/3층과 AGV 1층의 Place 하강 절대 Z
- 이후 station/층 판별 범위를 정할 때 사용할 원본 marker XYZ/yaw 샘플

이 노드는 로봇에 이동 명령을 보내지 않습니다. 모든 이동은 기존 `manual_jog`에서
저속으로 수행합니다. 기존 결과 YAML이 있으면 시작할 때 모든 원본 샘플과 fit을
불러오고, 새 데이터를 누적해 원자적으로 다시 저장합니다.

## 왜 한 층에 여러 샘플이 필요한가

Homography에는 같은 층에서 최소 4개의 서로 다른 대응점이 필요합니다. 컨테이너를
작업 가능 범위의 좌/우/앞/뒤로 옮겨 마커 위치가 넓게 퍼지도록 수집해야 합니다.
정확도 평가와 오검출 제거를 위해 층마다 6~10쌍을 권장합니다. 한 줄 위에 몰린
샘플은 자동으로 거부됩니다.

각 XY 대응점은 두 단계로 수집합니다.

1. 관찰 자세에서 ArUco TF를 안정화하고 `capture_marker`로 원본 좌표를 동결합니다.
2. 카메라에서 마커가 사라져도 괜찮습니다. `manual_jog`로 그리퍼 기준점을 실제
   마커 중심에 맞춘 뒤 `capture_xy_pair`를 호출합니다.

## 빌드와 실행

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm_pick_place
source install/setup.bash
```

터미널 1은 로봇 시리얼을 단독 점유하며 조그와 관절 TF 값을 제공합니다.

```bash
ros2 run arm_pick_place manual_jog --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p speed:=5
```

터미널 2는 카메라, 저장된 hand-eye TF, ArUco detector와 수집 노드를 실행합니다.
`manual_jog`가 이미 시리얼을 사용하므로 이 launch는 시리얼 포트를 열지 않습니다.

```bash
ros2 launch arm_pick_place floor_calibration.launch.py \
  video_device:=/dev/video2 \
  target_id:=9 \
  marker_size_m:=0.020
```

검출 영상은 다음 토픽에서 확인합니다.

```bash
ros2 run rqt_image_view rqt_image_view \
  /arm/gripper_camera/aruco_annotated
```

## XY 한 쌍 수집

먼저 현재 층과 station 이름을 지정합니다. station은 H 계산 그룹을 나누지는 않고,
나중에 TF 범위를 정하기 위한 라벨로 저장됩니다.

```bash
ros2 param set /arm/floor_calibrator active_surface floor
ros2 param set /arm/floor_calibrator active_floor 1
ros2 param set /arm/floor_calibrator active_station station_a
```

`active_surface=floor`에서는 기존 방식 그대로 `active_floor=0..3`을 사용합니다.
바닥 마커용 0층은 XY/marker plane/H만 수집하며 자체 Pick/Place Z는 없습니다.

AGV도 마커 높이에 따라 두 그룹으로 나눕니다.

- `agv_0`: 빈 화물칸에 부착된 지지면 마커. XY/H만 교시합니다.
- `agv_1`: AGV에 실린 컨테이너 마커. XY/H와 Pick/Place Z를 교시합니다.

빈 AGV 화물칸 마커를 교시하려면 다음처럼 선택합니다.

```bash
ros2 param set /arm/floor_calibrator active_surface agv
ros2 param set /arm/floor_calibrator active_floor 0
ros2 param set /arm/floor_calibrator active_station station_agv
```

적재된 컨테이너 마커와 AGV Pick/Place Z를 교시할 때는 `active_floor`를 1로
변경합니다.

```bash
ros2 param set /arm/floor_calibrator active_floor 1
```

관찰 자세에서 마커가 안정적으로 보일 때 원본 marker TF를 동결합니다.

```bash
ros2 service call /arm/floor_calibration/capture_marker \
  std_srvs/srv/Trigger "{}"
```

이후 `manual_jog`로 그리퍼 기준점을 실제 마커 중심에 맞추고 대응점을 저장합니다.

```bash
ros2 service call /arm/floor_calibration/capture_xy_pair \
  std_srvs/srv/Trigger "{}"
```

컨테이너 위치를 바꾼 뒤 위 과정을 같은 층에서 최소 4회, 권장 6~10회 반복합니다.
0층, 2층, 3층도 `active_floor`를 변경해 같은 방식으로 수집합니다. AGV는
`active_surface=agv`에서 `active_floor=0`과 `1`을 각각 수집합니다.

잘못 저장한 마지막 XY 한 쌍은 다음 서비스로 제거합니다.

```bash
ros2 service call /arm/floor_calibration/undo_last_xy \
  std_srvs/srv/Trigger "{}"
```

현재 선택한 level과 station의 XY 및 Pick/Place Z 데이터를 전부 삭제하려면 다음
서비스를 사용합니다.

```bash
ros2 service call /arm/floor_calibration/delete_active_group \
  std_srvs/srv/Trigger "{}"
```

예를 들어 `active_surface=floor`, `active_floor=2`,
`active_station=station_b`이면 `station_b + 2층` 데이터만 삭제합니다. 다른
station 데이터는 유지됩니다. 삭제된 샘플이 들어갔던 H는 무효화되므로 이후
`fit_and_save`를 다시 호출해야 합니다.

## 층별 Pick/Place Z 교시

그리퍼를 실제 Pick 하강 완료 높이까지 천천히 내린 뒤 저장합니다.

```bash
ros2 service call /arm/floor_calibration/capture_pick_z \
  std_srvs/srv/Trigger "{}"
```

Place 하강 완료 높이에서는 별도 서비스로 저장합니다.

```bash
ros2 service call /arm/floor_calibration/capture_place_z \
  std_srvs/srv/Trigger "{}"
```

각 높이는 2~3회 반복 교시하면 평균과 표준편차가 YAML에 함께 저장됩니다. 물체를
누르는 위치가 아니라 실제 그리퍼가 닫히거나 열리는 안전한 Z를 교시해야 합니다.
AGV 1층도 Pick/Place Z를 모두 교시할 수 있습니다. station 0층과 AGV 0층은
Place support geometry이므로 두 Z 서비스 모두 명시적으로 거부합니다. station
바닥에 놓을 때는 station 1층 `place_z_m`, 빈 AGV에 놓을 때는 AGV 1층
`place_z_m`을 사용합니다.

## H 계산과 결과 확인

```bash
ros2 service call /arm/floor_calibration/fit_and_save \
  std_srvs/srv/Trigger "{}"

ros2 service call /arm/floor_calibration/show_status \
  std_srvs/srv/Trigger "{}"
```

기본 결과 파일은 설치된 패키지의 `config/floor_calibration.yaml`입니다.
`--symlink-install` 빌드에서는 소스 패키지 내부의 같은 파일에 바로 반영됩니다.
매 캡처 직후 원본 샘플이 저장되므로 H 계산 전에 노드가 종료돼도 결과는 남습니다. 노드를 다시
실행하면 이 파일을 자동으로 불러오므로 station B/C나 AGV 데이터를 나중에
추가해도 기존 station A 데이터가 유지됩니다. 기존 YAML이 손상되었거나 형식이
맞지 않으면 안전을 위해 노드 시작을 거부하며 파일을 덮어쓰지 않습니다.
이전 버전에서 저장한 단일 `agv` 그룹은 시작할 때 `agv_1`로 호환 로드됩니다.

`homography_metrics.rmse_m`와 `max_error_m`는 교시점에서 측정한 오차입니다.
최소 4점만 사용하면 오차가 거의 0으로 나와도 일반화 정확도를 뜻하지 않습니다.
따라서 실제 사용 전에는 수집에 쓰지 않은 컨테이너 위치에서 별도로 검증해야
합니다.

## 제공 서비스와 TF

입력 TF:

- `arm/base_link -> arm/target_marker`
- `arm/base_link -> arm/controller_coords`

서비스:

- `/arm/floor_calibration/capture_marker`
- `/arm/floor_calibration/capture_xy_pair`
- `/arm/floor_calibration/capture_pick_z`
- `/arm/floor_calibration/capture_place_z`
- `/arm/floor_calibration/fit_and_save`
- `/arm/floor_calibration/undo_last_xy`
- `/arm/floor_calibration/delete_active_group`
- `/arm/floor_calibration/show_status`

## 안전 주의

- `manual_jog` 외에 `/dev/ttyUSB0`을 여는 노드를 동시에 실행하지 않습니다.
- 처음에는 속도 5 이하, 충분한 `safe_z`, 비상정지 가능한 상태에서 교시합니다.
- Z 교시는 한 번에 크게 내리지 말고 1 mm 단위로 접근합니다.
- 생성된 H와 Z는 자동 동작 코드에 연결하기 전에 미사용 위치에서 검증합니다.
