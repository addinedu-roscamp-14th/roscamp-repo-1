# Arm Relocation 2

실제 적재 상태를 배열로 선언하고, 실행 전에 모든 컨테이너 이동 순서를 계산하는
ROS 2 패키지입니다. 가려질 수 있는 원본 스택의 바닥/옆면 마커를 반복해서 찾지
않습니다.

## 스택 선언

스택은 `[바닥 마커, 1층 컨테이너, 2층 컨테이너, ..., 최상층 컨테이너]`
순서입니다.

```yaml
source_stack: "[1, 2, 3, 4]"
empty_stack: "[5]"
target_container_id: 2
final_place_marker_id: 6
```

위 선언의 의미:

- `1`: 원본 스택 바닥 마커
- `2`: 1층 컨테이너
- `3`: 2층 컨테이너
- `4`: 3층 컨테이너
- `5`: Empty 스택 바닥 마커
- `6`: 목표 컨테이너를 놓을 최종 Place 마커

계산되는 계획:

```text
4 -> 5 (Empty 바닥)
3 -> 4 (Empty에 놓인 컨테이너 4의 윗면)
2 -> 6 (최종 Place)
```

`empty_stack`에 이미 컨테이너가 있다면 함께 선언할 수 있습니다. 예를 들어
`"[5, 9]"`이면 첫 방해 컨테이너는 ID 9 위에 놓습니다.

## 동작 방식

1. 서비스 요청을 받으면 전체 이동 계획을 확정하고 로그로 출력합니다.
2. 각 이동에서 first/second observation pose로 이동합니다.
3. 현재 계획에 필요한 `pick_id`와 `place_target_id`의 top 마커만 안정화합니다.
4. 기존 `config/arm/container_pick_place.yaml`의 grasp/place offset을 적용합니다.
5. 계획된 Pick/Place를 실행한 뒤 다음 계획으로 진행합니다.

카메라 스트림은 계속 실행되지만 ArUco 검출은 로봇 이동 중 비활성화됩니다.
관찰 자세 도착 및 settle 이후의 capture 구간에서만 검출, TF, metadata를
발행합니다.

## 빌드 및 실행

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm arm_relocation2
source install/setup.bash

ros2 launch arm_relocation2 container_pick_place_relocation2.launch.py \
  source_stack:='[1,2,3,4]' \
  empty_stack:='[5]' \
  pick_marker:=2 \
  place_marker:=6 \
  video_device:=/dev/video2 \
  serial_port:=/dev/ttyUSB0
```

실행:

```bash
ros2 service call /arm/pick_place_relocation2 \
  std_srvs/srv/Trigger "{}"
```

## 설정

공용 Pick/Place 보정값은 다음 파일에서 읽습니다.

```text
config/arm/container_pick_place.yaml
```

스택 선언, 검출 시간 및 relocation 전용 속도는 다음 파일에 있습니다.

```text
src/arm_relocation2/config/container_pick_place_relocation2.yaml
```

launch 인수의 `source_stack`, `empty_stack`, `pick_marker`,
`place_marker` 값은 YAML의 동일 역할 값을 덮어씁니다.
