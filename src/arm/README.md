# Arm Homography Pick/Place

실측 캘리브레이션과 JetCobot TF 모델을 패키지 안에서 읽어 ArUco Pick/Place를
수행하는 독립 ROS 2 패키지입니다.

소스 폴더는 `poter_ws/src/arm`이지만 ROS 패키지명은
`arm_pick_place`입니다. 따라서 빌드·실행 명령에는 ROS 패키지명을
사용합니다.

다음 파일을 자체 포함합니다.

- Pick/Place용 dual ArUco 검출기
- JetCobot 6축 kinematic URDF
- USB 카메라 intrinsic calibration
- eye-in-hand static transform calibration
- floor/AGV homography 및 Pick/Place Z calibration
- floor calibration collector와 단일 ArUco 검출기
- 캘리브레이션 작업용 `manual_jog`

MoveIt은 사용하지 않습니다. 관찰 자세는 JetCobot `sync_send_angles`, 모든
Cartesian 동작은 `send_coords`를 한 번 전송한 뒤 `get_coords`로 직접 도착을
확인합니다.

수동 `/start` 호환 경로의 기본값:

- Pick ArUco ID: `2`
- Place ArUco ID: `19` (선박 2번 자리, 사용 가능 범위는 `19..23`)
- Station A 관찰 각도:
  `[-86.39, 57.12, -15.46, -88.15, 7.99, -36.82]`
- Station AGV 관찰 각도:
  `[15.38, 35.59, -2.81, -90.96, 4.13, -37.26]`
- 관찰 순서: `station_agv(3초) -> station_a(5초)`
- Pick nominal tool yaw: `wrap(aruco_yaw - 45도)`
- 같은 station Place yaw: `wrap(aruco_yaw)`
- AGV↔station 교차 Place yaw: `wrap(aruco_yaw - 45도)`
- Safe Z: `0.220 m`
- Pick/Place XY offset: 없음
- Parallel-gripper symmetric yaw selection: 활성화

## 동작 순서

1. `작업 시작` 로그 출력
2. Station AGV 관찰 자세로 이동하고 Pick/Place ID를 탐색
3. 이 pose의 검출은 `agv_0`, `agv_1`만으로 판별
4. Station A 관찰 자세로 이동하고 아직 없는 Pick/Place ID를 탐색
5. 이 pose의 검출은 station `0`, `1`, `2`, `3`층만으로 판별
6. 두 pose 스캔 후 Pick/Place가 모두 판별됐으면 동결된 base 좌표로 작업 시작
7. Pick은 검출 층을 Pick 층으로 사용
8. Place는 검출 층을 support 층, 다음 적재 층을 결과 층으로 사용
9. Pick/Place 모두 실제 검출 마커 층 H로 ArUco XY를 controller XY로 변환
10. Pick과 Place의 safe Z 후보를 각자의 하강 Z 기준으로 독립 생성
11. 그리퍼 열기
12. Pick 마커 상단 XY와 수직 그리퍼 RPY를 한 `send_coords()`로 먼저 시도
13. 도착하면 바로 Pick Z로 하강
14. 도착하지 못한 경우에만 정지 후 현재 RPY를 유지해 마커 상단으로 이동
15. 같은 위치에서 그리퍼를 수직으로 바꾼 뒤 Pick Z로 하강
16. 그리퍼 닫기 후 선택된 Pick safe Z로 상승
17. Place 마커 상단과 목표 RPY를 한 번에 이동
18. Place 결과 층의 `place_z_m`으로 하강
19. 하드웨어 `is_moving()==0`을 연속 3회 확인
20. 그리퍼 열기 후 선택된 Place safe Z로 상승
21. `작업 종료` 로그 출력

Place에서 H와 Z의 층 인덱스는 의도적으로 다릅니다.

```text
Place 마커가 1층 컨테이너 위에서 검출됨
  H       = H[1]          # 실제 검출된 support 마커 평면
  Place Z = place_z_m[2]  # 새 컨테이너가 만들어낼 결과 층
```

따라서 `0층 support -> 결과 1층`, `1층 support -> 결과 2층`,
`2층 support -> 결과 3층`입니다. 현재 4층 교시값이 없으므로 3층 support 위에
놓으려는 작업은 실제 이동 전에 거부합니다.

0층은 바닥에 부착된 support 마커의 층 판별과 XY 변환을 위한 geometry-only
항목입니다. 따라서 0층 H는 사용하지만 0층 Pick/Place Z는 요구하지 않습니다.
0층 마커 위에 놓을 때 하강 높이는 결과 층인 1층의 `place_z_m`을 사용합니다.
반대로 0층 마커를 Pick 대상으로 지정하면 Pick Z가 없으므로 이동 전에 거부합니다.

Pick과 Place에는 XY 오프셋을 적용하지 않습니다. 평행 그리퍼의 대칭성을 이용해
각 nominal yaw에 대해 다음 두 후보를 비교합니다.

```text
Pick candidate 1               = wrap(marker_yaw - 45도)
Same-station Place candidate 1 = wrap(marker_yaw)
Cross-station Place candidate 1 = wrap(marker_yaw - 45도)
candidate 2 = wrap(candidate 1 + 180도)
```

Pick은 현재 로봇 yaw에서 회전량이 작은 후보를 선택합니다. Place는 선택된 Pick
yaw에서 회전량이 작은 후보를 선택합니다. 선택된 yaw는 해당 역할의 접근·하강·상승
전체에 동일하게 사용합니다. 두 후보의 회전량이 정확히 같으면 nominal yaw를
선택합니다. 이 처리는 station과 관계없이 Pick과 Place 모두에 적용됩니다.

마커 검출은 관찰 자세에서만 활성화됩니다. 이후 Pick/Place 도중에는 처음 동결한
좌표만 사용합니다.

## 빌드와 실행

`manual_jog`, 기존 Pick/Place 노드처럼 `/dev/ttyUSB0`을 사용하는 프로그램을
먼저 모두 종료합니다. 이 패키지의 coordinator가 시리얼을 단독 점유합니다.

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select arm_pick_place
source install/setup.bash

ros2 launch arm_pick_place container_pick_place.launch.py \
  serial_port:=/dev/ttyUSB0 \
  video_device:=/dev/video2 \
  marker_size_m:=0.020
```

Floor/AGV 재교시는 [FLOOR_CALIBRATION.md](FLOOR_CALIBRATION.md)를 따릅니다.
교시 결과는 이 패키지의 `config/floor_calibration.yaml`에 저장되며 Pick/Place
launch가 같은 파일을 기본으로 읽습니다.

launch의 기본 경로는 모두 설치된 `arm_pick_place/share` 내부 파일을
사용합니다. 별도 calibration 경로를 시험할 때만 다음 인자를 덮어쓰면 됩니다.

```bash
ros2 launch arm_pick_place container_pick_place.launch.py \
  camera_info_url:=/path/to/gripper_camera_info.yaml \
  handeye_calibration_file:=/path/to/eye_in_hand.calib \
  calibration_file:=/path/to/floor_calibration.yaml
```

동적 작업 시작(중앙 LLM 경로가 사용하는 서비스):

```bash
ros2 service call /arm/pick_place/execute \
  porter_interfaces/srv/ExecutePickPlace \
  "{pick_id: 2, place_id: 19}"
```

`pick_id`와 `place_id`는 매 요청마다 변경할 수 있으며 coordinator는 검출기가
같은 ID로 전환됐다는 응답을 받은 뒤에만 로봇 동작을 시작합니다. launch에서는
두 ID를 지정하지 않습니다.

차량 트레일러 마커는 AMR1(agv1)=10, AMR2(agv2)=9로 고정합니다. 선박 배치
사용 가능한 선박 마커는 19~23이며, 마커 18은 사용하지 않고 트레일러 마커와도
혼용하지 않습니다.

현재 기본 ID로 수동 작업 시작:

```bash
ros2 service call /arm/pick_place/start \
  std_srvs/srv/Trigger "{}"
```

정지:

```bash
ros2 service call /arm/pick_place/stop \
  std_srvs/srv/Trigger "{}"
```

상태 로그:

```bash
ros2 topic echo /arm/pick_place/status
```

작업 상태 호출 (topic):

```bash
ros2 topic echo /arm/pick_place/work_state \
  --qos-durability transient_local \
  --qos-reliability reliable
```

`work_state`는 `std_msgs/msg/String`이며 Reliable + Transient Local QoS로 마지막
상태를 보존합니다. 늦게 연결된 중앙제어도 마지막 상태를 받으려면 subscriber에서
Transient Local durability를 요청해야 합니다. 중앙제어는 자유 형식인 `status`
로그 대신 아래 고정 문자열을 상태 머신 입력으로 사용합니다.

```text
IDLE
WORK_STARTED
SEARCHING
PICK_STARTED
PICK_COMPLETED
PLACE_STARTED
PLACE_COMPLETED
WORK_COMPLETED
STOP_REQUESTED
STOPPED
FAILED
```

정상 작업의 상태 순서는 다음과 같습니다.

```text
WORK_STARTED -> SEARCHING -> PICK_STARTED -> PICK_COMPLETED
-> PLACE_STARTED -> PLACE_COMPLETED -> WORK_COMPLETED
```

`PICK_COMPLETED`는 그리퍼를 닫고 Pick safe Z까지 상승한 뒤 발행합니다.
`PLACE_COMPLETED`는 그리퍼를 열고 Place safe Z까지 상승한 뒤 발행합니다.
오류가 발생하면 즉시 `FAILED`, stop 서비스가 호출되면 `STOP_REQUESTED`와
`STOPPED`를 발행합니다.

검출 영상:

```bash
ros2 run rqt_image_view rqt_image_view \
  /arm/gripper_camera/pick_place_aruco_annotated
```

## Station 추가

[container_pick_place.yaml](config/container_pick_place.yaml)의
`stations_json` 배열의 순서가 실제 관측 순서입니다. 각 pose에는 현재 관절값으로
추정하지 않는 명시적인 `calibration_surface`가 필요합니다.

```yaml
stations_json: >-
  [{"name":"station_agv",
    "calibration_surface":"agv",
    "joint_angles_deg":[15.38,35.59,-2.81,-90.96,4.13,-37.26],
    "timeout_sec":3.0},
   {"name":"station_a",
    "calibration_surface":"station",
    "joint_angles_deg":[-86.39,57.12,-15.46,-88.15,7.99,-36.82],
    "timeout_sec":5.0},
   {"name":"station_b",
    "calibration_surface":"station",
    "joint_angles_deg":[B의 6개 각도],
    "timeout_sec":5.0}]
```

모든 station pose는 작업 시작 시 한 번씩 방문합니다. 앞 station에서 찾은 마커의
base-frame 좌표는 유지하고 이후 pose에서는 누락된 ID만 찾습니다. 전 pose를 확인한
뒤에도 하나라도 없으면 로봇을 정지하고 실패 로그를 출력합니다. 따라서 Pick과
Place가 서로 다른 station에서 발견되어도 스캔 완료 후 두 위치 사이를 이동합니다.
AGV의 `agv_0`, `agv_1` H가 아직 모두 교시되지 않았다면 AGV pose에는 이동하지만
그 surface의 분류는 안전하게 건너뜁니다. 두 AGV 레벨을 fit/save한 다음 실행하면
코드나 pose 설정을 다시 바꾸지 않고 AGV 검출이 활성화됩니다.

## 적용된 안전 검사

- 이동 중 ArUco 검출 비활성화
- 마커 TF 최소 7개 샘플 안정화
- TF XYZ 층 평면 오차 및 차순위 층과의 간격 검사
- 캘리브레이션 convex hull에서 25 mm 이상 벗어난 H 외삽 거부
- 변환 XY가 교시한 command 영역을 벗어나면 거부
- 관찰 관절과 모든 `get_coords()` 도착 오차 검사
- service stop 시 detector 비활성화와 `robot.stop()` 호출

현재 기본 Cartesian 위치 허용오차는 전체 Pick/Place 흐름을 확인하기 위한 임시
값인 `20 mm`입니다. 기본 동작이 확인되면 다시 낮춰야 합니다.

`sync_send_coords()`는 하드웨어의 엄격한 도착 판정을 만족하지 못하면 로봇이 이미
멈춘 뒤에도 전체 20초를 기다립니다. 이 패키지는 이를 사용하지 않고 0.1초마다
`get_coords()`를 읽습니다. 위치 20 mm/각도 6도 이내가 연속 3회 확인되면 즉시
다음 단계로 진행하며, 20초 동안 들어오지 못할 때만 실패합니다.

Place 하강 직후에는 위 좌표 허용 범위만으로 그리퍼를 열지 않습니다. 하드웨어의
`is_moving()`이 정지 상태 `0`을 0.1초 간격으로 연속 3회 반환한 뒤에만 Place
그리퍼를 엽니다. Pick 시작 시의 그리퍼 개방에는 이 추가 대기를 적용하지 않습니다.

XY 오프셋은 적용하지 않고, 위의 최소회전 대칭 yaw branch를 사용합니다. 이
JetCobot 펌웨어에서 응답하지 않는 `solve_inv_kinematics()`는 더 이상 호출하지
않습니다. Pick과 Place 각각
220, 210, 200, 190 mm 순으로 후보를 생성하고, 각 역할의 하강 Z보다 최소 20 mm
높은 후보 중 가장 낮은 높이를 사용합니다. 따라서 높은 Place Z가 Pick safe Z를
제한하지 않습니다.

예를 들어 1층 Pick과 결과 3층 Place라면 기본 설정의 후보는 다음과 같습니다.

```text
Pick  safe Z = 220, 210, 200, 190 mm
Place safe Z = 220, 210 mm
```

컨트롤러 내부 IK 결과는 `send_coords()` 호출에서 별도로 반환되지 않으므로 실제
도착 여부로 성공을 판정합니다. Pick 결합 접근이 `get_coords()` 검증을 통과하면
분할 동작은 전혀 실행하지 않고 바로 하강합니다. 명령이 거부되거나 제한 시간 내
도착하지 못한 경우에만 로봇을 정지하고 다음 분할 접근을 실행합니다.

```text
기본 Pick:
  [Pick XY, safe Z, 목표 수직 RPY] -> Z 하강

기본 Pick 접근 실패 시에만:
  [Pick XY, safe Z, 현재 RPY] -> 같은 위치에서 목표 수직 RPY -> Z 하강
```

Place는 Pick 폴백 여부와 관계없이 기존 결합 자세 이동을 유지합니다. 분할 접근도
각 단계마다 동일한 `get_coords()` 도착 검증을 수행하며, 어느 단계든 실패하면
작업을 정지합니다.

## 현재 캘리브레이션 주의점

1층 H는 `5/6` inlier이지만 2층과 3층은 `4/6` inlier입니다. 4점 H의 0 mm
학습 오차는 네 점을 정확히 통과한 결과일 뿐, 별도 위치에서의 실제 정확도를
의미하지 않습니다. 그래서 이 코드에서는 교시 영역 밖 H 외삽을 막았습니다.

첫 실기체 시험은 물체 없이 실행하거나 Pick/Place Z보다 충분히 높은 임시 Z로
검증하는 것을 권장합니다. 실제 파지 시험 전에는 비상정지 가능한 상태에서 속도를
낮추고, 출력되는 층·보정 XY·yaw를 확인해야 합니다.
