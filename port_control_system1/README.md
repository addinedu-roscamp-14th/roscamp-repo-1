# Port Control System

CustomTkinter 기반 중앙관제 대시보드입니다. 기존 파일 기반 화물·위치 관리 기능을
유지하면서 ROS 2 중앙관제 토픽과 연결합니다.

## 연결 기능

| 방향 | 인터페이스 | 기능 |
| --- | --- | --- |
| ROS → UI | `/battery/percent` (`std_msgs/Float32`) | 차량 1 배터리 표시 |
| ROS → UI | `/battery/voltage` (`std_msgs/Float32`) | 차량 전압 수신 |
| ROS → UI | `/odom` (`nav_msgs/Odometry`) | 차량 1 odom 위치·방향 표시 |
| ROS → UI | `/arm/container_pick/status` | 1번 로봇팔 상태 수신 |
| ROS → UI | `/arm2/container_pick/status` | 2번 로봇팔 상태 수신 |
| UI → ROS | `/central/target_map_waypoints` (`nav_msgs/Path`) | 위치·경유지 주행 명령 |
| UI → ROS | `/central/target_map_pose` (`geometry_msgs/PoseStamped`) | 단일 목표용 인터페이스 |
| UI → ROS | `/cmd_vel` (`geometry_msgs/Twist`) | 수동 운전과 비상 정지 |
| UI → ROS | `/arm/pick_container`, `/arm/stack_container` | 1번 로봇팔 파지·적재 |
| UI → ROS | `/arm2/pick_container`, `/arm2/stack_container` | 2번 로봇팔 파지·적재 |

비상 정지가 활성화되면 `/cmd_vel`에 정지 명령을 100Hz로 계속 발행합니다.
관제 화면에서 명시적으로 **비상 정지 해제**를 누르기 전에는 수동 속도 명령도
차단됩니다.

## 실행

차량과 관제 노트북의 네트워크, `ROS_DOMAIN_ID`, DDS 설정을 먼저 맞춥니다.

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=<차량과_같은_번호>
```

최초 한 번 대시보드 Python 의존성을 설치합니다.

```bash
cd ~/poter_ws
python3 -m venv .venv
.venv/bin/pip install -r port_control_system1/requirements.txt
```

차량에서 Nav2가 실행 중인 상태에서 관제 노트북의 waypoint 브리지를 실행합니다.

```bash
ros2 launch drive target_waypoints_nav.launch.xml \
  path_topic:=/central/target_map_waypoints
```

이 launch는 Nav2를 시작하지 않습니다. 차량 또는 관제 노트북에서 Nav2가 먼저
실행되어 `navigate_through_poses` 액션 서버가 준비되어 있어야 합니다.

영상 API가 필요하면 별도 터미널에서 실행합니다.

```bash
ros2 launch dashboard dashboard_stream.launch.py
```

대시보드를 실행합니다.

```bash
cd ~/poter_ws/port_control_system1
../.venv/bin/python agv_control_center.py
```

상단에 `ROS 연결됨`이 표시되는지 확인합니다. 통합 관제 화면의 차량 탭에서
배터리와 odom 값이 갱신되어야 합니다.

비상상황 대처 화면에서 `JetCobot #01`은 `/arm`, `JetCobot #02`는 `/arm2`
서비스를 사용합니다. 각 로봇팔의 MoveIt 파지 launch가 먼저 실행되어 해당 서비스가
준비되어 있어야 합니다.

## 위치 명령

`location_marks_verified.json`에서 `map_meters`가 등록된 위치만 차량 목표로 보낼 수
있습니다. 대시보드의 **명령** 창에서 다음처럼 입력합니다.

```text
항구로 이동
창고 회차지점을 거쳐 창고 하역장으로 이동
```

명령은 `/central/target_map_waypoints`에 `map` 좌표계의 `nav_msgs/Path`로
발행됩니다. 현재 등록 위치에는 별도 헤딩값이 없으므로 각 waypoint의 기본 yaw는
0도입니다.

## VLM 영상 좌표 주행

명령 창은 최신 탑다운 프레임을 Ollama 비전 모델에 함께 전달할 수 있습니다. 모델이
`pixel_navigation` 액션의 목표 픽셀과 헤딩 픽셀을 반환하면 대시보드가 중앙제어
API로 전송합니다.

검출 객체를 지칭한 명령은 `visual_navigation`을 사용합니다. VLM에는 YOLO
어노테이션 JPEG와 클래스, confidence, bbox, 중심점, heading JSON이 함께 전달됩니다.
VLM은 `detection_index`와 `approach_side`만 선택하고, 실제 목표점은 코드가 선택된
bbox 바깥 50픽셀에서 계산합니다. 직접 픽셀 목표는 영상 범위를 벗어나거나 YOLO
bbox 안에 있으면 전송하지 않습니다.

예외적으로 `B-1`만 항구 상차·하차 전용 주차 구역으로 처리합니다. `B-1` 목적지
명령은 bbox 중심을 목표로 하고 세그멘테이션 `heading_deg`를 주차 방향으로
사용합니다. 중심에 다른 차량의 검출 bbox가 겹치면 점유된 구역으로 판단해 명령을
거부합니다. `A-1`, `A-2`, `A-3`과 차량/트레일러 클래스에는 이 주차 규칙을
적용하지 않습니다. 중앙 ROS 브리지는 B-1 중심을 map 좌표로 변환한 후 탑다운
카메라 영상의 왼쪽 방향으로 `0.15m`, 화면 아래쪽으로 `0.03m` 이동한 지점을
최종 주차 위치로 사용합니다.

```bash
export OLLAMA_HOST=http://127.0.0.1:11434
export LOCAL_LLM_MODEL=gemma4:31b
export PORT_CONTROL_API_URL=http://<중앙제어_IP>:8100
export PORT_CONTROL_API_TOKEN='porter1234'
```

테스트 명령:

```text
현재 영상에서 중앙의 빈 공간으로 이동하고 오른쪽을 바라봐
```

VLM이 이미지를 받지 못했거나 안전한 목표를 특정하지 못하면 좌표를 추측하지 않고
명령을 거부하도록 프롬프트에 지정되어 있습니다. 최종 픽셀 범위는 중앙제어 API가
다시 검증하고 실제 장애물과 경로 가능 여부는 Nav2가 판단합니다.

## 영상 설정

`stream_config.json`에서 기존 dashboard API 주소를 지정합니다.

```json
{
  "cctv_url": "http://127.0.0.1:8000/video",
  "slam_url": "http://127.0.0.1:8000/slam/video"
}
```

API 서버가 다른 노트북에서 실행 중이면 `127.0.0.1`을 해당 노트북 IP로
변경합니다.

## 토픽 확인

```bash
ros2 topic hz /battery/percent
ros2 topic hz /odom
ros2 topic echo /central/target_map_waypoints
ros2 topic echo /cmd_vel
ros2 topic info /central/target_map_waypoints -v
```

## 환경변수

필요한 경우 앱 실행 전에 인터페이스 토픽을 변경할 수 있습니다.

```bash
export PORT_CONTROL_CMD_VEL_TOPIC=/cmd_vel
export PORT_CONTROL_TARGET_POSE_TOPIC=/central/target_map_pose
export PORT_CONTROL_TARGET_WAYPOINTS_TOPIC=/central/target_map_waypoints
```

현재 ROS 브리지는 앱을 실행한 `ROS_DOMAIN_ID` 한 개에 연결됩니다. 서로 다른
ROS 도메인의 차량 두 대를 동시에 제어하려면 도메인별 게이트웨이 프로세스를 두고
차량 토픽을 중앙 도메인에 namespace로 중계해야 합니다.
