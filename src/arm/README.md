# arm

JetCobot 카메라 화면을 클릭해 컨테이너를 집고 놓는 ROS 2 Python 패키지입니다.

## 파일 구조

```text
arm/
├── arm/
│   ├── __init__.py
│   ├── main.py              # ROS 2 노드와 OpenCV 화면
│   ├── _config.py           # 장치, 좌표 보정, 높이와 자세 설정
│   ├── _vision_utils.py     # 컨테이너 검출과 픽셀→로봇 좌표 변환
│   ├── _angle_utils.py      # 각도 정규화
│   ├── _robot_utils.py      # JetCobot 이동과 그리퍼 제어
│   └── _container_task.py   # Pick/Place 작업 순서
├── resource/arm
├── package.xml
├── setup.cfg
└── setup.py
```

`main.py`만 복사하면 내부 모듈 import가 실패하므로 위 Python 파일들을 함께 관리해야 합니다.

## 최초 의존성 설치

ROS 2가 사용하는 시스템 Python에 `pymycobot`이 필요합니다.

```bash
/usr/bin/python3 -m pip install --user --break-system-packages pymycobot
```

ROS 의존성은 워크스페이스 루트에서 설치합니다.

```bash
cd ~/poter_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 빌드

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm
source install/setup.bash
```

Python 파일을 수정한 뒤에는 새 터미널에서 `source install/setup.bash`를 다시 실행합니다. `--symlink-install` 빌드에서는 일반적인 Python 코드 수정이 바로 반영되지만 `setup.py`나 `package.xml`을 바꾸면 다시 빌드해야 합니다.

## 실행

```bash
ros2 run arm click_pick_place --ros-args \
  -p camera_path:=/dev/video2 \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=1000000
```

조작 방법:

- 좌클릭: 클릭한 컨테이너 Pick
- 우클릭: 들고 있는 컨테이너 Place
- `Q`: 현재 카메라 화면 다시 촬영
- `ESC`: 종료

Pick/Place 성공 후에는 자동으로 새 사진을 촬영합니다.

## 주요 설정

실물 환경에 맞춰 `arm/_config.py`를 조정합니다.

- `CAMERA`, `PORT`, `BAUD`: 기본 장치 설정
- `H`: 카메라 픽셀과 로봇 XY 사이 homography
- `ROBOT_X_OFFSET`, `ROBOT_Y_OFFSET`: mm 단위 좌표 미세 보정
- `SAFE_Z`, `PICK_Z1`, `PLACE_Z1`: 이동 및 작업 높이
- `VERTICAL_RX`, `VERTICAL_RY`, `VERTICAL_RZ`: 그리퍼 수직 자세
- `J6_SIGN`: 카메라 각도에 따른 J6 보정 방향

## 주요 ROS 인터페이스

- 노드: `/jetcobot_click_control`
- 발행 토픽: `/joint_states` (`sensor_msgs/msg/JointState`)
- 파라미터: `camera_path`, `serial_port`, `baud_rate`, `window_name`

이 노드는 JetCobot 시리얼 포트를 직접 사용하므로 같은 포트에 접근하는 `joint_control` 노드와 동시에 실행하지 않습니다.

## GitHub에 올리기

`~/poter_ws`가 Git 저장소인 상태에서 다음 순서로 진행합니다.

```bash
cd ~/poter_ws
git status
git add src/arm
git commit -m "Add JetCobot click pick and place node"
git push
```

처음 올리기 전에 `git status`로 포함될 파일을 확인하고 `build/`, `install/`, `log/`, `__pycache__/`는 커밋하지 않습니다. GitHub에는 소스인 `src/arm`만 올라가면 됩니다.

## 주의사항

- 카메라와 로봇 위치가 바뀌면 homography를 다시 보정합니다.
- 변환된 X/Y/Z가 JetCobot 작업 범위를 벗어나면 이동하지 않습니다.
- 수직 자세는 실행 후 실제 RPY를 확인하며, 오차가 3도보다 크면 최대 3회 재시도합니다.
