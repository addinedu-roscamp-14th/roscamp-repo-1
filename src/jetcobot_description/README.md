# jetcobot_description

JetCobot의 관절, TCP, 그리퍼 카메라 좌표계를 제공하는 최소 ROS 2 description
패키지입니다. 원본 저장소에서 현재 프로젝트에 필요한 URDF와 해당 URDF가 직접
참조하는 메시만 포함합니다.

관절 위치 한계는 현재 설치된 `pymycobot.robot_info`의 `MyCobot280` 값과
동기화합니다: J1 ±168°, J2 ±140°, J3 ±150°, J4 ±150°,
J5 -155~160°, J6 ±180°.

## 구조

```text
jetcobot_description/
├── urdf/jetcobot.urdf  # base_link부터 TCP와 카메라까지의 좌표계
├── meshes/             # URDF가 직접 참조하는 시각화/충돌 STL
├── CMakeLists.txt
├── package.xml
└── LICENSE
```

주요 TF 체인은 다음과 같습니다.

```text
dummy -> base_link -> 1_Link ... 6_Link -> gripper_link -> TCP
                                      -> jetcocam -> ov3360
```

## 빌드

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select jetcobot_description
source install/setup.bash
```

## 확인

```bash
check_urdf src/jetcobot_description/urdf/jetcobot.urdf
```

이 패키지는 노드나 launch 파일을 제공하지 않습니다. 이후 `arm`의
`robot_state_publisher` launch에서 이 URDF를 읽고 실제 `/joint_states`와 결합해
현재 `TCP` 및 `jetcocam` TF를 발행합니다.

URDF의 `camera_joint`와 `TCP_joint` 원점은 원본 모델의 명목값입니다. 실제 장착
위치와 다르면 ArUco 미세 보정을 적용하기 전에 실측값으로 수정해야 합니다.
