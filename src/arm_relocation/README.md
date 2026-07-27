# Arm Relocation Package

기존 `arm` 패키지를 수정하지 않고 다층 컨테이너의 방해 컨테이너를 빈 위치로
옮긴 뒤 목표 컨테이너를 최종 위치로 옮기는 테스트 패키지입니다. 기존
`container_pick_place.yaml`의 관찰 자세, MoveIt 안전 설정, `grasp_offset_rpy_deg`,
pick/place 오프셋을 그대로 읽습니다.

## 동작 순서

1. first/second observation pose에서 역할 마커와 보이는 모든 ArUco 마커의
   base-frame pose를 수집합니다.
2. base Z축과 마커 법선의 각도로 윗면(`top`), 옆면(`side`),
   기울어진 면(`tilted`)을 구분합니다.
3. `side` 상태의 `pick_marker`와 XY상 같은 스택에 있는 `top` 마커 중
   base-frame Z가 가장 높은 마커를 선택합니다.
4. 선택한 top 마커 좌표로 이동한 뒤 `grasp_offset_rpy_deg`를 적용합니다.
5. 선택 ID가 `pick_marker`와 다르면 해당 컨테이너를 `empty_marker`로 옮기고
   first/second observation부터 반복합니다.
6. 선택 ID가 `pick_marker`이면 `place_marker`로 옮기고 종료합니다.

접근 후 카메라 재인식은 사용하지 않습니다. 스택 포함 여부는
`stack_xy_tolerance_m`, 높이 하한은 `stack_min_height_above_pick_m`로 조정합니다.

## 빌드 및 실행

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm arm_relocation
source install/setup.bash

ros2 launch arm_relocation container_pick_place_relocation.launch.py \
  pick_marker:=1 place_marker:=2 empty_marker:=3
```

실제 동작 시작:

```bash
ros2 service call /arm/pick_place_relocation std_srvs/srv/Trigger {}
```

기존 호환 서비스 `/arm/pick_and_place`도 같은 relocation 순서를 시작합니다.

## 인터페이스

- 입력: `/arm/gripper_camera/image_raw` (`sensor_msgs/Image`)
- 입력: `/arm/gripper_camera/camera_info` (`sensor_msgs/CameraInfo`)
- 입력: `/joint_states` (`sensor_msgs/JointState`)
- 출력: `/arm/gripper_camera/relocation_detections` (`std_msgs/String`, JSON)
- 출력: `/arm/gripper_camera/relocation_aruco_annotated`
  (`sensor_msgs/Image`)
- 출력: `/arm/container_pick/status` (`std_msgs/String`)
- TF: `arm/gripper_camera_optical_frame -> arm/relocation_marker_<ID>`
- 서비스: `/arm/pick_place_relocation` (`std_srvs/Trigger`)
- 정지 서비스: `/arm/stop_pick` (`std_srvs/Trigger`)

`top_face_max_angle_deg`, `side_face_min_angle_deg`,
`stack_xy_tolerance_m`, `stack_min_height_above_pick_m`,
`max_relocation_cycles`, `empty_stack_step_m`는
`config/container_pick_place_relocation.yaml`에서 조정합니다. 실제 로봇 테스트 전
`execute_motion:=false` 형태의 별도 dry-run launch가 아니므로 작업 반경을 비우고
비상 정지 수단을 준비해야 합니다.
