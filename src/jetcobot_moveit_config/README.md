# JetCobot MoveIt Config

JetCobot 6축 로봇팔의 MoveIt2 planning group, KDL IK, 관절 제한, SRDF와 controller
연결 설정을 제공합니다. 로봇 모델은 `jetcobot_description`의 기존 URDF를 사용합니다.
MoveIt 위치 한계도 `pymycobot.robot_info`의 `MyCobot280` 범위와 동일하게
유지하여 계획된 관절값이 하드웨어 검증에서 거부되지 않도록 합니다.

## 실기기 planning 실행

fake controller를 실행하지 않고 `move_group`, robot state publisher와 RViz만 실행합니다.
실제 trajectory action은 `arm` 패키지의 `jetcobot_trajectory_bridge`가 제공합니다.
따라서 이 launch만 단독 실행하면 계획은 가능하지만 로봇 trajectory는 실행되지 않습니다.

```bash
ros2 launch jetcobot_moveit_config real_planning.launch.py use_rviz:=true
```

주요 인터페이스:

```text
planning group: arm_group
base link: base_link
end-effector link: TCP
controller action: /arm_group_controller/follow_joint_trajectory
joint states: /joint_states
```
