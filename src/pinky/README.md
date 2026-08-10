# Pinky

Pinky 하드웨어, LiDAR, URDF 및 차량 내부 속도 안전 게이트를 실행합니다.
기존 `bringup_robot.launch.xml`은 단일 차량 호환용으로 유지됩니다.

## 다중 차량 하드웨어

차량 컴퓨터에서 해당 차량 ID를 지정합니다.

```bash
ros2 launch pinky multi_vehicle_bringup.launch.py vehicle_id:=agv1
```

```bash
ros2 launch pinky multi_vehicle_bringup.launch.py vehicle_id:=agv2
```

주요 인터페이스:

```text
/<vehicle_id>/cmd_vel_safe_input  geometry_msgs/Twist
/<vehicle_id>/cmd_vel_manual      geometry_msgs/Twist
/<vehicle_id>/cmd_vel             geometry_msgs/Twist
/<vehicle_id>/odom                nav_msgs/Odometry
/<vehicle_id>/scan                sensor_msgs/LaserScan
/<vehicle_id>/scan_raw            sensor_msgs/LaserScan
/<vehicle_id>/joint_states        sensor_msgs/JointState
/<vehicle_id>/battery/*           std_msgs/Float32
/<vehicle_id>/emergency_stop      std_srvs/SetBool
/<vehicle_id>/safety_hold         std_srvs/SetBool
```

`cmd_vel_safety_gate`는 Nav2/수동 속도를 `cmd_vel`로 전달합니다. 비상정지,
자동 충돌 `safety_hold`가 활성화되거나 입력이 0.5초 이상 끊기면 100Hz로 정지
속도를 유지합니다. `safety_hold`는 Nav2 목표를 취소하지 않으므로 해제 후 기존
목표를 계속 수행합니다.

`scan_timestamp_filter`는 `scan_raw` 중 최신 메시지 하나만 보관하고, 해당
타임스탬프의 `<vehicle_id>/odom -> <vehicle_id>/rplidar_link` TF가 준비된
경우에만 기존 `scan` 토픽으로 전달합니다. 기본 허용 지연은 0.5초입니다.
원본 타임스탬프는 변경하지 않습니다.

기존 단일 차량:

```bash
ros2 launch pinky bringup_robot.launch.xml
```

## LED Server

Start the LED service server:

```bash
ros2 run pinky led_server
```

Turn off all LEDs:

```bash
ros2 service call /set_led pinky/srv/SetLed "{command: 'fill', r: 0, g: 0, b: 0}"
```
