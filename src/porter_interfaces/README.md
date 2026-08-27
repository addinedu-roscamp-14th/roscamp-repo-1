# porter_interfaces

Shared ROS interfaces for fleet navigation, robot-arm dispatch, and port events.

- `DispatchNavigation`: namespaced AMR navigation dispatch.
- `DispatchArmCommand`: queued ARM1/ARM2 work with mission and vehicle correlation.
- `VehicleState` / `ArmState`: central readiness and execution state.
- `PortEvent`: debounced vessel arrival/departure events.

Port-ER의 2대 AMR 관제를 위한 ROS 2 인터페이스 패키지입니다.

## Interfaces

- `PixelNavigationCommand`: 카메라 픽셀 목표, 요청 차량, 구역 정보
- `VehicleState`: 차량별 Nav2/배터리/위치/비상정지 상태
- `DispatchNavigation`: AUTO 또는 지정 차량으로 단일 목표/웨이포인트 전송.
  배타 구역 요청은 `use_waiting_pose`와 `waiting_pose`로 선행 대기 위치를 함께
  전달할 수 있습니다. `predecessor_command_id`는 계획 단계 간 성공 의존성을,
  `queue_if_busy`는 현재 작업 뒤에 대기시킬지를 나타냅니다. 두 필드가 없는 일반
  새 명령은 기존 차량 작업을 선점합니다.

```bash
ros2 interface show porter_interfaces/msg/PixelNavigationCommand
ros2 interface show porter_interfaces/msg/VehicleState
ros2 interface show porter_interfaces/action/DispatchNavigation
```
