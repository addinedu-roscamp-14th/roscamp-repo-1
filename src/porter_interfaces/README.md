# porter_interfaces

Port-ER의 2대 AGV 관제를 위한 ROS 2 인터페이스 패키지입니다.

## Interfaces

- `PixelNavigationCommand`: 카메라 픽셀 목표, 요청 차량, 구역 정보
- `VehicleState`: 차량별 Nav2/배터리/위치/비상정지 상태
- `DispatchNavigation`: AUTO 또는 지정 차량으로 단일 목표/웨이포인트 전송

```bash
ros2 interface show porter_interfaces/msg/PixelNavigationCommand
ros2 interface show porter_interfaces/msg/VehicleState
ros2 interface show porter_interfaces/action/DispatchNavigation
```
