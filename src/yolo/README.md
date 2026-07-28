# YOLO 패키지

세그멘테이션 YOLO 추론과 결과 시각화를 담당하는 패키지입니다.

## 실행

```bash
cd /home/jio/poter_ws
source install/setup.bash

ros2 run yolo yolo_node --ros-args \
  -p weights_path:=config/weights/best.pt
```

기본 입력 토픽은 압축된 왜곡 보정 이미지인 `/image_rect/compressed`입니다.

기본 인식 클래스는 학습 모델의 다음 ID 순서를 사용합니다.

| ID | 클래스 |
|---:|---|
| 0 | `trailer` |
| 1 | `car_yellow` |
| 2 | `car_blue` |
| 3 | `A-1` |
| 4 | `A-2` |
| 5 | `A-3` |
| 6 | `B-1` |

시각화 토픽:

- `/central/yolo/image_annotated`
- `/central/yolo/detections`   

세그멘테이션 mask가 있으면 `/central/yolo/detections` JSON에 `heading_deg`를
포함합니다. B-1 주차 정렬에 사용되므로 계산과 JSON 발행은 유지하지만 annotated
image의 heading 텍스트 표시는 기본적으로 꺼져 있습니다.

헤딩 텍스트가 필요한 경우에만 다음 파라미터로 켭니다.

```bash
ros2 run yolo yolo_node --ros-args \
  -p show_heading_annotation:=true
```
