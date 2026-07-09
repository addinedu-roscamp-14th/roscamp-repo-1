# YOLO 패키지

세그멘테이션 YOLO 추론과 결과 시각화를 담당하는 패키지입니다.

## 실행

```bash
cd /home/jio/poter_ws
source install/setup.bash

ros2 run yolo yolo_node --ros-args \
  -p input_is_compressed:=False \
  -p input_topic:=/camera/image_rect \
  -p weights_path:=config/weights/best.pt \
  -p confidence_threshold:=0.6
```


시각화 토픽:

- `/central/yolo/image_annotated`
- `/central/yolo/detections`   

세그멘테이션 mask가 있을 경우 annotated image에 heading 각도를 함께 표시하고,
`/central/yolo/detections` JSON에는 `heading_deg`를 포함합니다.
