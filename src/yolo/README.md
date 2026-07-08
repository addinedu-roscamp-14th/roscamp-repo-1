# YOLO 패키지

세그멘테이션 YOLO 추론과 결과 시각화를 담당하는 패키지입니다.

## 실행

```bash
cd /home/jio/poter_ws
source install/setup.bash

ros2 run yolo yolo_node --ros-args \
  -p input_is_compressed:=False \
  -p input_topic:=/camera/image_rect \
  -p weights_path:=config/weights/best.pt
```

압축 이미지를 직접 받을 경우:

```bash
ros2 run yolo yolo_node --ros-args \
  -p input_is_compressed:=True \
  -p input_topic:=/camera/image_rect/compressed \
  -p weights_path:=config/weights/best.pt
```

시각화 토픽:

- `/central/yolo/image_annotated`
- `/central/yolo/detections`