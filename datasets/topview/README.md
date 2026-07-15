# Topview segmentation dataset

Classes must keep this exact order:

```text
0 red
1 blue
```

Label each visible object with a polygon segmentation mask, not only a bounding box.
Export labels in Ultralytics YOLO Segmentation format and place matching image/label
base names in the corresponding `train`, `val`, or `test` directories.

Example pair:

```text
images/train/topview_001.jpg
labels/train/topview_001.txt
```

Recommended split is 70% train, 20% validation, and 10% test. Split by scene or
recording session so near-identical adjacent frames do not occur in different sets.
Images containing no target object are useful negative samples; their label file may
be empty.
