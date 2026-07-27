"""Convert structured YOLO selections into validated navigation pixels."""

from __future__ import annotations

import math


PARKING_ZONE_LABELS = frozenset({'B-1'})


class VisualNavigationError(ValueError):
    """Raised when a visual navigation target is invalid or unsafe."""


def compact_detections(summary):
    """Return only the fields the VLM needs, with stable per-frame indices."""
    compact = []
    if not isinstance(summary, dict):
        return compact
    for index, detection in enumerate(summary.get('detections', [])):
        if not isinstance(detection, dict):
            continue
        bbox = _bbox(detection)
        if bbox is None:
            continue
        x1, y1, x2, y2 = bbox
        item = {
            'detection_index': index,
            'label': str(detection.get('label', 'unknown')),
            'confidence': detection.get('confidence'),
            'bbox_xyxy': [x1, y1, x2, y2],
            'center_xy': [
                round((x1 + x2) / 2.0, 2),
                round((y1 + y2) / 2.0, 2),
            ],
        }
        heading = detection.get('heading_deg')
        if isinstance(heading, (int, float)):
            item['heading_deg'] = float(heading)
        compact.append(item)
    return compact


def resolve_detection_approach(
    action,
    summary,
    image_width,
    image_height,
    clearance_px=50.0,
):
    """Compute a parking or outside-approach goal for a selected detection."""
    detections = compact_detections(summary)
    index = action.get('detection_index')
    if isinstance(index, bool) or not isinstance(index, int):
        raise VisualNavigationError('detection_index는 정수여야 합니다')
    selected = next(
        (item for item in detections if item['detection_index'] == index),
        None,
    )
    if selected is None:
        raise VisualNavigationError(
            f'detection_index={index} 검출 결과가 없습니다'
        )

    if selected['label'] in PARKING_ZONE_LABELS:
        return _resolve_parking_zone_goal(
            selected,
            summary,
            image_width,
            image_height,
        )

    side = str(action.get('approach_side', '')).lower()
    if side not in {'left', 'right', 'top', 'bottom'}:
        raise VisualNavigationError(
            'approach_side는 left/right/top/bottom 중 하나여야 합니다'
        )
    x1, y1, x2, y2 = selected['bbox_xyxy']
    center_x, center_y = selected['center_xy']
    candidates = {
        'left': (x1 - clearance_px, center_y),
        'right': (x2 + clearance_px, center_y),
        'top': (center_x, y1 - clearance_px),
        'bottom': (center_x, y2 + clearance_px),
    }
    target_x, target_y = candidates[side]
    if not (
        0.0 <= target_x < float(image_width)
        and 0.0 <= target_y < float(image_height)
    ):
        raise VisualNavigationError(
            f'{side} 접근점이 영상 범위를 벗어납니다: '
            f'({target_x:.1f}, {target_y:.1f})'
        )

    target = {'x': float(target_x), 'y': float(target_y)}
    heading = {'x': float(center_x), 'y': float(center_y)}
    validate_pixel_navigation(
        target,
        heading,
        image_width,
        image_height,
        summary,
        ignored_detection_index=index,
    )
    return target, heading, selected


def _resolve_parking_zone_goal(
    selected,
    summary,
    image_width,
    image_height,
    heading_distance_px=50.0,
):
    center_x, center_y = selected['center_xy']
    heading_deg = selected.get('heading_deg')
    if not isinstance(heading_deg, (int, float)):
        raise VisualNavigationError(
            f'{selected["label"]} 주차 구역의 heading_deg가 없습니다'
        )

    heading_rad = math.radians(float(heading_deg))
    heading_x = center_x + heading_distance_px * math.cos(heading_rad)
    heading_y = center_y + heading_distance_px * math.sin(heading_rad)
    if not (
        0.0 <= heading_x < float(image_width)
        and 0.0 <= heading_y < float(image_height)
    ):
        heading_x = center_x - heading_distance_px * math.cos(heading_rad)
        heading_y = center_y - heading_distance_px * math.sin(heading_rad)

    target = {'x': float(center_x), 'y': float(center_y)}
    heading = {'x': float(heading_x), 'y': float(heading_y)}
    validate_pixel_navigation(
        target,
        heading,
        image_width,
        image_height,
        summary,
        ignored_detection_index=selected['detection_index'],
    )
    return target, heading, selected


def validate_pixel_navigation(
    target,
    heading,
    image_width,
    image_height,
    summary=None,
    ignored_detection_index=None,
):
    """Validate bounds, heading distance, and target/object overlap."""
    target_x, target_y = _point(target, 'target')
    heading_x, heading_y = _point(heading, 'heading')
    for name, x, y in (
        ('target', target_x, target_y),
        ('heading', heading_x, heading_y),
    ):
        if not (0.0 <= x < image_width and 0.0 <= y < image_height):
            raise VisualNavigationError(
                f'{name}=({x:.1f}, {y:.1f})가 영상 범위를 벗어납니다'
            )
    if math.hypot(heading_x - target_x, heading_y - target_y) < 30.0:
        raise VisualNavigationError(
            'target과 heading은 최소 30픽셀 떨어져야 합니다'
        )

    for index, detection in enumerate(
        (summary or {}).get('detections', [])
    ):
        if index == ignored_detection_index:
            continue
        bbox = _bbox(detection)
        if bbox is None:
            continue
        x1, y1, x2, y2 = bbox
        padding = 5.0
        if (
            x1 - padding <= target_x <= x2 + padding
            and y1 - padding <= target_y <= y2 + padding
        ):
            raise VisualNavigationError(
                f'target이 YOLO 객체 내부입니다: '
                f'index={index}, label={detection.get("label", "unknown")}'
            )
    return True


def _bbox(detection):
    bbox = detection.get('bbox_xyxy')
    if not isinstance(bbox, list) or len(bbox) != 4:
        return None
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in bbox
    ):
        return None
    x1, y1, x2, y2 = (float(value) for value in bbox)
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def _point(value, field_name):
    if not isinstance(value, dict):
        raise VisualNavigationError(f'{field_name}은 x/y 객체여야 합니다')
    coordinates = []
    for axis in ('x', 'y'):
        coordinate = value.get(axis)
        if (
            isinstance(coordinate, bool)
            or not isinstance(coordinate, (int, float))
            or not math.isfinite(float(coordinate))
        ):
            raise VisualNavigationError(
                f'{field_name}.{axis}가 유효한 숫자가 아닙니다'
            )
        coordinates.append(float(coordinate))
    return coordinates
