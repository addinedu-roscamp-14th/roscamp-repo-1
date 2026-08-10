"""Convert structured YOLO selections into validated navigation pixels."""

from __future__ import annotations

import math


PARKING_ZONE_LABELS = frozenset({'B-1'})

# A-1/A-2/A-3 are cargo storage bins that all share one fixed, pre-measured
# map-frame stop pose (set in camera_to_map_bridge) instead of a per-bbox
# computed one, since a vehicle parked there loads/unloads from the same spot
# regardless of which bin is being worked.
FIXED_ZONE_LABELS = frozenset({'A-1', 'A-2', 'A-3'})

_ZONE_MODE_BY_LABEL = {
    'B-1': 'parking_b1',
    **{label: 'parking_a' for label in FIXED_ZONE_LABELS},
}

# AMR 1 (agv1) carries the blue cargo box, AMR 2 (agv2) the yellow one -
# matching pinky.urdf.xacro and fleet_collision_supervisor.yaml. This map used
# to be inverted, which sent colour-addressed commands to the wrong robot.
VEHICLE_ID_BY_LABEL = {
    'car_blue': 'agv1',
    'car_yellow': 'agv2',
}


def zone_mode_for_label(label):
    """Map a detection label to the pixel-goal mode used to reach it."""
    return _ZONE_MODE_BY_LABEL.get(label, 'direct')


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


def select_nearest_visible_vehicle(
    reference_detection,
    summary,
    eligible_vehicle_ids=None,
):
    """Select the live AGV detection nearest to a visible source object."""
    reference_center = reference_detection.get('center_xy')
    if not isinstance(reference_center, (list, tuple)) or len(reference_center) != 2:
        raise VisualNavigationError('출발 구역의 현재 중심 좌표가 없습니다')

    eligible = (
        None
        if eligible_vehicle_ids is None
        else {str(vehicle_id) for vehicle_id in eligible_vehicle_ids}
    )
    candidates = []
    for detection in compact_detections(summary):
        vehicle_id = VEHICLE_ID_BY_LABEL.get(detection['label'])
        if vehicle_id is None or (
            eligible is not None and vehicle_id not in eligible
        ):
            continue
        center_x, center_y = detection['center_xy']
        distance = math.hypot(
            float(center_x) - float(reference_center[0]),
            float(center_y) - float(reference_center[1]),
        )
        candidates.append((distance, vehicle_id))
    return min(candidates, default=(math.inf, ''))[1]


def is_reciprocal_zone_exchange(actions, summary, zone_status):
    """Return True when two AGVs are exchanging occupied A/B-1 zones."""
    if not isinstance(actions, list) or len(actions) != 2:
        return False

    owners = {}
    for entry in str(zone_status or '').split(';'):
        parts = entry.split(':')
        if len(parts) < 2 or parts[1] in {'', 'FREE', 'UNKNOWN'}:
            continue
        owners[parts[0]] = parts[-1]

    detections = {
        detection['detection_index']: detection
        for detection in compact_detections(summary)
    }
    destinations = {}
    for action in actions:
        if not isinstance(action, dict) or action.get('type') != 'visual_navigation':
            return False
        vehicle_id = str(action.get('vehicle_id') or '').strip().lower()
        if vehicle_id not in {'agv1', 'agv2'}:
            return False
        detection = detections.get(action.get('detection_index'))
        if detection is None:
            return False
        mode = zone_mode_for_label(detection['label'])
        zone_id = {'parking_a': 'A', 'parking_b1': 'B-1'}.get(mode)
        if zone_id is None:
            return False
        destinations[vehicle_id] = zone_id

    if set(destinations) != {'agv1', 'agv2'}:
        return False
    return all(
        owners.get(target_zone) in destinations
        and owners[target_zone] != vehicle_id
        for vehicle_id, target_zone in destinations.items()
    )


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
    if selected['label'] in FIXED_ZONE_LABELS:
        return _resolve_fixed_zone_goal(
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


def resolve_vehicle_position_swap(
    summary,
    image_width,
    image_height,
    heading_distance_px=50.0,
):
    """Build deterministic goals that exchange agv1/agv2 camera positions."""
    detections = compact_detections(summary)
    vehicles = {
        item['label']: item
        for item in detections
        if item['label'] in {'car_yellow', 'car_blue'}
    }
    missing = {
        label for label in {'car_yellow', 'car_blue'}
        if label not in vehicles
    }
    if missing:
        raise VisualNavigationError(
            '차량 위치 교환에는 car_yellow와 car_blue 검출이 모두 '
            f'필요합니다: missing={sorted(missing)}'
        )

    vehicle_ids = dict(VEHICLE_ID_BY_LABEL)
    opposite_labels = {
        'car_yellow': 'car_blue',
        'car_blue': 'car_yellow',
    }
    goals = []
    for source_label in ('car_yellow', 'car_blue'):
        source = vehicles[source_label]
        destination = vehicles[opposite_labels[source_label]]
        target = {
            'x': float(destination['center_xy'][0]),
            'y': float(destination['center_xy'][1]),
        }
        heading = _heading_from_detection(
            target,
            source,
            image_width,
            image_height,
            heading_distance_px,
        )
        target_mode = _zone_mode_at_point(target, detections)
        source_mode = _zone_mode_at_point(
            {
                'x': float(source['center_xy'][0]),
                'y': float(source['center_xy'][1]),
            },
            detections,
        )
        ignored_indices = {
            item['detection_index']
            for item in detections
            if _bbox_contains(item.get('bbox_xyxy'), target)
        }
        validate_pixel_navigation(
            target,
            heading,
            image_width,
            image_height,
            summary,
            ignored_detection_indices=ignored_indices,
        )
        goals.append({
            'vehicle_id': vehicle_ids[source_label],
            'source_label': source_label,
            'destination_label': destination['label'],
            'target': target,
            'heading': heading,
            'mode': target_mode,
            'source_mode': source_mode,
        })

    # A vehicle already holding an exclusive zone must leave first. The
    # incoming vehicle can then wait on the normal fleet zone lock.
    goals.sort(
        key=lambda goal: (
            0
            if goal['source_mode'] != 'direct'
            and goal['mode'] == 'direct'
            else 1,
            goal['vehicle_id'],
        )
    )
    return goals


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


def _resolve_fixed_zone_goal(
    selected,
    summary,
    image_width,
    image_height,
    heading_distance_px=50.0,
):
    """A-1/A-2/A-3: heading always faces image-up; the real stop pose is a
    fixed map-frame pose substituted server-side (camera_to_map_bridge), so
    these pixels only need to pass bounds/overlap validation."""
    center_x, center_y = selected['center_xy']
    heading_y = center_y - heading_distance_px
    if heading_y < 0.0:
        heading_y = center_y + heading_distance_px

    target = {'x': float(center_x), 'y': float(center_y)}
    heading = {'x': float(center_x), 'y': float(heading_y)}
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
    ignored_detection_indices=None,
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

    ignored_indices = set(ignored_detection_indices or ())
    if ignored_detection_index is not None:
        ignored_indices.add(ignored_detection_index)

    for index, detection in enumerate(
        (summary or {}).get('detections', [])
    ):
        if index in ignored_indices:
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


def _heading_from_detection(
    target,
    detection,
    image_width,
    image_height,
    distance_px,
):
    heading_deg = detection.get('heading_deg')
    if isinstance(heading_deg, (int, float)):
        angle = math.radians(float(heading_deg))
        dx = distance_px * math.cos(angle)
        dy = distance_px * math.sin(angle)
    else:
        dx = distance_px
        dy = 0.0

    candidates = (
        {'x': target['x'] + dx, 'y': target['y'] + dy},
        {'x': target['x'] - dx, 'y': target['y'] - dy},
    )
    return next(
        (
            point for point in candidates
            if (
                0.0 <= point['x'] < float(image_width)
                and 0.0 <= point['y'] < float(image_height)
            )
        ),
        {
            'x': min(
                max(target['x'] + distance_px, 0.0),
                float(image_width) - 1.0,
            ),
            'y': target['y'],
        },
    )


def _zone_mode_at_point(point, detections):
    for detection in detections:
        label = detection.get('label')
        if label not in PARKING_ZONE_LABELS | FIXED_ZONE_LABELS:
            continue
        if _bbox_contains(detection.get('bbox_xyxy'), point):
            return zone_mode_for_label(label)
    return 'direct'


def _bbox_contains(bbox, point):
    if not isinstance(bbox, list) or len(bbox) != 4:
        return False
    x1, y1, x2, y2 = (float(value) for value in bbox)
    return (
        x1 <= float(point['x']) <= x2
        and y1 <= float(point['y']) <= y2
    )


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
