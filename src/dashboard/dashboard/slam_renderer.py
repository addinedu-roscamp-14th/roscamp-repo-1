"""Render ROS occupancy grids and robot poses for the dashboard."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class MapLayout:
    """Map metadata and its fitted location in the output image."""

    grid_width: int
    grid_height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    scale: float
    offset_x: int
    offset_y: int


def quaternion_to_yaw(quaternion):
    """Return planar yaw from an object with x, y, z and w fields."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def render_occupancy_grid(message, output_width, output_height):
    """Convert an OccupancyGrid into a letterboxed BGR image."""
    width = int(message.info.width)
    height = int(message.info.height)
    resolution = float(message.info.resolution)
    if width <= 0 or height <= 0:
        raise ValueError('occupancy grid dimensions must be positive')
    if resolution <= 0.0:
        raise ValueError('occupancy grid resolution must be positive')
    if output_width <= 0 or output_height <= 0:
        raise ValueError('SLAM output dimensions must be positive')

    occupancy = np.asarray(message.data, dtype=np.int16)
    if occupancy.size != width * height:
        raise ValueError(
            f'occupancy data size {occupancy.size} does not match '
            f'{width}x{height}'
        )
    occupancy = occupancy.reshape((height, width))

    gray = np.full((height, width), 205, dtype=np.uint8)
    known = occupancy >= 0
    gray[known] = np.rint(
        254.0 * (100.0 - np.clip(occupancy[known], 0, 100)) / 100.0
    ).astype(np.uint8)

    # OccupancyGrid starts at the lower-left; images start at the upper-left.
    map_image = cv2.cvtColor(np.flipud(gray), cv2.COLOR_GRAY2BGR)
    scale = min(output_width / width, output_height / height)
    fitted_width = max(1, int(round(width * scale)))
    fitted_height = max(1, int(round(height * scale)))
    fitted = cv2.resize(
        map_image,
        (fitted_width, fitted_height),
        interpolation=cv2.INTER_NEAREST,
    )
    offset_x = (output_width - fitted_width) // 2
    offset_y = (output_height - fitted_height) // 2
    canvas = np.full(
        (output_height, output_width, 3),
        48,
        dtype=np.uint8,
    )
    canvas[
        offset_y:offset_y + fitted_height,
        offset_x:offset_x + fitted_width,
    ] = fitted

    origin = message.info.origin
    layout = MapLayout(
        grid_width=width,
        grid_height=height,
        resolution=resolution,
        origin_x=float(origin.position.x),
        origin_y=float(origin.position.y),
        origin_yaw=quaternion_to_yaw(origin.orientation),
        scale=scale,
        offset_x=offset_x,
        offset_y=offset_y,
    )
    return canvas, layout


def world_to_canvas(layout, world_x, world_y):
    """Convert a world coordinate into a fitted map image coordinate."""
    dx = world_x - layout.origin_x
    dy = world_y - layout.origin_y
    cos_yaw = math.cos(layout.origin_yaw)
    sin_yaw = math.sin(layout.origin_yaw)
    grid_x = (cos_yaw * dx + sin_yaw * dy) / layout.resolution
    grid_y = (-sin_yaw * dx + cos_yaw * dy) / layout.resolution
    pixel_x = layout.offset_x + grid_x * layout.scale
    pixel_y = (
        layout.offset_y
        + (layout.grid_height - 1 - grid_y) * layout.scale
    )
    return pixel_x, pixel_y


def draw_robot_pose(
    image,
    layout,
    world_x,
    world_y,
    world_yaw,
    body_color=(220, 90, 30),
    heading_color=(20, 20, 230),
    label='',
):
    """Draw the robot position and heading; return false if out of map."""
    grid_dx = world_x - layout.origin_x
    grid_dy = world_y - layout.origin_y
    cos_yaw = math.cos(layout.origin_yaw)
    sin_yaw = math.sin(layout.origin_yaw)
    grid_x = (
        cos_yaw * grid_dx + sin_yaw * grid_dy
    ) / layout.resolution
    grid_y = (
        -sin_yaw * grid_dx + cos_yaw * grid_dy
    ) / layout.resolution
    if not (
        0.0 <= grid_x < layout.grid_width
        and 0.0 <= grid_y < layout.grid_height
    ):
        return False

    pixel_x, pixel_y = world_to_canvas(layout, world_x, world_y)
    center = (int(round(pixel_x)), int(round(pixel_y)))
    radius = max(
        5,
        min(
            18,
            int(round(0.04 / layout.resolution * layout.scale)),
        ),
    )
    heading_length = max(16, radius * 2)
    relative_yaw = world_yaw - layout.origin_yaw
    endpoint = (
        int(round(pixel_x + heading_length * math.cos(relative_yaw))),
        int(round(pixel_y - heading_length * math.sin(relative_yaw))),
    )
    cv2.circle(image, center, radius + 2, (255, 255, 255), -1)
    cv2.circle(image, center, radius, body_color, -1)
    cv2.arrowedLine(
        image,
        center,
        endpoint,
        heading_color,
        3,
        tipLength=0.35,
    )
    if label:
        cv2.putText(
            image,
            str(label),
            (center[0] + radius + 4, center[1] - radius - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            body_color,
            2,
            cv2.LINE_AA,
        )
    return True


def draw_laser_scan(
    image,
    layout,
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    transform_x,
    transform_y,
    transform_yaw,
    max_points=720,
    point_color=(40, 220, 40),
):
    """Draw valid planar LaserScan points transformed into the map frame."""
    points = laser_scan_to_canvas_points(
        layout,
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        transform_x,
        transform_y,
        transform_yaw,
        max_points,
    )
    point_radius = max(1, min(3, int(round(layout.scale))))
    for x, y in points:
        cv2.circle(
            image,
            (int(x), int(y)),
            point_radius,
            point_color,
            -1,
        )
    return int(points.shape[0])


def laser_scan_to_canvas_points(
    layout,
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    transform_x,
    transform_y,
    transform_yaw,
    max_points=720,
):
    """Transform valid planar LaserScan ranges into map image pixels."""
    scan_ranges = np.asarray(ranges, dtype=np.float32)
    if scan_ranges.size == 0 or angle_increment == 0.0:
        return np.empty((0, 2), dtype=np.int32)

    stride = max(1, int(math.ceil(scan_ranges.size / max_points)))
    indices = np.arange(0, scan_ranges.size, stride)
    sampled_ranges = scan_ranges[indices]
    valid = (
        np.isfinite(sampled_ranges)
        & (sampled_ranges >= range_min)
        & (sampled_ranges <= range_max)
    )
    if not np.any(valid):
        return np.empty((0, 2), dtype=np.int32)

    sampled_ranges = sampled_ranges[valid]
    angles = angle_min + indices[valid] * angle_increment
    scan_x = sampled_ranges * np.cos(angles)
    scan_y = sampled_ranges * np.sin(angles)

    tf_cos = math.cos(transform_yaw)
    tf_sin = math.sin(transform_yaw)
    world_x = transform_x + tf_cos * scan_x - tf_sin * scan_y
    world_y = transform_y + tf_sin * scan_x + tf_cos * scan_y

    map_cos = math.cos(layout.origin_yaw)
    map_sin = math.sin(layout.origin_yaw)
    dx = world_x - layout.origin_x
    dy = world_y - layout.origin_y
    grid_x = (map_cos * dx + map_sin * dy) / layout.resolution
    grid_y = (-map_sin * dx + map_cos * dy) / layout.resolution
    inside = (
        (grid_x >= 0.0)
        & (grid_x < layout.grid_width)
        & (grid_y >= 0.0)
        & (grid_y < layout.grid_height)
    )
    if not np.any(inside):
        return np.empty((0, 2), dtype=np.int32)

    pixel_x = np.rint(
        layout.offset_x + grid_x[inside] * layout.scale
    ).astype(np.int32)
    pixel_y = np.rint(
        layout.offset_y
        + (layout.grid_height - 1 - grid_y[inside]) * layout.scale
    ).astype(np.int32)
    return np.column_stack((pixel_x, pixel_y))
