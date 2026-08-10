"""Tests for bounded latest-frame storage."""

from dashboard.frame_store import LatestFrameStore, LatestJpegStore
from dashboard.slam_renderer import (
    draw_laser_scan,
    draw_robot_pose,
    render_occupancy_grid,
    world_to_canvas,
)


class Value:
    pass


def make_map():
    message = Value()
    message.data = [0, 100, -1, 50]
    message.info = Value()
    message.info.width = 2
    message.info.height = 2
    message.info.resolution = 1.0
    message.info.origin = Value()
    message.info.origin.position = Value()
    message.info.origin.position.x = 0.0
    message.info.origin.position.y = 0.0
    message.info.origin.orientation = Value()
    message.info.origin.orientation.x = 0.0
    message.info.origin.orientation.y = 0.0
    message.info.origin.orientation.z = 0.0
    message.info.origin.orientation.w = 1.0
    return message


def test_latest_frame_replaces_older_message():
    store = LatestFrameStore()

    store.put('old')
    store.put('new')

    snapshot = store.latest()
    assert snapshot.message == 'new'
    assert snapshot.sequence == 2
    assert store.input_count == 2


def test_wait_for_new_returns_only_newer_sequence():
    store = LatestFrameStore()
    store.put('frame')

    assert store.wait_for_new(1, timeout=0.0) is None
    snapshot = store.wait_for_new(0, timeout=0.0)
    assert snapshot.message == 'frame'


def test_latest_jpeg_is_shared_without_per_client_copy():
    store = LatestJpegStore()
    jpeg = b'jpeg-data'

    store.put(jpeg, source_sequence=7, encode_duration_ms=1.5)
    store.add_client()
    store.add_client()

    snapshot = store.latest()
    assert snapshot.data is jpeg
    assert snapshot.source_sequence == 7
    assert store.metrics() == {'encoded_count': 1, 'client_count': 2}

    store.remove_client()
    assert store.metrics()['client_count'] == 1


def test_occupancy_grid_uses_ros_lower_left_origin():
    image, _ = render_occupancy_grid(make_map(), 2, 2)

    assert image[0, 0, 0] == 205
    assert image[0, 1, 0] == 127
    assert image[1, 0, 0] == 254
    assert image[1, 1, 0] == 0


def test_world_coordinate_maps_to_flipped_image_axis():
    _, layout = render_occupancy_grid(make_map(), 2, 2)

    pixel_x, pixel_y = world_to_canvas(layout, 0.5, 0.5)

    assert pixel_x == 0.5
    assert pixel_y == 0.5


def test_robot_pose_is_drawn_only_inside_map():
    image, layout = render_occupancy_grid(make_map(), 200, 200)

    assert draw_robot_pose(image, layout, 0.5, 0.5, 0.0)
    assert not draw_robot_pose(image, layout, 3.0, 0.5, 0.0)


def test_laser_scan_draws_only_valid_points_inside_map():
    image, layout = render_occupancy_grid(make_map(), 200, 200)

    count = draw_laser_scan(
        image,
        layout,
        ranges=[0.5, float('inf'), 3.0],
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.1,
        range_max=5.0,
        transform_x=0.5,
        transform_y=0.5,
        transform_yaw=0.0,
    )

    assert count == 1
