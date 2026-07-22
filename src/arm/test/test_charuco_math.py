"""Tests for ChArUco board geometry helpers."""

from arm.charuco_pose_publisher import (
    board_chessboard_corners,
    charuco_drawing_arrays,
    create_charuco_board,
    projected_axes_are_visible,
    select_charuco_correspondences,
)
import cv2
import numpy as np
import pytest


def make_board():
    """Create the project's recommended board."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return create_charuco_board(11, 8, 0.015, 0.011, dictionary)


def test_recommended_board_has_expected_corner_geometry():
    """An 11x8 board exposes 10x7 internal ChArUco corners."""
    corners = board_chessboard_corners(make_board())
    assert corners.shape == (70, 3)
    assert np.allclose(corners[0], [0.015, 0.015, 0.0])
    assert np.allclose(corners[-1], [0.150, 0.105, 0.0])


def test_legacy_pattern_is_applied_when_supported():
    """The physical even-row board can select OpenCV's legacy layout."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = create_charuco_board(
        11, 8, 0.015, 0.011, dictionary, legacy_pattern=True
    )
    if hasattr(board, 'getLegacyPattern'):
        assert board.getLegacyPattern() is True


def test_correspondences_follow_corner_ids():
    """Image observations retain the board corner ID ordering."""
    board_corners = board_chessboard_corners(make_board())
    image_corners = np.array([[[10.0, 20.0]], [[30.0, 40.0]]])
    object_points, image_points = select_charuco_correspondences(
        board_corners, image_corners, [[5], [2]]
    )
    assert np.allclose(object_points, board_corners[[5, 2]])
    assert np.allclose(image_points, [[10.0, 20.0], [30.0, 40.0]])


def test_correspondences_reject_invalid_corner_id():
    """A corrupt detector ID cannot index outside the board."""
    with pytest.raises(ValueError):
        select_charuco_correspondences(
            board_chessboard_corners(make_board()),
            [[[10.0, 20.0]]],
            [[99]],
        )


def test_drawing_arrays_normalize_opencv_5_shapes():
    """Flat OpenCV 5 outputs are converted for compatible drawing."""
    corners, ids = charuco_drawing_arrays(
        [[10.0, 20.0], [30.0, 40.0]],
        [5, 2],
    )
    assert corners.shape == (2, 1, 2)
    assert corners.dtype == np.float32
    assert ids.shape == (2, 1)
    assert ids.dtype == np.int32


def test_projected_axes_visibility_tracks_image_bounds():
    """Axes are drawable only when their origin and endpoints are in frame."""
    camera_matrix = np.array([
        [100.0, 0.0, 50.0],
        [0.0, 100.0, 50.0],
        [0.0, 0.0, 1.0],
    ])
    rotation = np.zeros((3, 1))
    distortion = np.zeros(5)
    assert projected_axes_are_visible(
        rotation,
        np.array([[0.0], [0.0], [1.0]]),
        camera_matrix,
        distortion,
        0.1,
        (100, 100, 3),
    )
    assert not projected_axes_are_visible(
        rotation,
        np.array([[-1.0], [0.0], [1.0]]),
        camera_matrix,
        distortion,
        0.1,
        (100, 100, 3),
    )
