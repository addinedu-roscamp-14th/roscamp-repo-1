"""Tests for ChArUco board geometry helpers."""

from arm.charuco_pose_publisher import (
    board_chessboard_corners,
    create_charuco_board,
    select_charuco_correspondences,
)
import cv2
import numpy as np
import pytest


def make_board():
    """Create the project's recommended board."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    return create_charuco_board(5, 7, 0.025, 0.018, dictionary)


def test_recommended_board_has_expected_corner_geometry():
    """A 5x7 board exposes 4x6 internal ChArUco corners."""
    corners = board_chessboard_corners(make_board())
    assert corners.shape == (24, 3)
    assert np.allclose(corners[0], [0.025, 0.025, 0.0])
    assert np.allclose(corners[-1], [0.100, 0.150, 0.0])


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
