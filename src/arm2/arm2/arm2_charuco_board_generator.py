"""Generate a dimensioned printable ChArUco board PNG."""

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from .arm2_charuco_pose_publisher import create_charuco_board


def generate_board_image(board, board_size_px, margin_px):
    """Render a board with an external white detection margin."""
    if hasattr(board, 'generateImage'):
        board_image = board.generateImage(
            board_size_px, marginSize=0, borderBits=1
        )
    else:
        board_image = board.draw(
            board_size_px, marginSize=0, borderBits=1
        )
    return np.pad(
        np.asarray(board_image, dtype=np.uint8),
        ((margin_px, margin_px), (margin_px, margin_px)),
        mode='constant',
        constant_values=255,
    )


def main():
    """Generate a board using millimetre dimensions and embedded DPI."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output',
        default='config/arm2/arm2_charuco_8x11_15mm_11mm.png',
    )
    parser.add_argument('--dictionary', default='DICT_4X4_50')
    # The physical label uses rows x columns (8x11), while OpenCV expects
    # squares-x=columns and squares-y=rows.
    parser.add_argument('--squares-x', type=int, default=11)
    parser.add_argument('--squares-y', type=int, default=8)
    parser.add_argument('--square-mm', type=float, default=15.0)
    parser.add_argument('--marker-mm', type=float, default=11.0)
    parser.add_argument(
        '--legacy-pattern',
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument('--margin-mm', type=float, default=10.0)
    parser.add_argument('--pixels-per-square', type=int, default=200)
    arguments = parser.parse_args()

    if arguments.squares_x < 3 or arguments.squares_y < 3:
        raise ValueError('Board must have at least 3x3 squares')
    if not 0.0 < arguments.marker_mm < arguments.square_mm:
        raise ValueError('Marker length must be below square length')
    if arguments.pixels_per_square < 50:
        raise ValueError('pixels-per-square must be at least 50')
    if not hasattr(cv2.aruco, arguments.dictionary):
        raise ValueError(f'Unknown dictionary: {arguments.dictionary}')

    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, arguments.dictionary)
    )
    board = create_charuco_board(
        arguments.squares_x,
        arguments.squares_y,
        arguments.square_mm / 1000.0,
        arguments.marker_mm / 1000.0,
        dictionary,
        arguments.legacy_pattern,
    )
    width = arguments.squares_x * arguments.pixels_per_square
    height = arguments.squares_y * arguments.pixels_per_square
    pixels_per_mm = arguments.pixels_per_square / arguments.square_mm
    margin_px = int(round(arguments.margin_mm * pixels_per_mm))
    image = generate_board_image(board, (width, height), margin_px)
    dpi = pixels_per_mm * 25.4

    output = Path(arguments.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(output, dpi=(dpi, dpi))
    print(f'Generated: {output.resolve()}')
    print(
        'Board content: '
        f'{arguments.squares_x * arguments.square_mm:g} x '
        f'{arguments.squares_y * arguments.square_mm:g} mm; '
        f'margin={arguments.margin_mm:g} mm; dpi={dpi:.2f}; '
        f'legacy_pattern={arguments.legacy_pattern}'
    )


if __name__ == '__main__':
    main()
