#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description='Generate printable ArUco marker PNG files')
    parser.add_argument('--dictionary', default='DICT_5X5_50')
    parser.add_argument('--ids', default='0,1,2,3,4,5,6,7,8,9')
    parser.add_argument('--pixels', type=int, default=800)
    parser.add_argument('--border', type=int, default=100)
    parser.add_argument(
        '--output',
        default=str(Path.home() / 'poter_ws/config/aruco_markers'),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    dictionary_id = getattr(cv2.aruco, args.dictionary, None)
    if dictionary_id is None:
        raise ValueError(f'Unknown ArUco dictionary: {args.dictionary}')
    if args.pixels < 100 or args.border < 0:
        raise ValueError('pixels must be >= 100 and border must be >= 0')

    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    output_dir = Path(args.output).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    marker_ids = [int(value.strip()) for value in args.ids.split(',') if value.strip()]
    for marker_id in marker_ids:
        marker = np.zeros((args.pixels, args.pixels), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, args.pixels, marker, 1)
        printable = cv2.copyMakeBorder(
            marker,
            args.border,
            args.border,
            args.border,
            args.border,
            cv2.BORDER_CONSTANT,
            value=255,
        )
        output_file = output_dir / f'aruco_{args.dictionary}_id_{marker_id}.png'
        if not cv2.imwrite(str(output_file), printable):
            raise RuntimeError(f'Failed to save {output_file}')
        print(output_file)

    print('\nPrint without page scaling and measure the black square precisely.')


if __name__ == '__main__':
    main()
