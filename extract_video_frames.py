#!/usr/bin/env python3

from pathlib import Path

import cv2


# ~/poter_ws에서 사용하는 입력/출력 경로입니다.
WORKSPACE = Path.home() / 'poter_ws'
VIDEO_DIR = WORKSPACE / 'datasets/topview/videos'
BASE_OUTPUT_DIR = WORKSPACE / 'datasets/topview/images'

# 30 FPS 영상에서 15프레임마다 저장하면 약 0.5초당 1장입니다.
FRAME_INTERVAL = 15
MAX_FRAME = 5400
TRAIN_RATIO = 0.8
VAL_DIRECTORY_NAME = 'val2'
VIDEO_EXTENSIONS = {'.mp4', '.avi', '.mov', '.mkv'}


def list_videos():
    if not VIDEO_DIR.is_dir():
        print(f'오류: 영상 폴더를 찾을 수 없습니다: {VIDEO_DIR}')
        return []

    return sorted(
        path for path in VIDEO_DIR.iterdir()
        if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS
    )


def extract_video(video_path, target_dir):
    target_dir.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        print(f'오류: 영상을 열 수 없습니다: {video_path}')
        return 0, 0

    frame_index = 0
    saved_count = 0

    while frame_index < MAX_FRAME:
        ok, frame = capture.read()
        if not ok:
            break

        if frame_index % FRAME_INTERVAL == 0:
            output_file = target_dir / f'{video_path.stem}_{frame_index:04d}.jpg'
            if cv2.imwrite(str(output_file), frame):
                saved_count += 1
            else:
                print(f'경고: 이미지 저장 실패: {output_file}')

        frame_index += 1

    capture.release()
    return frame_index, saved_count


def main():
    videos = list_videos()
    print(f'찾은 동영상 개수: {len(videos)}개')
    if not videos:
        return

    # 영상 단위로 나누어 비슷한 인접 프레임이 train과 val에
    # 동시에 들어가는 데이터 누출을 막습니다.
    train_count = max(1, int(len(videos) * TRAIN_RATIO))
    if len(videos) > 1:
        train_count = min(train_count, len(videos) - 1)

    total_saved = 0
    for index, video_path in enumerate(videos):
        split = 'train' if index < train_count else VAL_DIRECTORY_NAME
        target_dir = BASE_OUTPUT_DIR / split
        scanned_count, saved_count = extract_video(video_path, target_dir)
        total_saved += saved_count
        print(
            f'[{index + 1}/{len(videos)}] {video_path.name} -> {split}: '
            f'{saved_count}장 추출 (검사 프레임: {scanned_count})'
        )

    print(f'\n[완료] 총 {total_saved}장 저장: {BASE_OUTPUT_DIR}')


if __name__ == '__main__':
    main()
