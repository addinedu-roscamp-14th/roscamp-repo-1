"""Fetch the latest structured YOLO result from the central stream API."""

from __future__ import annotations

import os

import requests


class YoloDetectionError(RuntimeError):
    """Raised when fresh YOLO detections cannot be obtained."""


class YoloDetectionClient:
    """Read the newest detection summary without subscribing to ROS directly."""

    def __init__(self, url=None, timeout_sec=2.0, max_age_sec=2.0):
        self.url = url or os.environ.get(
            'PORT_CONTROL_DETECTIONS_URL',
            'http://127.0.0.1:8000/detections',
        )
        self.timeout_sec = float(timeout_sec)
        self.max_age_sec = float(max_age_sec)

    def get_latest(self):
        try:
            response = requests.get(self.url, timeout=self.timeout_sec)
        except requests.RequestException as exc:
            raise YoloDetectionError(
                f'YOLO 검출 API에 연결할 수 없습니다: {exc}'
            ) from exc
        try:
            payload = response.json()
        except ValueError as exc:
            raise YoloDetectionError(
                'YOLO 검출 API 응답이 JSON이 아닙니다'
            ) from exc
        if not response.ok:
            raise YoloDetectionError(
                f'YOLO 검출 API 오류 HTTP {response.status_code}: {payload}'
            )
        if payload.get('status') != 'ok':
            raise YoloDetectionError(
                f'최신 YOLO 검출이 준비되지 않았습니다: '
                f'{payload.get("status", "unknown")}'
            )
        age = payload.get('age_sec')
        if not isinstance(age, (int, float)) or age > self.max_age_sec:
            raise YoloDetectionError(
                f'YOLO 검출이 오래되었습니다: age={age}'
            )
        if not isinstance(payload.get('detections'), list):
            raise YoloDetectionError('YOLO detections 배열이 없습니다')
        return payload
