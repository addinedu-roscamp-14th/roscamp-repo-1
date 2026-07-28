"""Tests for the HTTP client that reads fresh YOLO summaries."""

from yolo_detection_client import YoloDetectionClient


def test_latest_detection_is_returned(monkeypatch):
    payload = {
        'status': 'ok',
        'age_sec': 0.1,
        'detections': [{'label': 'car_blue'}],
    }

    class Response:
        ok = True
        status_code = 200

        @staticmethod
        def json():
            return payload

    monkeypatch.setattr(
        'yolo_detection_client.requests.get',
        lambda url, timeout: Response(),
    )
    result = YoloDetectionClient(
        url='http://central:8000/detections'
    ).get_latest()
    assert result == payload
