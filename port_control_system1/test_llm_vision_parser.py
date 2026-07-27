"""Tests for image forwarding to the Ollama-compatible VLM."""

import sys
import types

from llm_command_parser import parse_command_with_llm


def test_image_is_forwarded_to_vlm(monkeypatch):
    """The current JPEG must be attached to the user message."""
    captured = {}

    class FakeClient:
        def __init__(self, host, timeout):
            captured['host'] = host
            captured['timeout'] = timeout

        def chat(self, **kwargs):
            captured['kwargs'] = kwargs
            return {
                'message': {
                    'content': (
                        '{"actions":[{"type":"pixel_navigation",'
                        '"target":{"x":100,"y":200},'
                        '"heading":{"x":150,"y":200}}]}'
                    )
                }
            }

    monkeypatch.setitem(
        sys.modules,
        'ollama',
        types.SimpleNamespace(Client=FakeClient),
    )
    result = parse_command_with_llm(
        '빈 공간으로 이동',
        [],
        [],
        [],
        image_jpeg=b'jpeg-bytes',
        image_width=640,
        image_height=480,
        yolo_detections=[
            {
                'detection_index': 0,
                'label': 'car_blue',
                'bbox_xyxy': [100, 120, 180, 200],
                'center_xy': [140, 160],
            }
        ],
    )

    user_message = captured['kwargs']['messages'][1]
    assert user_message['images'] == [b'jpeg-bytes']
    assert '"detection_index":0' in user_message['content']
    assert '"label":"car_blue"' in user_message['content']
    assert result['actions'][0]['type'] == 'pixel_navigation'
