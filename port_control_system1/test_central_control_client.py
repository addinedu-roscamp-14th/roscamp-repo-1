"""Tests for dashboard-to-gateway navigation requests."""

from central_control_client import CentralControlClient


def test_pixel_goal_contains_token_and_coordinates(monkeypatch):
    """The dashboard must send the VLM result and shared token unchanged."""
    captured = {}

    class FakeResponse:
        ok = True
        status_code = 200
        text = ''

        @staticmethod
        def json():
            return {
                'accepted': True,
                'duplicate': False,
                'command_id': 'vlm-test',
            }

    def fake_post(url, headers, json, timeout):
        captured.update({
            'url': url,
            'headers': headers,
            'json': json,
            'timeout': timeout,
        })
        return FakeResponse()

    monkeypatch.setattr(
        'central_control_client.requests.post',
        fake_post,
    )
    client = CentralControlClient(
        base_url='http://central:8100',
        token='porter1234',
    )
    result = client.send_pixel_goal(
        {'x': 320, 'y': 300},
        {'x': 380, 'y': 300},
        command_id='vlm-test',
        predecessor_command_id='vlm-before',
        mode='parking_b1',
        queue_if_busy=True,
    )

    assert result['accepted']
    assert captured['url'].endswith('/api/v1/navigation/pixel-goal')
    assert captured['headers']['X-Control-Token'] == 'porter1234'
    assert captured['json']['target'] == {'x': 320.0, 'y': 300.0}
    assert captured['json']['heading'] == {'x': 380.0, 'y': 300.0}
    assert captured['json']['mode'] == 'parking_b1'
    assert captured['json']['vehicle_id'] == ''
    assert captured['json']['zone_id'] == 'B-1'
    assert captured['json']['predecessor_command_id'] == 'vlm-before'
    assert captured['json']['queue_if_busy'] is True


def test_send_park_contains_token_and_vehicle_id(monkeypatch):
    """A park request must reach the dedicated parking endpoint with the token."""
    captured = {}

    class FakeResponse:
        ok = True
        status_code = 200
        text = ''

        @staticmethod
        def json():
            return {
                'accepted': True,
                'duplicate': False,
                'command_id': 'park-test',
                'vehicle_id': 'agv1',
            }

    def fake_post(url, headers, json, timeout):
        captured.update({
            'url': url,
            'headers': headers,
            'json': json,
            'timeout': timeout,
        })
        return FakeResponse()

    monkeypatch.setattr(
        'central_control_client.requests.post',
        fake_post,
    )
    client = CentralControlClient(
        base_url='http://central:8100',
        token='porter1234',
    )
    result = client.send_park(command_id='park-test', vehicle_id='agv1')

    assert result['accepted']
    assert captured['url'].endswith('/api/v1/navigation/park')
    assert captured['headers']['X-Control-Token'] == 'porter1234'
    assert captured['json']['command_id'] == 'park-test'
    assert captured['json']['vehicle_id'] == 'agv1'
