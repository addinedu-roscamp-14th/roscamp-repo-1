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


def test_pixel_goal_defaults_to_queueing_for_concurrent_control(monkeypatch):
    """Parallel control should preserve the earlier command instead of dropping it."""
    captured = {}

    class FakeResponse:
        ok = True
        status_code = 200
        text = ''

        @staticmethod
        def json():
            return {'accepted': True, 'duplicate': False, 'command_id': 'vlm-default'}

    def fake_post(url, headers, json, timeout):
        captured.update({'json': json})
        return FakeResponse()

    monkeypatch.setattr('central_control_client.requests.post', fake_post)
    client = CentralControlClient(base_url='http://central:8100', token='porter1234')

    client.send_pixel_goal(
        {'x': 320, 'y': 300},
        {'x': 380, 'y': 300},
        command_id='vlm-default',
    )

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
    result = client.send_park(
        command_id='park-test',
        vehicle_id='agv1',
        predecessor_command_id='arm-before-park',
    )

    assert result['accepted']
    assert captured['url'].endswith('/api/v1/navigation/park')
    assert captured['headers']['X-Control-Token'] == 'porter1234'
    assert captured['json']['command_id'] == 'park-test'
    assert captured['json']['vehicle_id'] == 'agv1'
    assert captured['json']['predecessor_command_id'] == 'arm-before-park'


def test_arm_command_uses_whitelisted_gateway_payload(monkeypatch):
    captured = {}

    class FakeResponse:
        ok = True
        status_code = 200
        text = ''

        @staticmethod
        def json():
            return {'accepted': True, 'command_id': 'arm-test'}

    def fake_request(method, url, headers, json, timeout):
        captured.update({
            'method': method,
            'url': url,
            'headers': headers,
            'json': json,
            'timeout': timeout,
        })
        return FakeResponse()

    monkeypatch.setattr(
        'central_control_client.requests.request',
        fake_request,
    )
    client = CentralControlClient(
        base_url='http://central:8100',
        token='porter1234',
    )

    result = client.send_arm_command(
        arm_id='arm2',
        operation='load_to_trailer',
        command_id='arm-test',
        mission_id='mission-test',
        source_id=3,
        vehicle_id='agv1',
        final_for_vehicle=True,
    )

    assert result['accepted']
    assert captured['method'] == 'POST'
    assert captured['url'].endswith('/api/v1/arms/commands')
    assert captured['headers']['X-Control-Token'] == 'porter1234'
    assert captured['json'] == {
        'command_id': 'arm-test',
        'mission_id': 'mission-test',
        'arm_id': 'arm2',
            'operation': 'load_to_trailer',
            'destination_slot': '',
            'destination_floor': 0,
            'source_id': 3,
        'destination_id': -1,
            'vehicle_id': 'agv1',
            'container_id': '',
            'final_for_vehicle': True,
    }


def test_arm_stop_uses_dedicated_endpoint(monkeypatch):
    captured = {}

    class FakeResponse:
        ok = True
        status_code = 200
        text = ''

        @staticmethod
        def json():
            return {'accepted': True}

    def fake_request(method, url, headers, json, timeout):
        captured.update({'method': method, 'url': url, 'json': json})
        return FakeResponse()

    monkeypatch.setattr(
        'central_control_client.requests.request',
        fake_request,
    )
    client = CentralControlClient(base_url='http://central:8100')

    assert client.stop_arm('arm2')['accepted']
    assert captured == {
        'method': 'POST',
        'url': 'http://central:8100/api/v1/arms/arm2/stop',
        'json': {},
    }


def test_arm1_stop_uses_arm1_endpoint(monkeypatch):
    captured = {}

    class FakeResponse:
        ok = True
        status_code = 200
        text = ''

        @staticmethod
        def json():
            return {'accepted': True}

    def fake_request(method, url, headers, json, timeout):
        captured.update({'method': method, 'url': url, 'json': json})
        return FakeResponse()

    monkeypatch.setattr(
        'central_control_client.requests.request', fake_request
    )
    client = CentralControlClient(base_url='http://central:8100')

    assert client.stop_arm('arm1')['accepted']
    assert captured['url'].endswith('/api/v1/arms/arm1/stop')
