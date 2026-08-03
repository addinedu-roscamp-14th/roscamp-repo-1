"""HTTP client for commands sent from the dashboard to the ROS gateway."""

from __future__ import annotations

import math
import os
import uuid

import requests


_DEFAULT_ZONE_BY_MODE = {
    'parking_b1': 'B-1',
    'parking_a': 'A',
}


class CentralControlApiError(RuntimeError):
    """Raised when a command cannot be accepted by the central gateway."""


class CentralControlClient:
    """Send validated high-level commands to the central ROS laptop."""

    def __init__(self, base_url=None, token=None, timeout_sec=5.0):
        """Configure the gateway URL, shared token, and request timeout."""
        self.base_url = (
            base_url
            or os.environ.get(
                'PORT_CONTROL_API_URL',
                'http://127.0.0.1:8100',
            )
        ).rstrip('/')
        self.token = (
            token
            if token is not None
            else os.environ.get('PORT_CONTROL_API_TOKEN', '')
        )
        self.timeout_sec = float(timeout_sec)

    def send_pixel_goal(
        self,
        target,
        heading,
        command_id=None,
        predecessor_command_id='',
        mode='direct',
        vehicle_id='',
        zone_id='',
        zone_visually_empty=False,
        queue_if_busy=False,
    ):
        """Send one VLM-selected target and heading pixel pair.

        zone_visually_empty tells the gateway whether the client's current
        camera frame shows no vehicle in this zone - only used to
        auto-clear a zone lock whose owner has already gone offline.
        """
        target_payload = self._pixel_payload(target, 'target')
        heading_payload = self._pixel_payload(heading, 'heading')
        command_id = command_id or f'vlm-{uuid.uuid4()}'
        headers = {'Content-Type': 'application/json'}
        if self.token:
            headers['X-Control-Token'] = self.token

        try:
            response = requests.post(
                f'{self.base_url}/api/v1/navigation/pixel-goal',
                headers=headers,
                json={
                    'command_id': command_id,
                    'predecessor_command_id': str(
                        predecessor_command_id or ''
                    ),
                    'mode': mode,
                    'vehicle_id': vehicle_id,
                    'zone_id': zone_id or _DEFAULT_ZONE_BY_MODE.get(mode, ''),
                    'zone_visually_empty': bool(zone_visually_empty),
                    'queue_if_busy': bool(queue_if_busy),
                    'target': target_payload,
                    'heading': heading_payload,
                },
                timeout=self.timeout_sec,
            )
        except requests.RequestException as exc:
            raise CentralControlApiError(
                f'중앙제어 API에 연결할 수 없습니다: {exc}'
            ) from exc

        try:
            body = response.json()
        except ValueError:
            body = {'detail': response.text.strip() or 'empty response'}
        if not response.ok:
            raise CentralControlApiError(
                '중앙제어 API가 명령을 거부했습니다 '
                f'(HTTP {response.status_code}): '
                f'{body.get("detail", body)}'
            )
        return body

    def status(self):
        """Return both AGV states and the B-1 lock from the gateway."""
        headers = {}
        if self.token:
            headers['X-Control-Token'] = self.token
        response = requests.get(
            f'{self.base_url}/api/v1/status',
            headers=headers,
            timeout=self.timeout_sec,
        )
        response.raise_for_status()
        return response.json()

    def set_emergency(self, enabled=True, vehicle_id='fleet'):
        """Latch or release the fleet/per-vehicle emergency gate."""
        headers = {'Content-Type': 'application/json'}
        if self.token:
            headers['X-Control-Token'] = self.token
        response = requests.post(
            f'{self.base_url}/api/v1/emergency-stop',
            headers=headers,
            json={
                'vehicle_id': vehicle_id,
                'enabled': bool(enabled),
            },
            timeout=self.timeout_sec,
        )
        response.raise_for_status()
        return response.json()

    @staticmethod
    def _pixel_payload(value, field_name):
        if not isinstance(value, dict):
            raise CentralControlApiError(
                f'{field_name}은 x, y 객체여야 합니다'
            )
        result = {}
        for axis in ('x', 'y'):
            coordinate = value.get(axis)
            if (
                isinstance(coordinate, bool)
                or not isinstance(coordinate, (int, float))
                or not math.isfinite(float(coordinate))
            ):
                raise CentralControlApiError(
                    f'{field_name}.{axis}가 유효한 숫자가 아닙니다'
                )
            result[axis] = float(coordinate)
        return result
