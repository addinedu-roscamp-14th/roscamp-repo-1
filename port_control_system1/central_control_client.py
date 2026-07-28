"""HTTP client for commands sent from the dashboard to the ROS gateway."""

from __future__ import annotations

import math
import os
import uuid

import requests


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
        mode='direct',
    ):
        """Send one VLM-selected target and heading pixel pair."""
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
                    'mode': mode,
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
