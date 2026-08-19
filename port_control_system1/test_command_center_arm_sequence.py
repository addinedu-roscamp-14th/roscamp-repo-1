"""Cross-domain predecessor wiring for LLM ARM/vehicle plans."""

import types

import command_center


def test_db_plan_locations_always_include_ship_marker_slots():
    locations = command_center.inventory_plan_locations(['A-2-2', 'B-1'])

    assert 'A-2-2' in locations
    assert '선박-1' in locations
    assert '선박-6' in locations
    assert 'AMR1' in locations
    assert 'AMR2' in locations


def test_arm_command_becomes_predecessor_for_following_vehicle_step(
    monkeypatch,
):
    captured = {}

    class FakeClient:
        def send_arm_command(self, **kwargs):
            captured.update(kwargs)
            return {
                'command_id': 'arm-command-123',
                'mission_id': kwargs['mission_id'],
            }

    monkeypatch.setattr(command_center, 'CentralControlClient', FakeClient)
    popup = object.__new__(command_center.CommandPopup)
    popup._log = lambda _message: None
    popup.result_label = types.SimpleNamespace(configure=lambda **_kwargs: None)
    context = {
        'mission_id': 'mission-1',
        'predecessor_command_id': 'navigation-to-a',
    }

    handled = popup._execute_arm_action(
        {
            'type': 'arm_transfer_to_slot',
            'arm_id': 'arm2',
            'destination_slot': 'A-1-2',
            'vehicle_id': 'agv1',
            'final_for_vehicle': True,
        },
        context,
    )

    assert handled
    assert captured['vehicle_id'] == 'agv1'
    assert context['predecessor_command_id'] == 'arm-command-123'


def test_arm1_pick_place_uses_central_arm_queue(monkeypatch):
    captured = {}

    class FakeClient:
        def send_arm_command(self, **kwargs):
            captured.update(kwargs)
            return {
                'command_id': 'arm1-command-123',
                'mission_id': kwargs['mission_id'],
            }

    monkeypatch.setattr(command_center, 'CentralControlClient', FakeClient)
    popup = object.__new__(command_center.CommandPopup)
    popup._log = lambda _message: None
    popup.result_label = types.SimpleNamespace(configure=lambda **_kwargs: None)
    context = {
        'mission_id': 'mission-arm1',
        'predecessor_command_id': '',
    }

    handled = popup._execute_arm_action(
        {
            'type': 'arm1_pick_place',
            'arm_id': 'arm1',
            'source_id': 2,
            'destination_id': 9,
        },
        context,
    )

    assert handled
    assert captured['arm_id'] == 'arm1'
    assert captured['operation'] == 'pick_place'
    assert captured['source_id'] == 2
    assert captured['destination_id'] == 9
    assert context['predecessor_command_id'] == 'arm1-command-123'


def test_arm1_command_execution_repairs_amr1_trailer_to_id_ten(monkeypatch):
    captured = {}
    logs = []

    class FakeClient:
        def send_arm_command(self, **kwargs):
            captured.update(kwargs)
            return {
                'command_id': 'arm1-command-trailer',
                'mission_id': kwargs['mission_id'],
            }

    monkeypatch.setattr(command_center, 'CentralControlClient', FakeClient)
    popup = object.__new__(command_center.CommandPopup)
    popup._log = logs.append
    popup.result_label = types.SimpleNamespace(configure=lambda **_kwargs: None)

    handled = popup._execute_arm_action(
        {
            'type': 'arm1_pick_place',
            'arm_id': 'arm1',
            'source_id': 6,
            'destination_id': 9,
            'vehicle_id': 'agv1',
            'final_for_vehicle': True,
        },
        {'mission_id': 'mission-arm1', 'predecessor_command_id': ''},
    )

    assert handled
    assert captured['destination_id'] == 10
    assert any('9 -> 10' in item for item in logs)


def test_arm1_stop_uses_arm1_endpoint(monkeypatch):
    captured = []

    class FakeClient:
        def stop_arm(self, arm_id):
            captured.append(arm_id)
            return {'accepted': True, 'message': 'stopping'}

    monkeypatch.setattr(command_center, 'CentralControlClient', FakeClient)
    popup = object.__new__(command_center.CommandPopup)
    popup._log = lambda _message: None
    popup.result_label = types.SimpleNamespace(configure=lambda **_kwargs: None)

    handled = popup._execute_arm_action(
        {'type': 'arm_stop', 'arm_id': 'arm1'},
        {'mission_id': 'mission-arm1', 'predecessor_command_id': ''},
    )

    assert handled
    assert captured == ['arm1']


def test_park_command_carries_arm_predecessor(monkeypatch):
    captured = {}

    class FakeClient:
        def send_park(self, **kwargs):
            captured.update(kwargs)
            return {'command_id': 'park-after-arm'}

    monkeypatch.setattr(command_center, 'CentralControlClient', FakeClient)
    popup = object.__new__(command_center.CommandPopup)
    popup._log = lambda _message: None
    popup.result_label = types.SimpleNamespace(configure=lambda **_kwargs: None)
    context = {
        'mission_id': 'mission-1',
        'predecessor_command_id': 'arm-command-123',
    }

    handled = popup._handle_single_action(
        '',
        {'type': 'park_command', 'vehicle_id': 'agv1'},
        plan_context=context,
    )

    assert handled
    assert captured == {
        'vehicle_id': 'agv1',
        'predecessor_command_id': 'arm-command-123',
    }
    assert context['predecessor_command_id'] == 'park-after-arm'


def test_sequential_plan_stops_dispatching_after_local_step_failure(
    monkeypatch,
):
    monkeypatch.setattr(
        command_center, 'is_reciprocal_zone_exchange', lambda *_args: False
    )
    monkeypatch.setattr(
        command_center.RosControlBridge,
        'get_instance',
        lambda: types.SimpleNamespace(
            snapshot=lambda: types.SimpleNamespace(b1_zone='')
        ),
    )
    popup = object.__new__(command_center.CommandPopup)
    popup._log = lambda _message: None
    calls = []

    def handle(_command, action, *_args):
        context = _args[-1]
        calls.append(action['type'])
        context['step_failed'] = True
        return True

    popup._handle_single_action = handle

    handled = popup._handle_llm_result(
        'sequential plan',
        {
            'execution_mode': 'sequential',
            'actions': [
                {'type': 'visual_navigation'},
                {'type': 'arm_load_to_trailer'},
                {'type': 'park_command'},
            ],
        },
    )

    assert handled
    assert calls == ['visual_navigation']
