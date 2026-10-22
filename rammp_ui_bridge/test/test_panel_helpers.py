"""Unit tests for the pure JSON-shaping/panel-building helpers in
ui_bridge_node -- no ROS2 or UE connection required. Uses lightweight
duck-typed stand-ins for the rammp_orchestrator_interfaces message types
instead of constructing real ROS2 messages, keeping these tests runnable
with plain pytest.
"""

import json

from types import SimpleNamespace

from rammp_ui_bridge.ros.ui_bridge_node import (
    _activation_to_dict,
    _object_record_to_dict,
    _result_to_dict,
    build_panel,
    control_panel_buttons,
    panel_button_ids,
    selection_panel_buttons,
)


def test_object_record_to_dict_parses_pose_json():
    obj = SimpleNamespace(
        instance_id="fridge_1",
        perception_class="refrigerator",
        last_pose_json='{"x": 1.0, "y": 2.0, "theta": 0.0}',
        location="kitchen",
        opened=1,
        last_seen=123.456,
        box=[1.0, 2.0, 3.0, 4.0],
        stream_id="",
    )
    result = _object_record_to_dict(obj)
    assert result == {
        "instance_id": "fridge_1",
        "perception_class": "refrigerator",
        "last_pose": {"x": 1.0, "y": 2.0, "theta": 0.0},
        "location": "kitchen",
        "opened": 1,
        "last_seen": 123.456,
        "box": [1.0, 2.0, 3.0, 4.0],
        "stream_id": "",
    }


def test_object_record_to_dict_handles_empty_pose_json_and_box():
    obj = SimpleNamespace(
        instance_id="food_1",
        perception_class="food",
        last_pose_json="",
        location="",
        opened=-1,
        last_seen=0.0,
        box=[0.0, 0.0, 0.0, 0.0],
        stream_id="",
    )
    result = _object_record_to_dict(obj)
    assert result["last_pose"] == {}
    assert result["box"] == []  # all-zero box means "no box reported yet"


def test_object_record_to_dict_passes_through_stream_id():
    obj = SimpleNamespace(
        instance_id="handle_1",
        perception_class="handle",
        last_pose_json="",
        location="",
        opened=-1,
        last_seen=5.0,
        box=[10.0, 20.0, 30.0, 40.0],
        stream_id="articulation_wrist_cam",
    )
    result = _object_record_to_dict(obj)
    assert result["box"] == [10.0, 20.0, 30.0, 40.0]
    assert result["stream_id"] == "articulation_wrist_cam"


def test_activation_to_dict_round_trips_availability():
    activation = SimpleNamespace(
        engine_id="articulation",
        availability=[
            SimpleNamespace(
                action_id="open_door",
                available=True,
                reason="",
                suggested_params_json='{"speed": "slow"}',
            ),
            SimpleNamespace(
                action_id="close_door",
                available=False,
                reason="already closed",
                suggested_params_json="",
            ),
        ],
    )
    result = _activation_to_dict(activation)
    assert result["engine_id"] == "articulation"
    assert result["availability"] == [
        {
            "action_id": "open_door",
            "available": True,
            "reason": "",
            "suggested_params": {"speed": "slow"},
        },
        {
            "action_id": "close_door",
            "available": False,
            "reason": "already closed",
            "suggested_params": {},
        },
    ]


def test_result_to_dict_parses_payload_json():
    result = SimpleNamespace(
        status=0,
        payload_json='{"holding": "food_1"}',
        error_code="",
        recoverable=True,
        suggestions=["retry"],
    )
    assert _result_to_dict(result) == {
        "status": 0,
        "payload": {"holding": "food_1"},
        "error_code": "",
        "recoverable": True,
        "suggestions": ["retry"],
    }


def test_build_panel_round_trips_buttons_and_mapping():
    panel_json, mapping = build_panel(
        title="fridge_1",
        message="selected",
        buttons=[
            ("action:articulation:open_door", "open_door", "normal", {"kind": "execute_action"}),
            ("deselect", "Deselect", "normal", {"kind": "deselect"}),
        ],
    )
    payload = json.loads(panel_json)
    assert payload["title"] == "fridge_1"
    assert payload["message"] == "selected"
    assert payload["buttons"] == [
        {"button_id": "action:articulation:open_door", "label": "open_door", "style": "normal"},
        {"button_id": "deselect", "label": "Deselect", "style": "normal"},
    ]
    assert "progress" not in payload
    assert "style" not in payload
    assert mapping == {
        "action:articulation:open_door": {"kind": "execute_action"},
        "deselect": {"kind": "deselect"},
    }


def test_build_panel_includes_progress_and_error_style():
    panel_json, _mapping = build_panel(
        title="t", message="m", progress={"phase": "p", "pct": 50.0, "message": "m", "pausable": True},
        style="error",
    )
    payload = json.loads(panel_json)
    assert payload["progress"] == {"phase": "p", "pct": 50.0, "message": "m", "pausable": True}
    assert payload["style"] == "error"


def test_panel_button_ids_extracts_from_engine_supplied_panel():
    panel_json = json.dumps(
        {
            "title": "Confirm",
            "message": "Pick the item?",
            "buttons": [
                {"button_id": "yes", "label": "Yes", "style": "normal"},
                {"button_id": "no", "label": "No", "style": "normal"},
            ],
        }
    )
    assert panel_button_ids(panel_json) == frozenset({"yes", "no"})


def test_panel_button_ids_handles_empty_and_malformed_input():
    assert panel_button_ids("") == frozenset()
    assert panel_button_ids("not json") == frozenset()


def test_selection_panel_buttons_filters_unavailable_and_appends_deselect():
    activations = [
        SimpleNamespace(
            engine_id="articulation",
            availability=[
                SimpleNamespace(
                    action_id="open_door", available=True, reason="", suggested_params_json=""
                ),
                SimpleNamespace(
                    action_id="close_door",
                    available=False,
                    reason="already closed",
                    suggested_params_json="",
                ),
            ],
        )
    ]
    buttons = selection_panel_buttons(activations)
    button_ids = [b[0] for b in buttons]
    assert "action:articulation:open_door" in button_ids
    assert "action:articulation:close_door" not in button_ids
    assert "deselect" in button_ids


def test_control_panel_buttons_omits_pause_resume_when_not_pausable():
    buttons = control_panel_buttons(pausable=False)
    button_ids = [b[0] for b in buttons]
    assert button_ids == ["control:cancel"]


def test_control_panel_buttons_includes_pause_resume_when_pausable():
    buttons = control_panel_buttons(pausable=True)
    button_ids = [b[0] for b in buttons]
    assert button_ids == ["control:pause", "control:resume", "control:cancel"]
