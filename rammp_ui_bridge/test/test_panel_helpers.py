"""Unit tests for the pure JSON-shaping/panel-building helpers in
ui_bridge_node -- no ROS2 or UE connection required.
"""

import json

from rammp_ui_bridge.ros.ui_bridge_node import panel_button_ids


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
