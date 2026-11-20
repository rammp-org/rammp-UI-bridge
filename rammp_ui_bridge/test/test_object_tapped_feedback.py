"""Unit tests for object_tapped_feedback: what an engine is told about a
tap, in RequestUserInput feedback_json -- no ROS2 or UE connection required.
"""

import json

from rammp_ui_bridge.ros.ui_bridge_node import object_tapped_feedback


def test_a_tap_on_a_camera_image_forwards_where_it_landed():
    command = {
        "event_type": "object_tapped",
        "instance_id": "cup_1",
        "stream_id": "camera/scene/color",
        "tap_point": [412.5, 233.25],
        "image_size": [1280, 720],
        "seq": 8,
    }
    assert object_tapped_feedback(command) == {
        "event_type": "object_tapped",
        "instance_id": "cup_1",
        "stream_id": "camera/scene/color",
        "tap_point": [412.5, 233.25],
        "image_size": [1280, 720],
    }


def test_a_tap_in_the_3d_view_is_forwarded_as_before_plus_its_stream():
    # A world marker sends its detection's source as stream_id, and no image keys.
    assert object_tapped_feedback({"event_type": "object_tapped", "instance_id": "tap_3", "stream_id": "scene"}) == {
        "event_type": "object_tapped",
        "instance_id": "tap_3",
        "stream_id": "scene",
    }


def test_the_legacy_select_object_and_a_bare_tap_keep_the_original_shape():
    assert object_tapped_feedback({"cmd": "select_object", "instance_id": "apple_1"}) == {
        "event_type": "object_tapped",
        "instance_id": "apple_1",
    }


def test_internal_keys_stay_out_of_the_engine_contract():
    feedback = object_tapped_feedback({"event_type": "object_tapped", "instance_id": "x", "seq": 3, "cmd": "y"})
    assert set(feedback) == {"event_type", "instance_id"}


def test_the_point_is_always_floats_and_the_size_always_ints():
    # UE's JSON writer prints 640.0 as 640, so an integral point arrives as ints.
    for point, size in (([640, 360], [1280, 720]), ([640.5, 360.25], [1280.0, 720.0])):
        feedback = object_tapped_feedback(
            {"event_type": "object_tapped", "instance_id": "x", "tap_point": point, "image_size": size}
        )
        assert [type(v) for v in feedback["tap_point"]] == [float, float]
        assert [type(v) for v in feedback["image_size"]] == [int, int]
        assert json.dumps(feedback["image_size"]) == "[1280, 720]"


def test_a_point_without_a_usable_frame_size_is_not_forwarded():
    for size in (None, [0, 720], [1280], "1280x720", [1280, None]):
        command = {"event_type": "object_tapped", "instance_id": "x", "tap_point": [1.0, 2.0]}
        if size is not None:
            command["image_size"] = size
        assert "tap_point" not in object_tapped_feedback(command), size
        assert "image_size" not in object_tapped_feedback(command), size


def test_a_malformed_point_is_not_forwarded():
    for point in ([1.0], [1.0, "2"], [True, False], {"x": 1, "y": 2}, [float("nan"), 1.0]):
        command = {"event_type": "object_tapped", "instance_id": "x", "tap_point": point, "image_size": [640, 480]}
        assert "tap_point" not in object_tapped_feedback(command), point


def test_an_empty_or_non_string_stream_id_is_left_out():
    for stream_id in ("", None, 7):
        assert "stream_id" not in object_tapped_feedback(
            {"event_type": "object_tapped", "instance_id": "x", "stream_id": stream_id}
        )
