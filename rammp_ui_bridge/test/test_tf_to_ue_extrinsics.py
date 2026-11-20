"""Unit tests for ``_tf_to_ue_extrinsics``'s optical-frame correction
(issue #8) -- pure math, no ROS2 graph/rclpy.init() needed.
"""

import math

from geometry_msgs.msg import TransformStamped

from rammp_ui_bridge.ros.ui_bridge_node import _OPTICAL_TO_BODY_QUAT, _tf_to_ue_extrinsics


def _transform(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)):
    msg = TransformStamped()
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (
        translation
    )
    (
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w,
    ) = rotation
    return msg


def test_optical_false_is_unchanged_body_frame_behavior():
    # Identity rotation, non-zero translation: optical=False (the default,
    # pre-issue-#8 behavior) should be untouched by the correction.
    transform = _transform(translation=(1.0, 2.0, 3.0))
    extrinsics = _tf_to_ue_extrinsics(transform)
    assert math.isclose(extrinsics["x"], 100.0)
    assert math.isclose(extrinsics["y"], -200.0)
    assert math.isclose(extrinsics["z"], 300.0)
    assert math.isclose(extrinsics["roll"], 0.0, abs_tol=1e-9)
    assert math.isclose(extrinsics["pitch"], 0.0, abs_tol=1e-9)
    assert math.isclose(extrinsics["yaw"], 0.0, abs_tol=1e-9)


def test_optical_true_cancels_the_standard_body_to_optical_rotation():
    # The body->optical static rotation's inverse is _OPTICAL_TO_BODY_QUAT
    # itself (it's the correction constant) -- a transform whose rotation
    # *is* that body->optical rotation (e.g. looking up TF straight from
    # the body frame to its own optical child frame) should therefore come
    # back as an identity rotation once optical=True undoes it, matching
    # issue #8's worked example ("scene_camera is x forward, z up" i.e.
    # zero relative rotation to the body frame).
    body_to_optical = (-0.5, 0.5, -0.5, 0.5)
    transform = _transform(rotation=body_to_optical)
    extrinsics = _tf_to_ue_extrinsics(transform, optical=True)
    assert math.isclose(extrinsics["roll"], 0.0, abs_tol=1e-6)
    assert math.isclose(extrinsics["pitch"], 0.0, abs_tol=1e-6)
    assert math.isclose(extrinsics["yaw"], 0.0, abs_tol=1e-6)


def test_optical_true_without_correction_would_have_been_wrong():
    # Sanity check that the correction actually matters: the raw
    # body->optical rotation fed through *without* optical=True (the old,
    # issue-#8 buggy behavior of treating an optical frame_id as if it
    # were already body-convention) does NOT come back as identity.
    body_to_optical = (-0.5, 0.5, -0.5, 0.5)
    transform = _transform(rotation=body_to_optical)
    extrinsics = _tf_to_ue_extrinsics(transform, optical=False)
    assert not math.isclose(extrinsics["roll"], 0.0, abs_tol=1e-3)


def test_optical_to_body_quat_is_a_unit_quaternion():
    x, y, z, w = _OPTICAL_TO_BODY_QUAT
    assert math.isclose(x * x + y * y + z * z + w * w, 1.0, rel_tol=1e-9)
