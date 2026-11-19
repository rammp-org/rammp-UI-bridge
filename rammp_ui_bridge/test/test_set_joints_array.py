"""Unit tests for UiBridgeNode._build_set_joints_array -- the pure,
no-ROS-graph-needed Gen1-compatible 53-element float array builder for the
SetJoints Remote Control call (§3.4). Exercised as a classmethod, so no
rclpy.init()/node instantiation is needed.
"""

import math

from sensor_msgs.msg import JointState

from rammp_ui_bridge.ros.ui_bridge_node import UiBridgeNode


def _joint_state(positions):
    msg = JointState()
    msg.position = list(positions)
    return msg


def test_build_set_joints_array_is_all_zero_without_any_joint_state():
    arr = UiBridgeNode._build_set_joints_array(None, None)
    assert len(arr) == 53
    assert arr == [0.0] * 53


def test_build_set_joints_array_arm_joints_converted_and_sign_flipped():
    # 7 arm joints (radians) + gripper (closed: > 0.9)
    arm = _joint_state([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 1.0])
    arr = UiBridgeNode._build_set_joints_array(arm, None)
    offset = UiBridgeNode._ARM_JOINT_INDEX_OFFSET
    expected_deg = [p * 180.0 / math.pi for p in arm.position[:7]]
    # joints 0, 2, 4, 5, 6 (0-indexed within the 7) are sign-flipped
    for i, flip in enumerate([True, False, True, False, True, True, True]):
        want = -expected_deg[i] if flip else expected_deg[i]
        assert math.isclose(arr[offset + i], want, rel_tol=1e-9)
    # gripper closed (position[7] > 0.9) -> both finger bones at 0.0
    assert arr[UiBridgeNode._ARM_GRIPPER_JOINT_LEFT_INDEX] == 0.0
    assert arr[UiBridgeNode._ARM_GRIPPER_JOINT_RIGHT_INDEX] == 0.0


def test_build_set_joints_array_gripper_open_maps_to_plus_minus_45():
    arm = _joint_state([0.0] * 7 + [0.0])  # gripper open (<= 0.9)
    arr = UiBridgeNode._build_set_joints_array(arm, None)
    assert arr[UiBridgeNode._ARM_GRIPPER_JOINT_LEFT_INDEX] == -45.0
    assert arr[UiBridgeNode._ARM_GRIPPER_JOINT_RIGHT_INDEX] == 45.0


def test_build_set_joints_array_short_arm_joint_state_is_ignored():
    # Fewer than 8 positions (no gripper joint) -- Gen1 leaves the whole
    # arm half of the array untouched (0.0) rather than guessing.
    arm = _joint_state([0.1] * 7)
    arr = UiBridgeNode._build_set_joints_array(arm, None)
    assert arr == [0.0] * 53


def test_build_set_joints_array_base_joints_converted():
    base = _joint_state([0.1, 0.2, 0.3, 0.4, 0.05, 0.06, 0.7, 0.8])
    arr = UiBridgeNode._build_set_joints_array(None, base)
    assert math.isclose(arr[UiBridgeNode._CHAIR_FC_JOINT_INDEX], base.position[0] * 180.0 / math.pi)
    assert math.isclose(arr[UiBridgeNode._CHAIR_RC_JOINT_INDEX], base.position[1] * 180.0 / math.pi)
    assert math.isclose(arr[UiBridgeNode._CHAIR_MR_JOINT_INDEX], base.position[2] * 180.0 / math.pi)
    assert math.isclose(arr[UiBridgeNode._CHAIR_ML_JOINT_INDEX], base.position[3] * 180.0 / math.pi)
    assert math.isclose(arr[UiBridgeNode._CHAIR_ML_CARRIAGE_INDEX], base.position[4] * 100.0)
    assert math.isclose(arr[UiBridgeNode._CHAIR_MR_CARRIAGE_INDEX], base.position[5] * 100.0)
    assert math.isclose(arr[UiBridgeNode._CHAIR_ML_WHEEL_INDEX], base.position[6] * 180.0 / math.pi)
    assert math.isclose(arr[UiBridgeNode._CHAIR_MR_WHEEL_INDEX], base.position[7] * 180.0 / math.pi)
