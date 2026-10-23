"""Thin rclpy bridge between an Unreal Engine touchscreen UI and the RAMMP
Orchestrator's public ROS2 interface (``rammp_orchestrator_interfaces``), per
``docs/NODE_INTERFACE.md``.

This plays exactly the role ``rammp_orchestrator.mocks.ui_node.UINode`` plays
in the Orchestrator's own test suite -- a client of
``~/get_world_state``/``~/get_detected_objects``/``~/select_object``/
``~/deselect``/``~/control``/``~/execute_action`` -- except driven by actual
UE user input instead of a scripted demo, and it pushes every response back
to UE instead of printing it.

**Governing principle (§5.1): UE only ever reports a tap.** UE sends exactly
two command shapes -- ``select_object`` (tapped an object) and
``button_press`` (tapped a button on the current ``InteractionPanel``) --
never a domain command (``execute_action``/``control``/``deselect``)
directly. This node owns the mapping from an opaque, bridge-assigned
``button_id`` to the Orchestrator call (plus params) it represents, rebuilt
every time a new panel is pushed (see ``_panel_buttons``).

**Engine-driven user input (§3.3.2).** This node also serves
``~/request_user_input`` (``rammp_ui_bridge_interfaces/RequestUserInput``),
called directly by an active engine node, bypassing the Orchestrator
entirely, and gated by the same ``ui_control`` arbitration grant as the
overlay camera below (a goal's ``token`` must match). A ``select_object``/
``button_press`` tap is resolved against an active goal *first*, if the
tapped id matches what that goal is offering; only otherwise does the
normal Orchestrator-call dispatch below apply. Tapping an object never
completes the goal by itself -- it's reported to the engine as feedback,
and the engine may call ``~/request_user_input/update``
(``rammp_ui_bridge_interfaces/UpdateUserInputPanel``) any number of times
while its goal is active to change the displayed panel/candidate
list/highlighted selection in response. Only a button press (or the
engine's own cancel) ends the goal.

By design this node never talks to engines about anything but this one
direct request/response exchange, and never duplicates Orchestrator logic --
it only (de)serializes JSON across the UE WebSocket on one side and ROS2
services/actions on the other, mirroring how ``ros/orchestrator_node.py`` is
a thin transport layer over the framework-agnostic ``core/orchestrator.py``.

Note: the Gen1 prototype GUI bridge (``rammp_prototype_gui.Gui_bridge``)
also streamed camera images/depth/masks to UE and mapped real robot joint
states into a UE skeleton (§5.2.A/B here), via a hardcoded 53-element
float array keyed by fixed skeleton-bone indices for its specific
chair+arm hardware (``set_ui_joints``). This node does **not** carry that
scheme forward -- §3.4/§5.2.B instead forwards every ``sensor_msgs/msg/
JointState`` arriving on ``arm_joint_states_topic`` (default
``/joint_states``) through **by name**, not by a hardcoded index layout
(see ``_on_arm_joint_states``/``_forward_arm_joint_states``), so it needs
no bridge-side knowledge of which joints exist or in what order -- UE's
own blueprint maps each joint name to its skeleton bone. This is the only
mock/real hardware node this bridge has wired up for §3.4 so far (one
topic, all of the arm's joints); chair/base telemetry (also part of
§5.2.B) still has no ROS2-side source. The scene camera (§3.2) also has
a real source -- this node subscribes directly to its raw color/depth
topics (see ``_on_scene_camera_color``/``_on_scene_camera_depth``) and
forwards every frame to UE as the §5.2.A base layer, over the RMSS
streaming protocol (``ramms_tools.streaming.StreamSender`` -- a TCP client
of UE's streaming server, separate from the Remote Control WebSocket,
since this is bulk pixel data). ``ramms_tools`` is a dependency, installed
via ``pip install git+https://github.com/rammp-org/ramms-tools.git`` (see
README.md), not vendored into this repo.

**Overlay-camera & user-input arbitration (§3.3.1/§3.3.2).** Every engine
with its own camera publishes to the *same* topic (``engine_overlay_topic``),
and any engine may call ``~/request_user_input`` directly -- so this node
also hosts a small ``rammp_common_interfaces`` arbitration server for one
shared "ui_control" resource covering both: an engine calls
``~/ui_control/acquire_control`` to get a token, stamps every
``EngineOverlayImage`` it publishes with it and includes it in its
``RequestUserInput`` goal, and this node only forwards overlay frames (or
accepts a user-input goal) whose token matches the one it most recently
granted (see ``_on_ui_acquire_control``/``_on_engine_overlay_frame``/
``_on_request_user_input_goal``). This is the same token-gating convention
``rammp_arm_interfaces``/``rammp_base_interfaces`` use for hardware, just
for a bridge-owned resource instead.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool

from rammp_common_interfaces.msg import ControlStatus
from rammp_common_interfaces.srv import AcquireControl, ReleaseControl
from rammp_orchestrator_interfaces.action import ExecuteAction
from rammp_orchestrator_interfaces.srv import (
    Control,
    Deselect,
    GetDetectedObjects,
    GetWorldState,
    SelectObject,
)
from rammp_ui_bridge_interfaces.action import RequestUserInput
from rammp_ui_bridge_interfaces.msg import EngineOverlayImage
from rammp_ui_bridge_interfaces.srv import UpdateUserInputPanel

from ramms_tools.streaming import StreamSender
from ..unreal_remote_websocket import UnrealRemoteWebsocket

_CONTROL_COMMANDS = {
    "PAUSE": Control.Request.PAUSE,
    "RESUME": Control.Request.RESUME,
    "CANCEL": Control.Request.CANCEL,
}

_DESELECT_BUTTON_ID = "deselect"
_DISMISS_BUTTON_ID = "dismiss"
_PAUSE_BUTTON_ID = "control:pause"
_RESUME_BUTTON_ID = "control:resume"
_CANCEL_BUTTON_ID = "control:cancel"

# §5.2.A base-layer camera stream: fixed RMSS channel numbers + stream IDs for
# the scene camera's two layers (one bridge, one scene camera -- no per-robot
# channel negotiation needed yet).
_SCENE_COLOR_CHANNEL = 0
_SCENE_DEPTH_CHANNEL = 1
_SCENE_COLOR_STREAM_ID = "scene/color"
_SCENE_DEPTH_STREAM_ID = "scene/depth"
_SCENE_STREAM_GROUP = "scene"

# §5.2.A overlay layer (§3.3.1): a single shared RMSS channel/stream_id for
# whichever engine is currently active's own camera -- see
# engine_overlay_topic below for why this is one topic, not per-engine.
_OVERLAY_COLOR_CHANNEL = 2
_OVERLAY_COLOR_STREAM_ID = "engine/overlay"
_OVERLAY_STREAM_GROUP = "overlay"

# ROS sensor_msgs/Image depth encodings this bridge knows how to forward
# as-is (uint16 mm, no conversion) -- see StreamSender.send_depth_uint16.
_UINT16_DEPTH_ENCODINGS = frozenset({"16UC1", "mono16"})


# ---------------------------------------------------------------------------
# Pure JSON-shaping helpers -- no ROS2/UE dependency, unit tested directly.
# ---------------------------------------------------------------------------

def _object_record_to_dict(obj) -> dict[str, Any]:
    """§5.2.D per-object shape. ``box``/``stream_id`` are the proposed
    ``ObjectRecord`` extension (§3.1's get_detected_objects note) -- box is
    an empty list when the object currently has no reported box."""
    box = list(obj.box) if any(obj.box) else []
    return {
        "instance_id": obj.instance_id,
        "perception_class": obj.perception_class,
        "last_pose": json.loads(obj.last_pose_json) if obj.last_pose_json else {},
        "location": obj.location,
        "opened": obj.opened,
        "last_seen": obj.last_seen,
        "box": box,
        "stream_id": obj.stream_id,
    }


def _activation_to_dict(activation) -> dict[str, Any]:
    return {
        "engine_id": activation.engine_id,
        "availability": [
            {
                "action_id": av.action_id,
                "available": av.available,
                "reason": av.reason,
                "suggested_params": json.loads(av.suggested_params_json)
                if av.suggested_params_json
                else {},
            }
            for av in activation.availability
        ],
    }


def _result_to_dict(result) -> dict[str, Any]:
    return {
        "status": result.status,
        "payload": json.loads(result.payload_json) if result.payload_json else {},
        "error_code": result.error_code,
        "recoverable": result.recoverable,
        "suggestions": list(result.suggestions),
    }


def build_panel(
    title: str = "",
    message: str = "",
    buttons: list[dict[str, str]] | None = None,
    progress: dict[str, Any] | None = None,
    style: str = "normal",
) -> tuple[str, dict[str, dict[str, Any]]]:
    """Builds one ``InteractionPanel`` (§5.2.C) JSON payload. Returns the
    JSON string to push to UE, and a fresh ``button_id -> mapping`` dict the
    caller should install as the new ``self._panel_buttons`` (replacing
    whatever was there before -- every panel push fully replaces the prior
    one, it's never a diff)."""
    buttons = buttons or []
    panel_buttons = []
    mapping: dict[str, dict[str, Any]] = {}
    for button_id, label, button_style, entry in buttons:
        panel_buttons.append({"button_id": button_id, "label": label, "style": button_style})
        mapping[button_id] = entry
    payload: dict[str, Any] = {
        "title": title,
        "message": message,
        "buttons": panel_buttons,
    }
    if progress is not None:
        payload["progress"] = progress
    if style != "normal":
        payload["style"] = style
    return json.dumps(payload), mapping


def panel_button_ids(panel_json: str) -> frozenset[str]:
    """Extracts the set of ``button_id``s an (engine-supplied, §3.3.2)
    ``InteractionPanel`` JSON payload offers, for dispatch-priority matching
    (§5.1) -- independent of this bridge's own ``_panel_buttons`` mapping,
    since an engine's panel buttons resolve against its own
    ``RequestUserInput`` goal, not an Orchestrator call."""
    if not panel_json:
        return frozenset()
    try:
        payload = json.loads(panel_json)
    except (TypeError, ValueError):
        return frozenset()
    return frozenset(b.get("button_id", "") for b in payload.get("buttons", []) if b.get("button_id"))


def selection_panel_buttons(activations) -> list[tuple[str, str, str, dict[str, Any]]]:
    """Builds the button list for a just-selected object: one button per
    available action across every claiming engine (§3.1's ``EngineActivation``),
    plus a ``deselect`` button. Label falls back to ``action_id`` -- see
    §7's open TBD on ``ActionAvailability`` missing ``ActionSpec.label``."""
    buttons: list[tuple[str, str, str, dict[str, Any]]] = []
    for activation in activations:
        for av in activation.availability:
            if not av.available:
                continue
            button_id = f"action:{activation.engine_id}:{av.action_id}"
            params = json.loads(av.suggested_params_json) if av.suggested_params_json else {}
            buttons.append((
                button_id,
                av.action_id,
                "normal",
                {
                    "kind": "execute_action",
                    "engine_id": activation.engine_id,
                    "action_id": av.action_id,
                    "params": params,
                },
            ))
    buttons.append((_DESELECT_BUTTON_ID, "Deselect", "normal", {"kind": "deselect"}))
    return buttons


def control_panel_buttons(pausable: bool) -> list[tuple[str, str, str, dict[str, Any]]]:
    buttons: list[tuple[str, str, str, dict[str, Any]]] = []
    if pausable:
        buttons.append((_PAUSE_BUTTON_ID, "Pause", "normal", {"kind": "control", "command": "PAUSE"}))
        buttons.append((_RESUME_BUTTON_ID, "Resume", "normal", {"kind": "control", "command": "RESUME"}))
    buttons.append((_CANCEL_BUTTON_ID, "Cancel", "warning", {"kind": "control", "command": "CANCEL"}))
    return buttons


class UiBridgeNode(Node):
    """Bridges one UE instance to one Orchestrator instance.

    All Orchestrator calls are made asynchronously from ROS2 callback
    threads (UE WebSocket user-input callback, ROS timers); none of this
    node's methods block the executor beyond the bounded waits already
    built into ``rclpy`` futures/actions. ``~/request_user_input``'s
    execute callback is the one exception -- it blocks its own (reentrant)
    callback-group thread until a matching UE tap or cancellation, exactly
    as long as the calling engine's goal is outstanding.
    """

    def __init__(self) -> None:
        super().__init__("ui_bridge_node")

        self.declare_parameter("ue_host", "127.0.0.1")
        self.declare_parameter("ue_ws_port", 30020)
        self.declare_parameter("ue_preset", "RAMMP")
        self.declare_parameter("orchestrator_ns", "/rammp_orchestrator")
        self.declare_parameter("world_state_poll_period", 0.5)
        self.declare_parameter("detected_objects_poll_period", 0.5)
        self.declare_parameter("connection_publish_period", 1.0)
        self.declare_parameter("service_wait_timeout", 5.0)
        # §3.2 Bridge <-> Camera: the scene camera's raw color/depth topics.
        # Depth's topic name depends on which scene_camera alternative a
        # given robot profile selects (see rammp-deployments'
        # sheppy-manifest.yaml) -- the Orbbec Gemini 336L publishes
        # "depth/image_raw", the OAK-D Pro W publishes
        # "depth/image_rect_raw" (the current flatbot profile default
        # below). Override per-robot via a launch/params-file argument.
        self.declare_parameter("scene_camera_color_topic", "/scene_camera/color/image_raw")
        self.declare_parameter("scene_camera_depth_topic", "/scene_camera/depth/image_rect_raw")
        # §3.3.1/§5.2.A overlay layer: one fixed, shared topic every engine
        # publishes its own camera to -- not a per-engine topic/parameter
        # table. Only one engine can ever be *active* at a time (the
        # Orchestrator's resource-locking already guarantees this, see
        # core/orchestrator.py's resource acquisition on activate/execute),
        # but a stale/crashed engine could still have a publisher open, so
        # frames also carry an arbitration token (EngineOverlayImage.token)
        # this node grants/checks itself -- see the ui_control_* services
        # below and _on_engine_overlay_frame.
        self.declare_parameter("engine_overlay_topic", "/engine_overlay/image_raw")
        # §5.2.A base layer: UE's RMSS streaming server port (separate TCP
        # socket from the Remote Control WebSocket above).
        self.declare_parameter("ue_stream_port", 30030)
        self.declare_parameter("ue_stream_reconnect_period", 3.0)
        # §3.4 Bridge <-> Hardware node / §5.2.B system status: the arm
        # driver's (or its mock's) sensor_msgs/msg/JointState topic, covering
        # every arm joint -- see sheppy's `arm` node (rammp-deployments'
        # december_2026/sheppy-manifest.yaml), whose mock alternative
        # (arm_mock) publishes this same topic name with empty name/position
        # arrays for wiring, and whose real alternative (kinova_gen3_driver)
        # publishes the Kinova Gen3's 7 arm joints plus the 2F-85 gripper's
        # own joint. Forwarded to UE by name (_on_arm_joint_states), not a
        # hardcoded index layout -- see the module docstring.
        self.declare_parameter("arm_joint_states_topic", "/joint_states")

        self._ns = self.get_parameter("orchestrator_ns").value
        self._service_wait_timeout = (
            self.get_parameter("service_wait_timeout").get_parameter_value().double_value
        )

        self._cb_group = ReentrantCallbackGroup()
        self._last_world_state_version: int | None = None
        self._last_detected_ids: frozenset[str] = frozenset()

        # -- panel/selection state ------------------------------------------------
        # button_id -> {"kind": "execute_action"/"control"/"deselect"/"dismiss", ...}
        # for whatever InteractionPanel this bridge itself most recently built
        # (not an engine-supplied one, see _active_input_goal below).
        self._panel_buttons: dict[str, dict[str, Any]] = {}
        self._selected_instance_id: str = ""
        self._selected_activations: list = []
        # Set while a RequestUserInput goal from an engine is active; see
        # _on_request_user_input_execute. None when no engine is waiting on
        # user input right now. Guarded by _active_input_lock since it's
        # mutated from the UE-callback thread (object taps/button presses,
        # see _try_resolve_active_input_goal), the ~/request_user_input/update
        # service thread (_on_update_user_input_panel), and read from the
        # action's own execute-callback thread.
        self._active_input_lock = threading.Lock()
        self._active_input_goal: dict[str, Any] | None = None

        # -- scene camera state (§3.2) ----------------------------------------
        # Latest frame from each topic, forwarded to UE as the §5.2.A base
        # layer (see _forward_scene_camera_frame). Guarded by a lock since
        # subscription callbacks run on the (multi-threaded) executor, not
        # necessarily the same thread.
        self._camera_lock = threading.Lock()
        self._latest_scene_color: Image | None = None
        self._latest_scene_depth: Image | None = None
        self._scene_color_frame_count = 0
        self._scene_depth_frame_count = 0

        # -- engine overlay camera state (§3.3.1) -----------------------------
        # Latest frame from the single shared overlay topic (see
        # engine_overlay_topic above), forwarded to UE as the §5.2.A overlay
        # layer (see _forward_engine_overlay_color). Same lock as the scene
        # camera state above -- all three subscriptions can run on different
        # executor threads.
        self._latest_engine_overlay: Image | None = None
        self._engine_overlay_frame_count = 0

        # -- arm joint-state telemetry (§3.4/§5.2.B) --------------------------
        # No "latest frame" buffer/lock needed like the cameras above -- each
        # JointState is forwarded to UE immediately as a single Remote
        # Control call (small structured data, not a streamed frame), so
        # there's nothing to hold between messages except the frame counter
        # used for throttled logging.
        self._arm_joint_states_frame_count = 0

        # -- ui-control arbitration state (§3.3.1/§3.3.2) ----------------------
        # This node is the arbitration *server* for the "UI control"
        # resource -- whichever engine currently holds it is the one
        # allowed to (a) have its EngineOverlayImage frames forwarded to UE
        # and (b) hold an active RequestUserInput goal; both are facets of
        # "this engine is the one currently talking to the UI", so they
        # share one grant rather than two independent ones. Uses
        # rammp_common_interfaces' AcquireControl/ReleaseControl/
        # ControlStatus protocol -- same one rammp_arm_interfaces/
        # rammp_base_interfaces use for hardware, see that repo's README.
        # Guarded by _ui_control_lock since the service callbacks and the
        # overlay subscription callback can run on different executor
        # threads. `_ui_token` is all-zero bytes when nobody holds it;
        # AcquireControl.Request.token/ReleaseControl.Request.token are
        # uint8[16], so this stays a 16-byte value throughout. `rejected_count`
        # mirrors ControlStatus's own field: frames/calls refused because
        # their token didn't match the current grant.
        self._ui_control_lock = threading.Lock()
        self._ui_owner_id: str = ""
        self._ui_token: bytes = bytes(16)
        self._ui_generation = 0
        self._ui_rejected_count = 0

        # §5.2.A: TCP client of UE's RMSS streaming server. Connection is
        # managed by a dedicated reconnect-loop thread (_stream_reconnect_loop)
        # since StreamSender itself has no built-in retry -- mirrors how
        # UnrealRemoteWebsocket manages its own WebSocket reconnection.
        self._stream_sender = StreamSender(
            host=self.get_parameter("ue_host").value,
            port=self.get_parameter("ue_stream_port").get_parameter_value().integer_value,
        )
        self._stream_reconnect_period = (
            self.get_parameter("ue_stream_reconnect_period").get_parameter_value().double_value
        )
        self._stream_shutdown = threading.Event()
        self._stream_reconnect_thread = threading.Thread(
            target=self._stream_reconnect_loop, name="ui_bridge-stream-reconnect", daemon=True
        )
        self._stream_reconnect_thread.start()

        self._get_world_state_client = self.create_client(
            GetWorldState, f"{self._ns}/get_world_state", callback_group=self._cb_group
        )
        self._get_detected_objects_client = self.create_client(
            GetDetectedObjects,
            f"{self._ns}/get_detected_objects",
            callback_group=self._cb_group,
        )
        self._select_object_client = self.create_client(
            SelectObject, f"{self._ns}/select_object", callback_group=self._cb_group
        )
        self._deselect_client = self.create_client(
            Deselect, f"{self._ns}/deselect", callback_group=self._cb_group
        )
        self._control_client = self.create_client(
            Control, f"{self._ns}/control", callback_group=self._cb_group
        )
        self._execute_action_client = ActionClient(
            self, ExecuteAction, f"{self._ns}/execute_action", callback_group=self._cb_group
        )

        self._request_user_input_server = ActionServer(
            self,
            RequestUserInput,
            "~/request_user_input",
            execute_callback=self._on_request_user_input_execute,
            goal_callback=self._on_request_user_input_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self._update_user_input_panel_server = self.create_service(
            UpdateUserInputPanel,
            "~/request_user_input/update",
            self._on_update_user_input_panel,
            callback_group=self._cb_group,
        )

        # §3.3.1/§3.3.2 ui-control arbitration server: any engine calls these
        # before/after publishing to engine_overlay_topic and/or holding a
        # RequestUserInput goal. Latched (transient_local) ControlStatus
        # publisher so a late-starting engine learns the current grant
        # immediately, same convention as rammp_common_interfaces' own
        # ControlStatus doc comment.
        self._ui_acquire_control_server = self.create_service(
            AcquireControl,
            "~/ui_control/acquire_control",
            self._on_ui_acquire_control,
            callback_group=self._cb_group,
        )
        self._ui_release_control_server = self.create_service(
            ReleaseControl,
            "~/ui_control/release_control",
            self._on_ui_release_control,
            callback_group=self._cb_group,
        )
        self._ui_control_status_publisher = self.create_publisher(
            ControlStatus,
            "~/ui_control/status",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self._publish_ui_control_status()

        self._connection_publisher = self.create_publisher(Bool, "~/ue_connection", 10)
        self.create_timer(
            self.get_parameter("connection_publish_period").value,
            self._publish_connection_status,
        )
        self.create_timer(
            self.get_parameter("world_state_poll_period").value,
            self._poll_world_state,
            callback_group=self._cb_group,
        )
        self.create_timer(
            self.get_parameter("detected_objects_poll_period").value,
            self._poll_detected_objects,
            callback_group=self._cb_group,
        )

        self.ue = UnrealRemoteWebsocket(
            host=self.get_parameter("ue_host").value,
            ws_port=self.get_parameter("ue_ws_port").get_parameter_value().integer_value,
            preset=self.get_parameter("ue_preset").value,
            user_input_callback=self._on_ue_user_input,
        )

        self._scene_color_sub = self.create_subscription(
            Image,
            self.get_parameter("scene_camera_color_topic").value,
            self._on_scene_camera_color,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self._scene_depth_sub = self.create_subscription(
            Image,
            self.get_parameter("scene_camera_depth_topic").value,
            self._on_scene_camera_depth,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self._engine_overlay_sub = self.create_subscription(
            EngineOverlayImage,
            self.get_parameter("engine_overlay_topic").value,
            self._on_engine_overlay_frame,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self._arm_joint_states_sub = self.create_subscription(
            JointState,
            self.get_parameter("arm_joint_states_topic").value,
            self._on_arm_joint_states,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )

        self._push_idle_panel()

        self.get_logger().info(
            f"rammp_ui_bridge ready; bridging UE at "
            f"{self.get_parameter('ue_host').value} <-> Orchestrator at '{self._ns}'; "
            f"scene camera color='{self.get_parameter('scene_camera_color_topic').value}' "
            f"depth='{self.get_parameter('scene_camera_depth_topic').value}' "
            f"engine overlay='{self.get_parameter('engine_overlay_topic').value}' "
            f"arm joint states='{self.get_parameter('arm_joint_states_topic').value}' -> "
            f"UE stream port {self.get_parameter('ue_stream_port').value}"
        )

    # ------------------------------------------------------------------
    # Bridge <-> Camera (§3.2): raw scene-camera frames, forwarded to UE as
    # the §5.2.A base layer over the RMSS streaming protocol.
    #
    # Lands the latest frame (plus a log line on the first frame and then
    # one every 100, to confirm wiring without spamming), then forwards it
    # to UE if currently connected -- silently skipped (frame dropped, not
    # queued) while disconnected, since only the newest frame matters for a
    # live camera feed and _stream_reconnect_loop is already working on
    # reconnecting.
    # ------------------------------------------------------------------
    def _on_scene_camera_color(self, msg: Image) -> None:
        with self._camera_lock:
            self._latest_scene_color = msg
            self._scene_color_frame_count += 1
            count = self._scene_color_frame_count
        if count == 1 or count % 100 == 0:
            self.get_logger().info(
                f"scene camera color: frame #{count} ({msg.width}x{msg.height}, "
                f"encoding={msg.encoding})"
            )
        self._forward_scene_camera_color(msg)

    def _on_scene_camera_depth(self, msg: Image) -> None:
        with self._camera_lock:
            self._latest_scene_depth = msg
            self._scene_depth_frame_count += 1
            count = self._scene_depth_frame_count
        if count == 1 or count % 100 == 0:
            self.get_logger().info(
                f"scene camera depth: frame #{count} ({msg.width}x{msg.height}, "
                f"encoding={msg.encoding})"
            )
        self._forward_scene_camera_depth(msg)

    def _forward_scene_camera_color(self, msg: Image) -> None:
        if not self._stream_sender.is_connected():
            return
        try:
            self._stream_sender.send_image(
                _SCENE_COLOR_CHANNEL,
                bytes(msg.data),
                msg.width,
                msg.height,
                fmt=msg.encoding,
                group=_SCENE_STREAM_GROUP,
                role="color",
                stream_id=_SCENE_COLOR_STREAM_ID,
                name="Scene camera",
            )
        except (RuntimeError, OSError) as exc:
            self.get_logger().warning(f"failed to forward scene camera color frame to UE: {exc}")

    def _forward_scene_camera_depth(self, msg: Image) -> None:
        if not self._stream_sender.is_connected():
            return
        try:
            if msg.encoding in _UINT16_DEPTH_ENCODINGS:
                self._stream_sender.send_depth_uint16(
                    _SCENE_DEPTH_CHANNEL,
                    bytes(msg.data),
                    msg.width,
                    msg.height,
                    group=_SCENE_STREAM_GROUP,
                    role="depth",
                    stream_id=_SCENE_DEPTH_STREAM_ID,
                    name="Scene camera depth",
                )
            else:
                # 32FC1 (metres) and anything else not explicitly a uint16 mm
                # encoding: pass through as generic image data, let UE/§7
                # sort out interpretation once a real depth source confirms
                # which encodings actually show up in practice.
                self._stream_sender.send_image(
                    _SCENE_DEPTH_CHANNEL,
                    bytes(msg.data),
                    msg.width,
                    msg.height,
                    fmt=msg.encoding,
                    group=_SCENE_STREAM_GROUP,
                    role="depth",
                    stream_id=_SCENE_DEPTH_STREAM_ID,
                    name="Scene camera depth",
                )
        except (RuntimeError, OSError) as exc:
            self.get_logger().warning(f"failed to forward scene camera depth frame to UE: {exc}")

    # ------------------------------------------------------------------
    # Bridge <-> Hardware node (§3.4): arm joint-state telemetry, forwarded
    # to UE as the §5.2.B system-status channel.
    #
    # Always-on and independent of engine selection (unlike §3.3's
    # per-engine overlay camera) -- the arm driver (or its sheppy mock,
    # arm_mock) publishes sensor_msgs/msg/JointState on a single fixed
    # topic covering every arm joint, no per-robot topic table needed.
    # Forwarded by name, not a hardcoded skeleton-bone index array like the
    # Gen1 prototype's set_ui_joints -- UE's own blueprint resolves each
    # joint name to its bone. Positions are converted radians -> degrees,
    # matching Gen1's convention (UE skeletons author joint angles in
    # degrees). An empty JointState (name=[], position=[]) -- e.g. from
    # arm_mock, which exists only to keep this topic wired up on a
    # workstation profile -- forwards as empty arrays; UE is expected to
    # simply leave the skeleton unchanged when it receives no joints.
    # ------------------------------------------------------------------
    def _on_arm_joint_states(self, msg: JointState) -> None:
        self._arm_joint_states_frame_count += 1
        count = self._arm_joint_states_frame_count
        if count == 1 or count % 100 == 0:
            self.get_logger().info(
                f"arm joint states: frame #{count} ({len(msg.name)} joint(s))"
            )
        self._forward_arm_joint_states(msg)

    def _forward_arm_joint_states(self, msg: JointState) -> None:
        if not self.ue.is_connected():
            return
        try:
            positions_deg = [p * 180.0 / math.pi for p in msg.position]
            self.ue.call_function(
                "UpdateArmJointStates",
                {"Names": list(msg.name), "PositionsDeg": positions_deg},
            )
        except (RuntimeError, OSError) as exc:
            self.get_logger().warning(f"failed to forward arm joint states to UE: {exc}")

    # ------------------------------------------------------------------
    # Bridge <-> Engine node (§3.3.1): a single shared overlay-camera topic,
    # and (§3.3.2 below) direct engine<->bridge user-input requests -- both
    # gated by the same "ui_control" arbitration grant.
    #
    # Every engine publishes its own camera (if it has one) to the same
    # well-known topic (engine_overlay_topic) rather than each getting a
    # distinct topic name/parameter. The Orchestrator's resource-locking
    # already means only one engine can ever hold execution resources at a
    # time, but a crashed/slow-to-release engine could still have a stale
    # publisher open, so every frame also carries an arbitration token
    # (EngineOverlayImage.token) -- the bridge is the arbitration *server*
    # for this resource (see the ui_control_* services below) and only
    # forwards frames whose token matches the one it most recently granted.
    # ------------------------------------------------------------------
    def _on_ui_acquire_control(
        self, request: AcquireControl.Request, response: AcquireControl.Response
    ) -> AcquireControl.Response:
        with self._ui_control_lock:
            # AcquireControl.srv's "pre-generated key" field: a non-all-zero
            # request.token means the caller (by contract, the Orchestrator)
            # supplied its own session token to be shared across several
            # resources rather than asking this one to mint its own -- grant
            # exactly that token instead. All-zero (the default, and what
            # any pre-this-field caller always sends) keeps today's
            # behaviour: mint a fresh one.
            requested_token = bytes(request.token)
            if requested_token != bytes(16):
                self._ui_token = requested_token
            else:
                self._ui_token = os.urandom(16)
            self._ui_owner_id = request.owner_id
            self._ui_generation += 1
            response.accepted = True
            response.token = list(self._ui_token)
            response.generation = self._ui_generation
            response.message = f"granted to '{request.owner_id}'"
        self.get_logger().info(
            f"ui control acquired by '{request.owner_id}' "
            f"(generation {self._ui_generation})"
        )
        self._publish_ui_control_status()
        return response

    def _on_ui_release_control(
        self, request: ReleaseControl.Request, response: ReleaseControl.Response
    ) -> ReleaseControl.Response:
        with self._ui_control_lock:
            if bytes(request.token) == self._ui_token and self._ui_owner_id:
                owner = self._ui_owner_id
                self._ui_owner_id = ""
                self._ui_token = bytes(16)
                response.released = True
                response.message = ""
            else:
                owner = None
                self._ui_rejected_count += 1
                response.released = False
                response.message = "token mismatch or already released"
        if owner is not None:
            self.get_logger().info(f"ui control released by '{owner}'")
            self._publish_ui_control_status()
        return response

    def _publish_ui_control_status(self) -> None:
        with self._ui_control_lock:
            msg = ControlStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.arbitration_enabled = True
            msg.estopped = False
            msg.owned = bool(self._ui_owner_id)
            msg.owner_id = self._ui_owner_id
            msg.generation = self._ui_generation
            msg.rejected_count = self._ui_rejected_count
        self._ui_control_status_publisher.publish(msg)

    def _on_engine_overlay_frame(self, msg: EngineOverlayImage) -> None:
        with self._ui_control_lock:
            granted = bool(self._ui_owner_id) and bytes(msg.token) == self._ui_token
            if not granted:
                self._ui_rejected_count += 1
                rejected = self._ui_rejected_count
        if not granted:
            if rejected == 1 or rejected % 100 == 0:
                self.get_logger().warning(
                    f"engine overlay frame dropped: token doesn't match the current "
                    f"ui_control grant (rejected #{rejected})"
                )
            return

        image = msg.image
        with self._camera_lock:
            self._latest_engine_overlay = image
            self._engine_overlay_frame_count += 1
            count = self._engine_overlay_frame_count
        if count == 1 or count % 100 == 0:
            self.get_logger().info(
                f"engine overlay camera: frame #{count} ({image.width}x{image.height}, "
                f"encoding={image.encoding})"
            )
        self._forward_engine_overlay_color(image)

    def _forward_engine_overlay_color(self, msg: Image) -> None:
        if not self._stream_sender.is_connected():
            return
        try:
            self._stream_sender.send_image(
                _OVERLAY_COLOR_CHANNEL,
                bytes(msg.data),
                msg.width,
                msg.height,
                fmt=msg.encoding,
                group=_OVERLAY_STREAM_GROUP,
                role="color",
                stream_id=_OVERLAY_COLOR_STREAM_ID,
                name="Engine overlay camera",
            )
        except (RuntimeError, OSError) as exc:
            self.get_logger().warning(f"failed to forward engine overlay frame to UE: {exc}")

    def _stream_reconnect_loop(self) -> None:
        """Background thread: keeps ``self._stream_sender`` connected to
        UE's RMSS streaming server, independent of the camera subscription
        callbacks above -- mirrors ``UnrealRemoteWebsocket``'s own
        reconnect loop for the Remote Control WebSocket, just thread-based
        instead of asyncio-based since ``StreamSender`` is a plain blocking
        TCP client."""
        while not self._stream_shutdown.is_set():
            if not self._stream_sender.is_connected():
                try:
                    self._stream_sender.connect()
                    self.get_logger().info("connected to UE streaming server")
                except OSError as exc:
                    self.get_logger().debug(f"UE streaming server connect failed: {exc}")
            self._stream_shutdown.wait(self._stream_reconnect_period)

    # ------------------------------------------------------------------
    # UE -> Orchestrator / UE -> active engine: dispatch user-input commands
    # ------------------------------------------------------------------
    def _on_ue_user_input(self, raw: str) -> None:
        """Invoked (on the UE WebSocket's asyncio thread) whenever UE's
        "UserInput" preset property changes -- see §5.1 for the expected
        JSON command schema. Only ``select_object``/``button_press`` are
        valid UE-originated commands; everything else (action execution,
        pause/resume/cancel, deselect) is this bridge's own internal
        button-mapping lookup (§5.1's "governing principle")."""
        try:
            command = json.loads(raw)
        except (TypeError, ValueError):
            self.get_logger().error(f"ignoring malformed UserInput (not JSON): {raw!r}")
            self._push_error_panel(f"malformed UserInput: {raw!r}")
            return

        cmd = command.get("cmd")

        # Dispatch priority (§5.1): an active engine input request wins
        # first, if the tapped id matches something it's offering.
        if self._active_input_goal is not None:
            if self._try_resolve_active_input_goal(cmd, command):
                return

        handler = {
            "select_object": self._handle_select_object,
            "button_press": self._handle_button_press,
        }.get(cmd)
        if handler is None:
            self.get_logger().error(f"ignoring UserInput with unknown cmd: {cmd!r}")
            self._push_error_panel(f"unknown cmd: {cmd!r}")
            return
        try:
            handler(command)
        except Exception as e:  # noqa: BLE001 -- isolate one bad command from the bridge
            self.get_logger().error(f"error handling UserInput cmd '{cmd}': {e}")
            self._push_error_panel(str(e))

    def _try_resolve_active_input_goal(self, cmd: str, command: dict[str, Any]) -> bool:
        """Returns True if the tap matched the active ``RequestUserInput``
        goal and was consumed by it (the normal dispatch table must not
        also run for it). A matched ``select_object`` does NOT end the
        goal -- the user may tap several different objects in a row while
        deciding -- it's reported to the engine as feedback instead
        (``RequestUserInput.Feedback.EVENT_OBJECT_TAPPED``), and
        ``_active_input_goal``'s own ``selected_instance_id`` is updated so
        a freshly-pushed panel/objects (e.g. from
        ``_on_update_user_input_panel``) reflects it. Only a matched
        ``button_press`` ends the goal."""
        with self._active_input_lock:
            goal_state = self._active_input_goal
            if goal_state is None:
                return False
            if cmd == "select_object":
                instance_id = command.get("instance_id", "")
                if instance_id not in goal_state["object_ids"]:
                    return False
                goal_state["selected_instance_id"] = instance_id
                goal_handle = goal_state["goal_handle"]
            elif cmd == "button_press":
                button_id = command.get("button_id", "")
                if button_id not in goal_state["button_ids"]:
                    return False
                goal_state["result"]["status"] = RequestUserInput.Result.STATUS_BUTTON
                goal_state["result"]["button_id"] = button_id
                goal_state["event"].set()
                return True
            else:
                return False
        feedback = RequestUserInput.Feedback()
        feedback.event_type = RequestUserInput.Feedback.EVENT_OBJECT_TAPPED
        feedback.instance_id = instance_id
        goal_handle.publish_feedback(feedback)
        return True

    def _handle_select_object(self, command: dict[str, Any]) -> None:
        instance_id = command.get("instance_id", "")
        # §6: a tap on an instance_id the bridge no longer recognizes must
        # be reported, not silently forwarded or dropped.
        if instance_id not in self._last_detected_ids:
            self._push_error_panel(f"'{instance_id}' is no longer detected")
            return
        if not self._select_object_client.wait_for_service(
            timeout_sec=self._service_wait_timeout
        ):
            self._report_service_unavailable("select_object")
            return
        req = SelectObject.Request(
            instance_id=instance_id,
            perception_class=command.get("perception_class", ""),
        )
        future = self._select_object_client.call_async(req)
        future.add_done_callback(lambda fut: self._on_select_object_response(fut, instance_id))

    def _on_select_object_response(self, future, instance_id: str) -> None:
        resp = future.result()
        if not resp.success:
            self._push_error_panel(resp.message or f"could not select '{instance_id}'")
            return
        self._selected_instance_id = instance_id
        self._selected_activations = list(resp.activations)
        self._push_selection_panel(instance_id, resp.activations, resp.message)

    def _handle_button_press(self, command: dict[str, Any]) -> None:
        button_id = command.get("button_id", "")
        mapping = self._panel_buttons.get(button_id)
        if mapping is None:
            # §6: a stale/unknown button_id (panel already changed) must be
            # reported, not silently ignored or forwarded.
            self._push_error_panel(f"'{button_id}' is no longer valid")
            return
        kind = mapping["kind"]
        if kind == "execute_action":
            self._start_execute_action(mapping["engine_id"], mapping["action_id"], mapping.get("params", {}))
        elif kind == "control":
            self._send_control(mapping["command"])
        elif kind == "deselect":
            self._send_deselect()
        elif kind == "dismiss":
            self._rebuild_idle_or_selected_panel()
        else:
            self.get_logger().error(f"unknown panel button kind {kind!r} for {button_id!r}")

    def _send_deselect(self) -> None:
        if not self._deselect_client.wait_for_service(timeout_sec=self._service_wait_timeout):
            self._report_service_unavailable("deselect")
            return
        future = self._deselect_client.call_async(Deselect.Request())
        future.add_done_callback(self._on_deselect_response)

    def _on_deselect_response(self, future) -> None:
        resp = future.result()
        if not resp.success:
            self._push_error_panel(resp.message or "deselect failed")
            return
        self._selected_instance_id = ""
        self._selected_activations = []
        self._push_idle_panel()

    def _send_control(self, command_name: str) -> None:
        if not self._control_client.wait_for_service(timeout_sec=self._service_wait_timeout):
            self._report_service_unavailable("control")
            return
        req = Control.Request(command=_CONTROL_COMMANDS[command_name])
        future = self._control_client.call_async(req)

        def on_response(fut):
            resp = fut.result()
            if not resp.accepted:
                self._push_error_panel(resp.message or f"'{command_name}' not accepted")

        future.add_done_callback(on_response)

    def _start_execute_action(self, engine_id: str, action_id: str, params: dict[str, Any]) -> None:
        if not self._execute_action_client.wait_for_server(
            timeout_sec=self._service_wait_timeout
        ):
            self._report_service_unavailable("execute_action")
            return
        goal = ExecuteAction.Goal(
            engine_id=engine_id,
            action_id=action_id,
            params_json=json.dumps(params),
        )

        def on_feedback(feedback_msg):
            p = feedback_msg.feedback.progress
            panel_json, mapping = build_panel(
                title=f"{engine_id}: {action_id}",
                message=p.message,
                buttons=control_panel_buttons(p.pausable),
                progress={
                    "phase": p.phase,
                    "pct": p.pct,
                    "message": p.message,
                    "pausable": p.pausable,
                },
            )
            self._panel_buttons = mapping
            self.ue.call_function("UpdatePanel", {"PanelJson": panel_json})

        send_future = self._execute_action_client.send_goal_async(
            goal, feedback_callback=on_feedback
        )
        send_future.add_done_callback(
            lambda fut: self._on_execute_action_accepted(fut, engine_id, action_id)
        )

    def _on_execute_action_accepted(self, future, engine_id: str, action_id: str) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._push_error_panel(f"'{engine_id}/{action_id}' goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self._on_execute_action_result(fut, engine_id, action_id)
        )

    def _on_execute_action_result(self, future, engine_id: str, action_id: str) -> None:
        result = _result_to_dict(future.result().result.result)
        status_label = {0: "Success", 1: "Failed", 2: "Cancelled"}.get(result["status"], "Unknown")
        panel_json, mapping = build_panel(
            title=f"{engine_id}: {action_id} -- {status_label}",
            message=result["error_code"] or status_label,
            buttons=[(_DESELECT_BUTTON_ID, "Deselect", "normal", {"kind": "deselect"})],
            style="normal" if result["status"] == 0 else "error",
        )
        self._panel_buttons = mapping
        self.ue.call_function("UpdatePanel", {"PanelJson": panel_json})

    def _report_service_unavailable(self, name: str) -> None:
        self.get_logger().error(f"Orchestrator service '{name}' is not available")
        self._push_error_panel(f"'{name}' service unavailable")

    # ------------------------------------------------------------------
    # InteractionPanel (§5.2.C) push helpers
    # ------------------------------------------------------------------
    def _push_idle_panel(self) -> None:
        panel_json, mapping = build_panel(
            title="", message="Select an object or action to begin."
        )
        self._panel_buttons = mapping
        self.ue.call_function("UpdatePanel", {"PanelJson": panel_json})

    def _push_selection_panel(self, instance_id: str, activations, message: str) -> None:
        panel_json, mapping = build_panel(
            title=instance_id,
            message=message,
            buttons=selection_panel_buttons(activations),
        )
        self._panel_buttons = mapping
        self.ue.call_function("UpdatePanel", {"PanelJson": panel_json})

    def _push_error_panel(self, message: str) -> None:
        panel_json, mapping = build_panel(
            title="Error",
            message=message,
            buttons=[(_DISMISS_BUTTON_ID, "Dismiss", "error", {"kind": "dismiss"})],
            style="error",
        )
        self._panel_buttons = mapping
        self.ue.call_function("UpdatePanel", {"PanelJson": panel_json})

    def _rebuild_idle_or_selected_panel(self) -> None:
        if self._selected_instance_id:
            self._push_selection_panel(self._selected_instance_id, self._selected_activations, "")
        else:
            self._push_idle_panel()

    # ------------------------------------------------------------------
    # §3.3.2: engine-initiated, Orchestrator-bypassing user input requests
    # ------------------------------------------------------------------
    def _on_request_user_input_goal(self, goal_request) -> GoalResponse:
        with self._ui_control_lock:
            granted = bool(self._ui_owner_id) and bytes(goal_request.token) == self._ui_token
        if not granted:
            self.get_logger().warning(
                f"rejecting RequestUserInput from '{goal_request.engine_id}': "
                "token doesn't match the current ui_control grant"
            )
            return GoalResponse.REJECT
        if self._active_input_goal is not None:
            self.get_logger().warning(
                f"rejecting RequestUserInput from '{goal_request.engine_id}': "
                "another engine's goal is already active"
            )
            return GoalResponse.REJECT
        if not goal_request.panel_json and not goal_request.objects:
            self.get_logger().warning(
                f"rejecting RequestUserInput from '{goal_request.engine_id}': "
                "neither panel_json nor objects set"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _push_engine_objects(
        self, engine_id: str, objects, selected_instance_id: str
    ) -> None:
        """Pushes an engine's ``RequestUserInput`` candidate objects to UE's
        ``UpdateDetectedObjects`` channel, tagged with that engine's own
        overlay ``stream_id`` and the currently-highlighted selection (if
        any) -- shared by the goal's initial push and every subsequent
        ``_on_update_user_input_panel`` call."""
        self.ue.call_function(
            "UpdateDetectedObjects",
            {
                "Objects": [
                    {
                        "instance_id": o.instance_id,
                        "perception_class": o.perception_class,
                        "box": list(o.box),
                        "stream_id": engine_id,
                    }
                    for o in objects
                ],
                "SelectedInstanceId": selected_instance_id,
            },
        )

    def _on_request_user_input_execute(self, goal_handle):
        goal = goal_handle.request
        button_ids = panel_button_ids(goal.panel_json)
        object_ids = frozenset(o.instance_id for o in goal.objects)
        done_event = threading.Event()
        with self._active_input_lock:
            self._active_input_goal = {
                "engine_id": goal.engine_id,
                "goal_handle": goal_handle,
                "button_ids": button_ids,
                "object_ids": object_ids,
                "objects": list(goal.objects),
                "selected_instance_id": goal.selected_instance_id,
                "event": done_event,
                "result": {
                    "status": RequestUserInput.Result.STATUS_CANCELLED,
                    "button_id": "",
                },
            }

        if goal.panel_json:
            self.ue.call_function("UpdatePanel", {"PanelJson": goal.panel_json})
        if goal.objects:
            self._push_engine_objects(goal.engine_id, goal.objects, goal.selected_instance_id)

        try:
            while not done_event.is_set():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = RequestUserInput.Result()
                    result.status = RequestUserInput.Result.STATUS_CANCELLED
                    return result
                time.sleep(0.02)

            goal_handle.succeed()
            with self._active_input_lock:
                state = self._active_input_goal["result"]
                result = RequestUserInput.Result()
                result.status = state["status"]
                result.button_id = state["button_id"]
            return result
        finally:
            with self._active_input_lock:
                self._active_input_goal = None
            # Whatever panel/objects this goal pushed on the engine's
            # behalf is now stale -- revert to the normal Orchestrator-
            # driven state (§5.2.C's "Engine-driven override" note).
            self._rebuild_idle_or_selected_panel()

    def _on_update_user_input_panel(
        self, request: UpdateUserInputPanel.Request, response: UpdateUserInputPanel.Response
    ) -> UpdateUserInputPanel.Response:
        """Lets the engine currently holding an active ``RequestUserInput``
        goal change what's displayed (new panel/objects/highlighted
        selection) without completing or restarting the goal -- e.g. after
        a ``EVENT_OBJECT_TAPPED`` feedback, to say "<object> selected" and
        highlight it."""
        with self._ui_control_lock:
            granted = bool(self._ui_owner_id) and bytes(request.token) == self._ui_token
        if not granted:
            response.accepted = False
            response.message = "token doesn't match the current ui_control grant"
            return response

        with self._active_input_lock:
            state = self._active_input_goal
            if state is None:
                response.accepted = False
                response.message = "no active RequestUserInput goal"
                return response
            if request.update_panel:
                state["button_ids"] = panel_button_ids(request.panel_json)
            if request.update_objects:
                state["objects"] = list(request.objects)
                state["object_ids"] = frozenset(o.instance_id for o in request.objects)
            if request.update_selected:
                state["selected_instance_id"] = request.selected_instance_id
            engine_id = state["engine_id"]
            objects = state["objects"]
            selected_instance_id = state["selected_instance_id"]

        if request.update_panel:
            self.ue.call_function("UpdatePanel", {"PanelJson": request.panel_json})
        if request.update_objects or request.update_selected:
            self._push_engine_objects(engine_id, objects, selected_instance_id)

        response.accepted = True
        response.message = ""
        return response

    # ------------------------------------------------------------------
    # Orchestrator -> UE: polled push of world state / detected objects
    # ------------------------------------------------------------------
    def _poll_world_state(self) -> None:
        if not self.ue.is_connected():
            return
        if not self._get_world_state_client.service_is_ready():
            return
        future = self._get_world_state_client.call_async(GetWorldState.Request())
        future.add_done_callback(self._on_world_state_response)

    def _on_world_state_response(self, future) -> None:
        resp = future.result()
        if resp.version == self._last_world_state_version:
            return  # unchanged since the last push -- avoid spamming UE
        self._last_world_state_version = resp.version
        self.ue.call_function(
            "UpdateWorldState",
            {"Version": resp.version, "SnapshotJson": resp.snapshot_json},
        )

    def _poll_detected_objects(self) -> None:
        if not self.ue.is_connected():
            return
        if not self._get_detected_objects_client.service_is_ready():
            return
        future = self._get_detected_objects_client.call_async(GetDetectedObjects.Request())
        future.add_done_callback(self._on_detected_objects_response)

    def _on_detected_objects_response(self, future) -> None:
        resp = future.result()
        ids = frozenset(o.instance_id for o in resp.objects)
        if ids == self._last_detected_ids:
            return  # same set of visible instances as last push
        self._last_detected_ids = ids
        self.ue.call_function(
            "UpdateDetectedObjects",
            {"Objects": [_object_record_to_dict(o) for o in resp.objects]},
        )

    def _publish_connection_status(self) -> None:
        msg = Bool()
        msg.data = self.ue.is_connected()
        self._connection_publisher.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self._stream_shutdown.set()
        self._stream_reconnect_thread.join(timeout=5.0)
        if self._stream_sender.is_connected():
            self._stream_sender.disconnect()
        self.ue.shutdown()
        ue_loop = getattr(self.ue, "loop", None)
        if ue_loop is not None:
            ue_loop.call_soon_threadsafe(ue_loop.stop)
            start = time.time()
            while ue_loop.is_running() and (time.time() - start) < 5.0:
                time.sleep(0.1)
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = UiBridgeNode()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
