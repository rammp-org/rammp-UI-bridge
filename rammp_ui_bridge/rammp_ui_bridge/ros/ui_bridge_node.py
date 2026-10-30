"""Thin rclpy bridge between an Unreal Engine touchscreen UI and the RAMMP
Orchestrator, per ``docs/NODE_INTERFACE.md``. Talks to the Orchestrator
(and to engines) purely via ``rammp_ui_bridge_interfaces``'s generic
``RequestUserInput``/``UpdateUserInputPanel`` mechanism -- it no longer
depends on ``rammp_orchestrator_interfaces`` at all.

**Governing principle (§5.1): UE only ever reports a tap.** UE sends exactly
two command shapes -- ``select_object`` (tapped an object) and
``button_press`` (tapped a button on the current ``InteractionPanel``).
Every caller -- the Orchestrator's own always-open browse session and any
engine running an action -- is a ``~/request_user_input``/
``~/request_user_input/update`` caller; this bridge never originates
Orchestrator calls (``select_object``/``deselect``/``control``/
``execute_action``) on its own anymore. A tap is resolved purely against
whichever ``RequestUserInput`` goal is currently active (see
``_try_resolve_active_input_goal``); there is no other dispatch path.

**Engine-driven user input (§3.3.2).** This node serves
``~/request_user_input`` (``rammp_ui_bridge_interfaces/RequestUserInput``),
called by whichever caller (Orchestrator or an active engine) currently
holds the ``ui_control`` arbitration grant (a goal's ``token`` must match).
Tapping an object never completes the goal by itself -- it's reported to
the caller as feedback, and the caller may call
``~/request_user_input/update``
(``rammp_ui_bridge_interfaces/UpdateUserInputPanel``) any number of times
while its goal is active to change the displayed panel/candidate
list/highlighted selection in response. Only a button press (or the
caller's own cancel) ends the goal.

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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool

from rammp_common_interfaces.msg import ControlStatus
from rammp_common_interfaces.srv import AcquireControl, ReleaseControl
from rammp_ui_bridge_interfaces.action import RequestUserInput
from rammp_ui_bridge_interfaces.msg import EngineOverlayImage
from rammp_ui_bridge_interfaces.srv import UpdateUserInputPanel

from ramms_tools.streaming import StreamSender
from ..unreal_remote_websocket import UnrealRemoteWebsocket

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

def panel_button_ids(panel_json: str) -> frozenset[str]:
    """Extracts the set of ``button_id``s a ``RequestUserInput`` caller's
    (Orchestrator or engine, §5.1/§3.3.2) ``InteractionPanel`` JSON payload
    offers, for matching against the active goal in
    ``_try_resolve_active_input_goal``."""
    if not panel_json:
        return frozenset()
    try:
        payload = json.loads(panel_json)
    except (TypeError, ValueError):
        return frozenset()
    return frozenset(b.get("button_id", "") for b in payload.get("buttons", []) if b.get("button_id"))


def parse_request_json(request_json: str) -> dict[str, Any]:
    """Parses a ``RequestUserInput`` goal's or ``UpdateUserInputPanel``
    request's ``request_json`` (§3.3.2) into a plain dict -- keys present:
    ``panel_json`` (str), ``objects`` (list of ``{instance_id,
    perception_class, box}`` dicts), ``selected_instance_id`` (str), each
    only present if the caller included it (``UpdateUserInputPanel``
    distinguishes "update this aspect" from "leave it alone" purely by key
    presence, see rammp-interfaces-ros2's UpdateUserInputPanel.srv).
    Malformed/non-object JSON parses to ``{}`` rather than raising, same
    fail-soft convention as ``panel_button_ids``."""
    if not request_json:
        return {}
    try:
        payload = json.loads(request_json)
    except (TypeError, ValueError):
        return {}
    return payload if isinstance(payload, dict) else {}


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
        self.declare_parameter("connection_publish_period", 1.0)
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
        # When False, this node stops forwarding frames to UE itself (the
        # three _forward_* methods below become no-ops) -- set False when a
        # separate rammp_ui_bridge_streaming (C++) node is running instead,
        # to avoid sending every frame to UE twice. Everything else (topic
        # subscriptions, frame-count logging, ui_control arbitration/token
        # checking) keeps running unchanged either way, since the C++ node
        # relies on this node's validated_engine_overlay_topic republish
        # (see _on_engine_overlay_frame) for the overlay layer rather than
        # re-implementing arbitration itself. Default True: zero behaviour
        # change out of the box.
        self.declare_parameter("use_python_camera_streaming", True)
        # Plain (token-stripped) republish of engine-overlay frames that
        # already passed this node's ui_control arbitration check -- the
        # C++ streaming node (if running) subscribes to this instead of the
        # raw EngineOverlayImage topic, so it never needs to know about
        # arbitration tokens at all (see _on_engine_overlay_frame).
        self.declare_parameter("validated_engine_overlay_topic", "~/validated_engine_overlay")
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

        self._cb_group = ReentrantCallbackGroup()

        # Set while a RequestUserInput goal from an engine is active; see
        # _on_request_user_input_execute. None when no engine is waiting on
        # user input right now. Guarded by _active_input_lock since it's
        # mutated from the UE-callback thread (object taps/button presses,
        # see _try_resolve_active_input_goal), the ~/request_user_input/update
        # service thread (_on_update_user_input_panel), and read from the
        # action's own execute-callback thread.
        self._active_input_lock = threading.Lock()
        self._active_input_goal: dict[str, Any] | None = None

        # See use_python_camera_streaming above -- when False, a separate
        # rammp_ui_bridge_streaming (C++) node is forwarding frames to UE
        # instead, so this node's own _forward_* methods become no-ops.
        self._use_python_streaming = bool(
            self.get_parameter("use_python_camera_streaming").value
        )

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
        # Plain (token-stripped) republish of overlay frames that already
        # passed ui_control arbitration -- see validated_engine_overlay_topic
        # above and _on_engine_overlay_frame below. Published unconditionally
        # (cheap, and useful for anyone else wanting a validated feed), not
        # gated by use_python_camera_streaming.
        self._validated_overlay_publisher = self.create_publisher(
            Image,
            self.get_parameter("validated_engine_overlay_topic").value,
            qos_profile_sensor_data,
        )
        self._arm_joint_states_sub = self.create_subscription(
            JointState,
            self.get_parameter("arm_joint_states_topic").value,
            self._on_arm_joint_states,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"rammp_ui_bridge ready; bridging UE at "
            f"{self.get_parameter('ue_host').value}; "
            f"scene camera color='{self.get_parameter('scene_camera_color_topic').value}' "
            f"depth='{self.get_parameter('scene_camera_depth_topic').value}' "
            f"engine overlay='{self.get_parameter('engine_overlay_topic').value}' "
            f"arm joint states='{self.get_parameter('arm_joint_states_topic').value}' -> "
            f"UE stream port {self.get_parameter('ue_stream_port').value} "
            f"(python camera streaming "
            f"{'enabled' if self._use_python_streaming else 'disabled -- external streamer expected'})"
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
        if not self._use_python_streaming or not self._stream_sender.is_connected():
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
        if not self._use_python_streaming or not self._stream_sender.is_connected():
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
                minted_fresh = False
            else:
                self._ui_token = os.urandom(16)
                minted_fresh = True
            self._ui_owner_id = request.owner_id
            self._ui_generation += 1
            response.accepted = True
            response.token = list(self._ui_token)
            response.generation = self._ui_generation
            response.message = f"granted to '{request.owner_id}'"
        self.get_logger().info(
            f"ui control acquired by '{request.owner_id}' "
            f"({'minted a new' if minted_fresh else 'using orchestrator-supplied'} "
            f"token {self._ui_token.hex()[:8]}..., generation {self._ui_generation})"
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
        # Republish token-stripped, post-arbitration, for the optional C++
        # streaming node (or any other consumer) to forward without needing
        # to know about ui_control arbitration tokens at all.
        self._validated_overlay_publisher.publish(image)
        self._forward_engine_overlay_color(image)

    def _forward_engine_overlay_color(self, msg: Image) -> None:
        if not self._use_python_streaming or not self._stream_sender.is_connected():
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
    # UE -> active RequestUserInput caller (Orchestrator or engine):
    # dispatch user-input commands
    # ------------------------------------------------------------------
    def _on_ue_user_input(self, raw: str) -> None:
        """Invoked (on the UE WebSocket's asyncio thread) whenever UE's
        "UserInput" preset property changes -- see §5.1 for the expected
        JSON command schema. Only ``select_object``/``button_press`` are
        valid UE-originated commands. Every tap is resolved purely against
        whichever ``RequestUserInput`` goal is currently active (the
        Orchestrator's own always-open browse session, or an engine's,
        while it runs an action) -- there is no other dispatch path."""
        try:
            command = json.loads(raw)
        except (TypeError, ValueError):
            self.get_logger().error(f"ignoring malformed UserInput (not JSON): {raw!r}")
            return

        cmd = command.get("cmd")
        if cmd not in ("select_object", "button_press"):
            self.get_logger().error(f"ignoring UserInput with unknown cmd: {cmd!r}")
            return
        if self._try_resolve_active_input_goal(cmd, command):
            return
        self.get_logger().warning(
            f"dropped UserInput {command!r}: no active RequestUserInput goal "
            "matched it (nobody is currently offering this tap)"
        )

    def _try_resolve_active_input_goal(self, cmd: str, command: dict[str, Any]) -> bool:
        """Returns True if the tap matched the active ``RequestUserInput``
        goal and was consumed by it (the normal dispatch table must not
        also run for it). A matched ``select_object`` does NOT end the
        goal -- the user may tap several different objects in a row while
        deciding -- it's reported to the engine as feedback instead
        (``feedback_json``'s ``event_type: "object_tapped"``), and
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
                goal_state["result"]["status"] = "button"
                goal_state["result"]["button_id"] = button_id
                goal_state["event"].set()
                return True
            else:
                return False
        feedback = RequestUserInput.Feedback()
        feedback.feedback_json = json.dumps({"event_type": "object_tapped", "instance_id": instance_id})
        goal_handle.publish_feedback(feedback)
        return True

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
        request = parse_request_json(goal_request.request_json)
        if not request.get("panel_json") and not request.get("objects"):
            self.get_logger().warning(
                f"rejecting RequestUserInput from '{goal_request.engine_id}': "
                "neither panel_json nor objects set in request_json"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _push_engine_objects(
        self, objects: list[dict[str, Any]], selected_instance_id: str
    ) -> None:
        """Pushes an engine's ``RequestUserInput`` candidate objects to UE's
        ``UpdateDetectedObjects`` channel, along with the currently-
        highlighted selection (if any) -- shared by the goal's initial push
        and every subsequent ``_on_update_user_input_panel`` call.
        ``objects`` is already a list of plain dicts (parsed from
        ``request_json``), not ROS messages. Each object's own ``source``
        key (``"scene"``/``"overlay"``, caller-supplied) is passed through
        as-is so UE knows which video channel the box's pixel coordinates
        are relative to -- the bridge does not infer or default this from
        ``engine_id``, since one goal's objects can mix both sources."""
        self.ue.call_function(
            "UpdateDetectedObjects",
            {
                "Objects": [
                    {
                        "instance_id": o.get("instance_id", ""),
                        "perception_class": o.get("perception_class", ""),
                        "box": list(o.get("box", (0.0, 0.0, 0.0, 0.0))),
                        "source": o.get("source", ""),
                    }
                    for o in objects
                ],
                "SelectedInstanceId": selected_instance_id,
            },
        )

    def _on_request_user_input_execute(self, goal_handle):
        goal = goal_handle.request
        request = parse_request_json(goal.request_json)
        panel_json = request.get("panel_json") or ""
        objects = request.get("objects") or []
        selected_instance_id = request.get("selected_instance_id") or ""
        button_ids = panel_button_ids(panel_json)
        object_ids = frozenset(o.get("instance_id", "") for o in objects)
        done_event = threading.Event()
        with self._active_input_lock:
            self._active_input_goal = {
                "engine_id": goal.engine_id,
                "goal_handle": goal_handle,
                "button_ids": button_ids,
                "object_ids": object_ids,
                "objects": list(objects),
                "selected_instance_id": selected_instance_id,
                "event": done_event,
                "result": {
                    "status": "cancelled",
                    "button_id": "",
                },
            }

        if panel_json:
            self.ue.call_function("UpdatePanel", {"PanelJson": panel_json})
        if objects:
            self._push_engine_objects(objects, selected_instance_id)

        try:
            while not done_event.is_set():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = RequestUserInput.Result()
                    result.result_json = json.dumps({"status": "cancelled"})
                    return result
                time.sleep(0.02)

            goal_handle.succeed()
            with self._active_input_lock:
                state = self._active_input_goal["result"]
                result = RequestUserInput.Result()
                if state["status"] == "button":
                    result.result_json = json.dumps({"status": "button", "button_id": state["button_id"]})
                else:
                    result.result_json = json.dumps({"status": "cancelled"})
            return result
        finally:
            with self._active_input_lock:
                self._active_input_goal = None
            # Whatever panel/objects this goal pushed is now stale, but
            # it's not this node's job to decide what replaces it -- the
            # next caller (the Orchestrator's own browse session reopening
            # after a handoff, or nothing at all) pushes its own panel via
            # a fresh RequestUserInput goal.

    def _on_update_user_input_panel(
        self, request: UpdateUserInputPanel.Request, response: UpdateUserInputPanel.Response
    ) -> UpdateUserInputPanel.Response:
        """Lets the engine currently holding an active ``RequestUserInput``
        goal change what's displayed (new panel/objects/highlighted
        selection) without completing or restarting the goal -- e.g. after
        a ``"object_tapped"`` feedback, to say "<object> selected" and
        highlight it."""
        with self._ui_control_lock:
            granted = bool(self._ui_owner_id) and bytes(request.token) == self._ui_token
        if not granted:
            response.accepted = False
            response.message = "token doesn't match the current ui_control grant"
            return response

        update = parse_request_json(request.request_json)
        update_panel = "panel_json" in update
        update_objects = "objects" in update
        update_selected = "selected_instance_id" in update

        with self._active_input_lock:
            state = self._active_input_goal
            if state is None:
                response.accepted = False
                response.message = "no active RequestUserInput goal"
                return response
            if update_panel:
                state["button_ids"] = panel_button_ids(update["panel_json"])
            if update_objects:
                objects = update["objects"] or []
                state["objects"] = list(objects)
                state["object_ids"] = frozenset(o.get("instance_id", "") for o in objects)
            if update_selected:
                state["selected_instance_id"] = update["selected_instance_id"]
            objects = state["objects"]
            selected_instance_id = state["selected_instance_id"]

        if update_panel:
            self.ue.call_function("UpdatePanel", {"PanelJson": update["panel_json"]})
        if update_objects or update_selected:
            self._push_engine_objects(objects, selected_instance_id)

        response.accepted = True
        response.message = ""
        return response

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
