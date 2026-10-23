# rammp-UI-bridge

A ROS2 node that bridges an **Unreal Engine (UE)** touchscreen UI to the
[RAMMP Orchestrator](https://github.com/rammp-org/rammp-orchestrator)'s
public ROS2 interface
([`rammp_orchestrator_interfaces`](https://github.com/rammp-org/rammp-interfaces-ros2)).
UE is not a ROS2 participant, so this node is the only thing that talks to
both sides: it translates UE user-input events into Orchestrator
service/action calls, and pushes Orchestrator state (world state, detected
objects, action progress/results) back to UE for rendering.

```
 ┌──────────────┐   UE Remote Control    ┌────────────────┐   ROS2 services/   ┌───────────────┐
 │ Unreal Engine │ ───── WebSocket ─────▶│  rammp_ui_bridge│ ───── actions ────▶│ rammp_orchestr-│
 │ touchscreen UI│◀────── (JSON) ────────│      node       │◀── (topics/srv) ───│ ator           │
 └──────────────┘                        └────────────────┘                    └───────────────┘
```

## Prerequisites

Besides the usual ROS2 (Humble) + `rammp_orchestrator_interfaces`/
`rammp_ui_bridge_interfaces` colcon workspace setup, this node depends on
[`ramms-tools`](https://github.com/rammp-org/ramms-tools) (MIT licensed)
for its RMSS camera-streaming client, which is not on PyPI/rosdep -- install
it manually into the same Python environment colcon builds against:

```bash
pip install git+https://github.com/rammp-org/ramms-tools.git
```

## What this node does

1. **Connects to UE** over its Remote Control WebSocket API (a preset-based
   RPC + property-change-notification protocol UE exposes natively), and
   registers the configured preset on connect/reconnect.
2. **Translates UE user input into Orchestrator calls.** UE only ever
   reports that the user tapped an object or a button (`select_object` /
   `button_press` -- it has no notion of what a button *means*); the bridge
   maps the reported `button_id` back to the right Orchestrator call
   (`deselect`, `control` pause/resume/cancel, or `execute_action`) using a
   mapping it builds whenever it pushes a panel. When an engine is directly
   blocked waiting on user input (via the `~/request_user_input` action,
   below), the bridge checks that pending request first before falling back
   to its own Orchestrator-driven dispatch.
3. **Pushes state back to UE over four channels**: a camera video stream
   (RGBD from Perception and/or RGB from an engine's own camera, each
   optionally carrying detected-object info), system status (arm
   joints/base status), world-state/detected-objects pushes, and a single
   dynamic `InteractionPanel` JSON schema (title/message/buttons/progress)
   that lets UE build whatever popup/dialog the current task calls for
   without hardcoding layouts per action. See `docs/NODE_INTERFACE.md` §5.2
   for the full schema.
4. **Reports bridge health.** Publishes a connection-status topic so other
   ROS2 nodes/operators can see whether a UE instance is currently attached.
5. **Serves engines directly.** Exposes a `~/request_user_input` action
   (`rammp_ui_bridge_interfaces/RequestUserInput`) so an engine that needs
   user input mid-task (an overlay selection, a panel of buttons, or both)
   can ask the bridge directly, without going through the Orchestrator.

This node is intentionally "dumb": it owns no business logic of its own
(no capability/engine knowledge, no world-state rules) -- all of that lives
in the Orchestrator. The bridge only (de)serializes JSON across two
transports, mirroring how `rammp_orchestrator`'s own `ros/orchestrator_node`
is a thin transport layer over its framework-agnostic core.

See [`docs/NODE_INTERFACE.md`](docs/NODE_INTERFACE.md) for the full,
language-agnostic interface specification (ROS2 parameters/services/
actions/topics consumed or published, and the UE↔bridge JSON protocol), and
[`docs/LANGUAGE_DECISION.md`](docs/LANGUAGE_DECISION.md) for the
Python-vs-C++ implementation-language discussion.

## Status

The `rammp_ui_bridge` ament_python package in this repo is the current,
maintained implementation, matching the interface spec below: Orchestrator
world-state/detected-objects polling and push, the unified
`InteractionPanel` scheme (built from a single `UpdatePanel` Remote Control
call), the two-command UE→bridge dispatch (`select_object`/`button_press`
with stale-ID error reporting), the token-gated `~/request_user_input`
action server + `~/request_user_input/update` service for direct
engine↔bridge interaction (§3.3.2 -- an object tap is reported to the
engine as feedback rather than completing the goal, so the engine can
react to repeated/alternating taps via `UpdateUserInputPanel` before the
user finally presses a button), and the camera base-layer stream
(§5.2.A) -- subscribes to the scene camera's raw color/depth topics
(§3.2) and forwards every frame to UE over the RMSS streaming protocol
from the [`ramms-tools`](https://github.com/rammp-org/ramms-tools)
dependency (`ramms_tools.streaming.StreamSender`, see "Prerequisites"
above), as well as the camera *overlay* layer (§3.3.1/§5.2.A) --
subscribes to a single shared topic (`engine_overlay_topic`, default
`/engine_overlay/image_raw`, `rammp_ui_bridge_interfaces/EngineOverlayImage`)
every engine publishes its own camera to (not per-engine, since only one
engine can hold execution resources at a time), and forwards those
frames to UE the same way on a separate channel/`stream_id` -- but only
once the frame's token is verified against the bridge's own
`~/ui_control/{acquire_control,release_control,status}` arbitration
endpoints (reusing `rammp_common_interfaces`' AcquireControl/
ReleaseControl/ControlStatus protocol -- the same grant also gates
`~/request_user_input`, since both represent the one currently-active
engine's exclusive right to talk to the UI), so a stale/crashed publisher
can't leak frames to UE. No real engine calls `acquire_control`/publishes
to that topic or calls `~/request_user_input` yet, so these paths have
only been exercised with synthetic test traffic. The
system-status channel (§5.2.B) now covers arm joints -- subscribes to
`arm_joint_states_topic` (default `/joint_states`,
`sensor_msgs/msg/JointState`, published by the arm driver or its sheppy
mock `arm_mock`) and forwards every joint through to UE **by name**
(not a hardcoded skeleton-bone index array like the Gen1 prototype),
radians converted to degrees, via a `UpdateArmJointStates` Remote
Control call -- but chair/base speed and status, the rest of §5.2.B,
is still not implementable, since no chair/base driver (mock or real)
exists yet in the workspace (see "Not yet in scope" in
`docs/NODE_INTERFACE.md`).

An earlier Python prototype that predates this package (built against an
outdated draft of `docs/NODE_INTERFACE.md`, before the `InteractionPanel`
redesign) lived in a local, uncommitted workspace folder
(`rammp-ui-bridge-local`); its Orchestrator-client wiring and UE WebSocket
transport were ported forward into this package, and it is now superseded.

## Relationship to the Gen1 prototype GUI bridge and ramms-tools

An earlier, hardware-specific GUI bridge (`rammp_prototype_gui.Gui_bridge`)
existed for the Gen1 chair+arm prototype. It streamed camera
color/depth/segmentation-mask frames to UE and mapped real robot joint
states into a UE skeleton, using the same RMSS binary streaming protocol
that [rammp-org/ramms-tools](https://github.com/rammp-org/ramms-tools)
packages as a reusable client SDK (`ramms_tools.streaming`). This bridge
depends on that same SDK (see "Prerequisites" above) for the camera base
layer -- see `docs/NODE_INTERFACE.md` §5.2.A -- now wired up to the scene
camera's real topics (§3.2). The joint/base-state side (§5.2.B) still has
no ROS2-side source -- see `docs/NODE_INTERFACE.md`'s "Not yet in scope"
section for what's blocked on that.


