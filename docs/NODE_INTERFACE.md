# UI Bridge Node Interface

This document specifies the `rammp_ui_bridge` node's interface:
language-agnostic (applies equally to a Python/`rclpy` or C++/`rclcpp`
implementation), since it's a direct consequence of
`rammp_orchestrator_interfaces` plus the UE Remote Control protocol, not of
any implementation choice. See `docs/LANGUAGE_DECISION.md` for the
implementation-language question itself.

## 1. Node identity

- Node name: `ui_bridge_node`
- Talks to exactly one Orchestrator instance, addressed by a configurable
  namespace (default `/rammp_orchestrator`, matching
  `rammp_orchestrator.ros.orchestrator_node`'s default node name), and
  exactly one UE instance, addressed by host/port/preset parameters.

## 2. Parameters

| Name | Type | Default | Meaning |
|---|---|---|---|
| `ue_host` | string | `127.0.0.1` | IP/hostname of the UE instance |
| `ue_ws_port` | int | `30020` | UE Remote Control WebSocket API port |
| `ue_preset` | string | `RAMMP` | UE Remote Control preset name to register against |
| `orchestrator_ns` | string | `/rammp_orchestrator` | Namespace the Orchestrator's services/action are advertised under |
| `world_state_poll_period` | double (seconds) | `0.5` | Interval between `~/get_world_state` polls |
| `detected_objects_poll_period` | double (seconds) | `0.5` | Interval between `~/get_detected_objects` polls |
| `connection_publish_period` | double (seconds) | `1.0` | Interval between connection-status publishes |
| `service_wait_timeout` | double (seconds) | `5.0` | How long to wait for an Orchestrator service/action server to become available before reporting it unavailable |

## 3. Orchestrator-facing interface (ROS2 client side)

All of the following are defined in `rammp_orchestrator_interfaces`
(package `rammp-interfaces-ros2`); this node is a **client only** -- it
never advertises any of these itself.

| Orchestrator interface | Type | Used for |
|---|---|---|
| `<orchestrator_ns>/get_world_state` | `GetWorldState.srv` | Polled; result pushed to UE on version change |
| `<orchestrator_ns>/get_detected_objects` | `GetDetectedObjects.srv` | Polled; result pushed to UE on instance-set change (see note below re: bounding boxes) |
| `<orchestrator_ns>/select_object` | `SelectObject.srv` | Called when UE sends a `select_object` command (user tapped an object) |
| `<orchestrator_ns>/deselect` | `Deselect.srv` | Called when the bridge resolves a `button_press` whose panel button represented "deselect"/"back" (§5.1) |
| `<orchestrator_ns>/control` | `Control.srv` | Called when the bridge resolves a `button_press` whose panel button represented a pause/resume/cancel control (§5.1) |
| `<orchestrator_ns>/execute_action` | `ExecuteAction.action` | Called when the bridge resolves a `button_press` whose panel button represented a specific action (§5.1); feedback and result both relayed to UE |

The bridge never calls `register_engine`, `write_world_state`,
`update_object_state`, or `request_intent` -- those are engine-facing, not
UI-facing (see `rammp_orchestrator_interfaces` package docs).

**Note: UE itself never sends `deselect`/`control`/`execute_action`
commands.** See §5.1 -- the bridge turns each available control/action
option into a button on the current `InteractionPanel`, and UE only ever
reports back which button was tapped (`button_press` + `button_id`). This
table's "Used for" column describes what the bridge calls once it resolves
that `button_id`, not a direct UE→Orchestrator pass-through.

**Note on tap-to-select overlays / bounding boxes.** `ObjectRecord` today
carries `instance_id`/`perception_class`/`last_pose_json` (a world/robot-
frame pose) /`location`/`opened`/`last_seen` -- no camera-pixel-space
bounding box. For UE to let a user tap an object directly in the live
camera-stream video (§5.2.A) and have that resolve to a `select_object`
call, each detected object needs a 2D bbox in that camera's pixel space,
tagged with which `stream_id` it overlays.

The recommended design (not yet implemented, needs `ObjectRecord`/
perception changes upstream) is to **extend the existing detected-objects
push** with an optional `bbox: {x, y, w, h}` + `stream_id` per object,
reusing the same `instance_id` that `select_object` already consumes --
rather than routing object/bbox data through `InteractionPanel` (§5.2.C).
`InteractionPanel` is a one-shot task dialog (title/message/buttons/
progress) with a different update cadence and payload shape than per-
frame, per-camera-stream object detections; keeping detected-object data on
its existing dedicated channel keeps the "tap an object" and "tap a task
button" interactions on two cleanly separate paths while still sharing one
`instance_id`/`button_id`-style identifier scheme. This needs upstream
`ObjectRecord`/perception support before it can be implemented -- not
blocking this doc, but tracked here as the resolved direction pending
perception-side bbox availability.

## 4. UI-facing interface (ROS2 server/publisher side)

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `~/ue_connection` | `std_msgs/Bool` | published | Whether the UE WebSocket is currently connected, at `connection_publish_period` |

No services or actions are currently advertised by this node to other ROS2
clients; all UI interaction happens over the UE WebSocket, not ROS2.

## 5. UE-facing interface (Remote Control WebSocket)

Transport: UE's Remote Control WebSocket API
(`ws://<ue_host>:<ue_ws_port>`), with the bridge registering `ue_preset` on
every connect (including reconnects after a dropped connection).

### 5.1 UE → bridge: `UserInput` command schema

The bridge watches the preset's `UserInput` string property for changes
(delivered as a `ChangedFields` WebSocket event) and parses each new value
as one JSON object.

**Governing principle: UE only ever reports a tap -- it never sends a
domain command directly.** UE has exactly two things a user can tap: an
object (in the camera view or a world-state list) or a button (on the
current `InteractionPanel`, §5.2.C). There is no UE-facing `deselect`,
`control`, or `execute_action` command -- those are Orchestrator-facing
concepts the bridge itself decides to expose, by rendering each available
action/control option as a panel button. UE just echoes back whichever
opaque ID (`instance_id` or `button_id`) the bridge most recently assigned
it; the bridge is the only side that knows what that ID actually means and
resolves it internally.

| `cmd` | Fields | Bridge behavior |
|---|---|---|
| `select_object` | `instance_id` (string) -- echoed from the last `UpdateDetectedObjects` push (§5.2.D) | Calls Orchestrator `select_object`, then rebuilds and pushes an updated `InteractionPanel` reflecting the newly available actions/controls for that object (as buttons) |
| `button_press` | `button_id` (string) -- echoed from the last `InteractionPanel` push (§5.2.C) | The bridge looks up what `button_id` meant when it built that panel, and performs the corresponding Orchestrator call: `execute_action` (a specific action button), `control` (a pause/resume/cancel button), `deselect` (a "deselect"/"back" button), or no Orchestrator call at all (e.g. a purely client-side "dismiss this panel" button) |

The bridge owns the `button_id → (Orchestrator call + params)` mapping for
whatever panel it most recently pushed; this mapping is internal bridge
state, never sent to or interpreted by UE. A stale/unknown `button_id`
(e.g. from a panel that's since been replaced) must be treated like any
other unresolvable command (see error handling below), not forwarded
blindly.

Unknown `cmd` values, and values that don't parse as JSON at all, are
logged and must **not** be forwarded to the Orchestrator; the bridge
reports them back to UE via an error report (see §5.2.C) rather than
silently dropping them, so a UI/Blueprint bug is visible during
development.

### 5.2 bridge → UE: four channels

Bridge → UE traffic is organized into four channels with different
transports and update rates, rather than one undifferentiated stream of
Remote Control function calls:

#### A. Camera stream (video data plane)

Reuses the Gen1 prototype's TCP streaming mechanism unchanged
(`streaming/sender.py` + `protocol.py` + `client.py` -- per-channel
`send_image`/`send_depth`/`send_mask` framing, not the Remote Control
WebSocket, since this is bulk pixel data). Two kinds of sources:

- **RGBD from a Perception node** -- color + depth, optionally accompanied
  by detected-object information for that frame when Perception has
  detections; absent otherwise.
- **RGB-only from an Engine node's own camera** (e.g. a wrist camera active
  only during a manipulation action), likewise optionally accompanied by
  detections.

Each source gets its own channel/`stream_id`, tagged with
`source`/`role`/`group` metadata so UE can route each frame to the right
material/viewport -- same conventions as Gen1's per-camera channel
numbering (see that bridge's `image_channel`/`depth_channel`/`mask_channel`
parameters for precedent).

**Important correction re: "detected objects" transport.** The Gen1
streaming protocol itself (`StreamHeader`/`StreamMessage`) carries **no
semantic labelling** -- its metadata is just `w`/`h`/`fmt`/`group`/`role`/
`stream_id`/`name`/`material_params`, and `send_mask()` sends only a
per-pixel segmentation-ID image plus a `NumSegmentIDs` count (a shader
hint, not a label list). In Gen1, the actual object semantics (e.g.
"segment ID 1 = cup, state = full") were pushed **separately** via Remote
Control calls (`UpdateCupInfo`/`UpdateButtonInfo`), correlated with the
mask stream only by convention, not carried in-band. This bridge should do
the same: a segmentation mask (if sent) travels on the camera-stream
channel as pixel data only, while any accompanying detected-object
semantics (class, instance ID, 2D bounding box, pose) are pushed as data on
the **existing detected-objects push** (§3's note on tap-to-select
overlays/bounding boxes), *not* on `InteractionPanel` (§5.2.C), which stays
reserved for the task/button dialog only. **Open/TBD:** neither perception
nor engine camera topics are defined in `rammp_orchestrator_interfaces` /
`rammp_perception_interfaces` yet -- this channel's exact ROS2-side source
topics (and whether a segmentation mask is even produced) need to be
defined before implementation.

#### B. System status (robot proprioception)

Arm joints, base status, and similar small/structured/high-frequency
telemetry -- pushed via Remote Control preset function calls (like Gen1's
`setJoints`/`UpdateSystemState`), **not** the streaming channel, since this
is small structured data rather than bulk pixel payloads. **Open/TBD:**
no engine or Orchestrator interface currently publishes joint states or
base status -- the source topic(s) need to be defined (likely a new
engine-published topic) before this channel can be implemented.

#### C. `InteractionPanel` -- task/capability prompt (renamed from "information")

A single JSON schema describing whatever dialog/popup UE should render for
the current task/capability context, letting UE build it dynamically
instead of hardcoding dialog layouts per action. Content (**as sent to
UE**):

- `title` (string)
- `message` (string) -- free-text status/instruction to show the user
- `buttons`: array of `{button_id, label, style}`, where `style` is one of
  `normal` / `warning` / `error`, and `button_id` is an opaque identifier
  the bridge assigns and later resolves when UE reports a button press
  (see §5.1)
- `progress` (optional): `{phase, pct, message, pausable}` -- the same
  shape as `ExecuteAction` feedback, folded into the panel instead of a
  separate function call

Per §5.1, every button the bridge puts in `buttons` corresponds to exactly
one of: an available action (from `ActionAvailability`, resolved on tap as
an `execute_action` call with its `engine_id`/`action_id`/pre-filled
`params`), a control option (pause/resume/cancel, resolved as a `control`
call), a deselect/back option (resolved as a `deselect` call), or a purely
client-side dismiss with no Orchestrator call. **The bridge-internal
mapping that records which of these each `button_id` represents (plus
whatever call parameters it needs) is never part of the JSON sent to
UE** -- it's bridge-side state keyed by `button_id`, rebuilt every time a
new panel is pushed.

This is deliberately a superset that can express what §5.2 previously
modeled as four separate functions (`UpdateSelection` i.e. available
actions as buttons, `UpdateProgress`, `UpdateActionResult`, and
`UpdateBridgeError` i.e. an error-styled panel) -- those are superseded by
pushing an appropriately-populated `InteractionPanel` instead. A single
unified, versionable JSON schema is easier for UE to parse generically
(build N buttons from an array) and to extend later (new optional fields)
than a growing set of bespoke RPC function signatures.

**Open/TBD:**
- Transport: either a Remote Control function call (e.g.
  `UpdatePanel({"PanelJson": "..."})`) or the streaming channel --
  undecided; likely driven by size/frequency once real payloads are
  measured (Remote Control calls are simpler for small/sporadic panels;
  the streaming channel may be preferable if panels become large/frequent).
- Exact JSON field names/casing and whether `world_state`/
  `detected_objects` pushes (`UpdateWorldState`/`UpdateDetectedObjects`)
  also fold into this scheme or remain separate -- for now, treat those two
  as still-separate raw-data pushes, distinct from the dialog/prompt
  `InteractionPanel` describes.
- The name itself -- `InteractionPanel` was chosen over `ActionPanel`/
  `UiPanel`/the placeholder `information`; open to revision.

#### D. World state / detected objects (Remote Control preset functions)

Separate from `InteractionPanel`, the bridge pushes two raw-data feeds over
the same Remote Control preset-function mechanism as §5.2.B (request/
response RPC over the WebSocket, `PUT /remote/preset/<preset>/function/
<FunctionName>`) -- not the streaming channel, since both are small,
infrequent JSON payloads rather than bulk pixel data:

| Function | Parameters | Pushed when |
|---|---|---|
| `UpdateWorldState` | `Version` (int), `SnapshotJson` (string, JSON-encoded World State) | `~/get_world_state` poll result's `version` differs from the last pushed value |
| `UpdateDetectedObjects` | `Objects`: array of `{instance_id, perception_class, last_pose, location, opened, last_seen}` (plus `bbox`/`stream_id` per object once §3's bbox extension lands) | `~/get_detected_objects` poll result's set of `instance_id`s differs from the last pushed set |

Mechanics:

1. The bridge polls `~/get_world_state` every `world_state_poll_period`
   and `~/get_detected_objects` every `detected_objects_poll_period`
   (§2) -- polling, not a subscription, since the Orchestrator exposes
   these as services, not topics.
2. Each poll response is compared against what was last actually pushed
   to UE (World State `version` number; the `frozenset` of detected
   `instance_id`s) -- **not** against the previous poll response, so a
   dropped/late poll can't cause a spurious re-push.
3. Only on a real change does the bridge call the corresponding UE preset
   function, once, with the full new snapshot/object list (not a diff) --
   UE always receives a complete, self-consistent payload rather than
   incremental patches to reconcile.
4. Per-object JSON shape (`_object_record_to_dict` in the reference
   implementation): `instance_id`/`perception_class` as-is;
   `last_pose_json` is JSON-decoded into a nested `last_pose` object
   (`{}` if empty); `location`/`opened`/`last_seen` passed through as-is.

This is the channel §3's bounding-box extension (tap-to-select camera
overlays) and any future per-object metadata should be added to --
`InteractionPanel` (§5.2.C) is deliberately not used for this data, since
it's keyed by `instance_id` on its own change-triggered cadence, not tied
to the task/button-dialog lifecycle.

### 5.3 Push semantics

- World-state/detected-objects pushes remain **change-triggered, not
  every-poll** -- the bridge must track the last-pushed World State
  `version` and the last-pushed set of detected `instance_id`s, and only
  push when either actually changed. This bounds UE-side traffic to actual
  state changes regardless of poll frequency.
- `InteractionPanel` pushes are **event-triggered** -- one push whenever
  the available actions, progress, or an error condition changes (e.g. on
  `select_object` response, on each `ExecuteAction` feedback message, on
  terminal result, or on a reportable bridge error), no polling involved.
- Camera stream and system-status pushes are inherently continuous/
  event-driven at whatever rate their ROS2 sources publish, once those
  sources are defined (see §5.2.A/B "Open/TBD").

## 6. Error handling requirements

- If an Orchestrator service/action server is not available within
  `service_wait_timeout`, the bridge must not block indefinitely waiting
  for it -- report it to UE via an error-styled `InteractionPanel` (§5.2.C)
  and return.
- A single malformed/invalid UE command must not crash the node or stop
  subsequent commands from being processed (isolate failures per-command).
- The UE WebSocket connection must reconnect automatically on drop
  (including re-registering the preset) without requiring the node to
  restart, and `~/ue_connection` must reflect disconnection promptly.

## 7. Not yet in scope

- **Exact ROS2 source topics for §5.2.A (camera streams) and §5.2.B
  (system status)** are undefined -- `rammp_orchestrator_interfaces`/
  `rammp_perception_interfaces` do not currently publish camera frames or
  joint/base state. Camera streaming and system status are *intended*
  channels of this bridge (unlike in the prior version of this doc), but
  implementation is blocked on those source topics being defined first.
- **2D bounding boxes for tap-to-select camera overlays** (§3's note) are
  undefined on the perception side -- `ObjectRecord` has no camera-pixel-
  space bbox field today. The direction (extend the detected-objects push,
  not `InteractionPanel`) is decided; the upstream `ObjectRecord`/
  perception change to actually produce it is not.
- Joint-state → UE skeleton **mapping logic** (the specific joint-index
  layout the Gen1 prototype's `set_ui_joints` hardcoded for its chair+arm
  robot) does not carry over as-is, since it was specific to that
  hardware's joint layout -- §5.2.B needs its own mapping once a real
  engine's joint-state shape is known.

