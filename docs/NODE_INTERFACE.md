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

## 3. ROS2-facing interfaces (client side)

This node is a ROS2 **client only** on four separate peers -- it never
advertises any of the interfaces in this section itself (its own
server/publisher side is §4):

- **§3.1 Bridge ↔ Orchestrator** -- the control/state surface: object
  selection, actions, pause/resume/cancel, world state, and detected
  objects (including pixel-space bounding boxes -- see the note in
  §3.1.1's `get_detected_objects` entry for why this is sourced from the
  Orchestrator rather than a direct Perception subscription).
- **§3.2 Bridge ↔ Camera** -- a *direct* subscription to the scene
  camera's raw RGB/depth topics, for the pixel data actually streamed to
  UE (§5.2.A). This is a camera driver node (e.g. a RealSense node), not
  Perception -- Perception is itself just a consumer of this same feed
  (`Detect.srv`/`StartTracking.srv`'s `image_topic` parameter), not its
  source.
- **§3.3 Bridge ↔ Engine node** -- a *direct* subscription to the
  currently-active engine's own camera stream (e.g. a wrist camera, with
  any interaction mask/overlay already rendered into the RGB pixels by
  the engine itself) -- nothing else. The engine does **not** push
  detected objects to the bridge directly; it pushes them to the
  Orchestrator (via `update_object_state`), which is what the bridge
  reads from §3.1 -- see the note at the end of this section.
- **§3.4 Bridge ↔ Hardware node** -- a *direct* subscription to
  always-on, hardware-level robot telemetry (arm joint states, chair/base
  speed and status) that exists independent of which engine (if any) is
  currently active, unlike §3.3's per-engine camera.

There is deliberately **no direct Bridge ↔ Perception interface**. See
the design-decision note after `get_detected_objects` below for why.

### 3.1 Bridge ↔ Orchestrator

All of the following are defined in `rammp_orchestrator_interfaces`
(package `rammp-interfaces-ros2`), advertised by `orchestrator_node`
under `orchestrator_ns` (§2).

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

#### 3.1.1 Field-level detail per call

#### `select_object` (`SelectObject.srv`)

```
# request
string instance_id          # from the tapped ObjectRecord (§5.2.D)
string perception_class     # from the same ObjectRecord
---
# response
bool success
rammp_orchestrator_interfaces/EngineActivation[] activations
string message
```

Each `EngineActivation` is `{engine_id, ActionAvailability[] availability}`
-- one entry per engine that claims `perception_class` (more than one can,
e.g. "microwave" claimed by Articulation, Pick-and-Place, and Microwave
Controls). Each `ActionAvailability` is
`{action_id, available, reason, suggested_params_json}`.

The bridge flattens every `available=true` entry across all `activations`
into the button list for the next `InteractionPanel` push (§5.2.C),
discarding (or optionally surfacing as disabled buttons with `reason` as
the tooltip/message) the `available=false` ones. `engine_id`+`action_id`
together become the bridge-internal key the resulting `button_id` maps
back to (so two engines offering the same `action_id`, e.g. both
Articulation and Pick-and-Place offering `"open"` on a fridge, resolve to
two distinct buttons, not one).

**Gap: no human-readable label in the response.** `ActionAvailability`
carries only the machine `action_id`, not a display string -- the
human-readable text lives on `ActionSpec.label` (in each engine's
`CapabilityManifest`, registered at `register_engine` time), which the
Orchestrator already holds internally but does not currently expose to UI
clients. Until this is resolved upstream, the bridge cannot build a
correctly-labelled button purely from `select_object`'s response. Tracked
as an open item in §7; the two candidate fixes are (a) add
`label`/`style_hint` fields to `ActionAvailability` itself, populated from
the matching `ActionSpec` at the point the Orchestrator already builds
`ActionAvailability` from each `ActionSpec`, or (b) add a dedicated
UI-facing query service that returns the registered `CapabilityManifest`s
(and thus every `ActionSpec.label`) so the bridge can cache
`action_id -> label` per engine. (a) is the smaller change and avoids a
second round trip, so it's the recommended direction, but needs
Orchestrator-side confirmation before being implemented.

#### `deselect` (`Deselect.srv`)

```
# request: (empty)
---
# response
bool success
string message
```
Called with no request fields. On `success`, the Orchestrator has
hibernated every activated engine and returned to Browse mode; the bridge
responds by clearing the current `InteractionPanel` (or replacing it with
whatever default/idle panel state is defined) and forgetting its
`button_id -> (call, params)` mapping, since none of those buttons are
valid anymore.

#### `control` (`Control.srv`)

```
# request
uint8 command   # PAUSE=0, RESUME=1, CANCEL=2
---
# response
bool accepted
string message
```
Forwarded to whichever engine currently holds execution resources (the
delegate while in `Delegated` mode, otherwise the primary engine -- this
routing is internal to the Orchestrator, the bridge does not need to know
which engine is actually targeted). On `accepted=false`, the bridge should
surface `message` as an error/toast via `InteractionPanel` (§6) rather
than silently dropping the tap.

#### `execute_action` (`ExecuteAction.action`)

```
# goal
string engine_id
string action_id
string params_json     # from ActionAvailability.suggested_params_json, or
                        # bridge-constructed if the button needed user
                        # input beyond a one-tap default
---
# result
rammp_orchestrator_interfaces/Result result
---
# feedback
rammp_orchestrator_interfaces/Progress progress
```
`engine_id`/`action_id` are exactly the pair the bridge resolved from the
tapped `button_id` (see the `select_object` note above). Feedback
(`Progress`: `phase`, `pct`, `message`, `pausable`) streams into the
`InteractionPanel`'s progress field (§5.2.C) for the lifetime of the goal;
`Progress.pausable` tells the bridge whether to keep a "Pause" button live
on the panel for this step. `Result` (`status` one of
`STATUS_SUCCESS`/`STATUS_FAILED`/`STATUS_CANCELLED`, plus `payload_json`,
`error_code`, `recoverable`, `suggestions[]`) arrives once and should
collapse the in-progress panel into a terminal state: a success/failure
message, and -- per `recoverable`/`suggestions` -- either a retry button
(re-issuing the same `execute_action` goal) or a dismiss/deselect button.

**Note on the standalone `~/progress` topic.** The Orchestrator node also
publishes every `Progress` update as a plain topic
(`<orchestrator_ns>/progress`, type `Progress`) in addition to relaying it
as this action's feedback -- the same message, published twice. The bridge
should prefer the action feedback channel (it's scoped to the specific
goal the bridge itself issued), but the topic is available as a fallback
if a future design needs progress visibility decoupled from holding an
active goal handle (e.g. a supervisory/read-only UI).

#### `get_world_state` (`GetWorldState.srv`)

```
# request: (empty)
---
# response
string snapshot_json
int64 version
```
Polled at `world_state_poll_period`; pushed to UE (`UpdateWorldState`,
§5.2.D) only when `version` changes from the last *pushed* value.

#### `get_detected_objects` (`GetDetectedObjects.srv`)

```
# request: (empty)
---
# response
rammp_orchestrator_interfaces/ObjectRecord[] objects
```
Each `ObjectRecord`: `instance_id`, `perception_class`, `last_pose_json`,
`location`, `opened` (tri-state), `last_seen`, plus two **proposed new
fields** -- `box` (`[x_min, y_min, x_max, y_max]` pixel coords, empty when
not currently visible) and `stream_id` (which camera stream `box` is in
pixel-space for) -- see the design-decision note just below. Polled at
`detected_objects_poll_period`; pushed to UE (`UpdateDetectedObjects`,
§5.2.D) only when the `instance_id` set changes from the last *pushed*
set (or, once `box` exists, whenever any pushed object's `box` changes
materially -- exact threshold is an implementation tuning detail, not a
design question).

**Note: UE itself never sends `deselect`/`control`/`execute_action`
commands.** See §5.1 -- the bridge turns each available control/action
option into a button on the current `InteractionPanel`, and UE only ever
reports back which button was tapped (`button_press` + `button_id`). This
table's "Used for" column describes what the bridge calls once it resolves
that `button_id`, not a direct UE→Orchestrator pass-through.

**Design decision: bbox comes from `ObjectRecord`, not a direct Bridge ↔
Perception subscription.** An earlier revision of this doc had the bridge
subscribe directly to `rammp_perception_interfaces/ObjectDetections` to
get pixel-space boxes, reasoning that `ObjectRecord` doesn't carry them.
That's superseded: **detections are not always perception-sourced.**
`UpdateObjectState.srv` lets a *remote engine* merge fields onto an
`ObjectRecord` mid-execution -- e.g. a manipulation engine's wrist camera
refining an object's box/pose once it's selected and that engine is
active, overwriting whatever the generic Perception subscription last
wrote for that `instance_id`. If the bridge read boxes from Perception
directly, it would only ever see the generic/initial-scene detection and
would miss these engine-sourced refinements entirely -- silently showing
a stale overlay position during exactly the phase (active manipulation)
where precision matters most.

The Orchestrator already merges both sources into one `ObjectRecord` per
`instance_id` (Perception's topic via its own `_on_object_detections`
handler, and engines via `update_object_state`) -- it's the only place
that has the single, freshest, source-agnostic view. So the fix is
**upstream**: add `box`/`stream_id` to `ObjectRecord.msg`
(`rammp_orchestrator_interfaces`), populated by whichever of those two
write paths last touched that instance. The bridge then gets box data
purely from this already-polled service, with no separate Perception
dependency and no risk of reading a stale/wrong-source value. **Not yet
implemented** -- tracked in §7 as an upstream interface change.

This also removes the need for a mask/`label_image`-based *precise*
hit-test mechanism (floated in an earlier revision): since the freshest
box may come from an engine's own camera rather than the shared scene
Perception feed, there's no single mask image guaranteed to cover every
object consistently. `box`-only hit-testing (UE does a simple
point-in-rectangle check against the latest pushed `box`) is simpler and
is now the **sole** tap-to-select overlay mechanism, not a fallback.

### 3.2 Bridge ↔ Camera

No shared interface package defines this today, and nothing in this
workspace publishes it yet -- the camera driver (e.g. a RealSense ROS2
node) that would own this feed isn't part of this workspace either. The
bridge would subscribe directly to its raw `sensor_msgs/Image` color/
depth topics (color + aligned depth, likely with `sensor_msgs/CameraInfo`
alongside for intrinsics) to get the actual pixel data for the "RGBD"
source in §5.2.A's camera stream.

This is deliberately its own peer, not folded into §3.1 (Orchestrator,
which never touches pixel data) or §3.3 (Engine node, which owns
*engine-specific* cameras like a wrist cam tied to one engine's
activation lifecycle): the scene camera exists independent of any
engine or task, and Perception (§3.1's `get_detected_objects`/
`ObjectRecord` note) is itself only a *consumer* of this same feed
(`Detect.srv`/`StartTracking.srv` take an `image_topic` parameter rather
than publishing one) -- so the bridge reading the camera directly is
peer to Perception reading it, not a dependency on Perception.

**Still TBD** (tracked in §7): no camera driver node/topic exists in this
workspace yet, so the exact topic name(s)/encoding/whether there's one
scene camera or several can't be pinned down until one does.

### 3.3 Bridge ↔ Engine node

No shared interface package defines this today -- it's **per-engine, not
generic**, unlike §3.1/§3.2. The reference engine wiring
(`rammp_orchestrator.ros.engine_node.EngineNode`) only advertises
`~/activate`/`~/hibernate`/`~/control`/`~/execute_action` -- all
Orchestrator-facing, not a camera topic a UI would subscribe to -- and
no mock or real engine in this workspace currently publishes one.

**Scope is deliberately narrow: a camera stream, and nothing else.**
While an engine is active and working a task, it may have its own camera
(e.g. a wrist cam) that's more relevant to the user than the scene
camera -- and unlike §3.2's scene feed, this engine stream is expected to
already have any interaction mask/overlay the engine wants to show
**rendered directly into the RGB pixels** by the engine itself (burned-in,
not a separate mask/segmentation channel), since this is a
task-in-progress view for the user to watch, not a tap-to-select surface.
Concretely, once a real engine exists: the bridge would take a
per-engine topic-name parameter (or a small per-engine-id config table,
since different engines may publish different image shapes/encodings)
and subscribe directly only while that engine is activated/selected in
the Orchestrator, switching the §5.2.A channel over to it.

**No detected-objects push belongs on this peer.** An engine that
refines or produces its own detections during a task (e.g. a wrist
camera finding a more precise grasp pose) reports them to the
**Orchestrator** via `update_object_state` (§3.1), the same path any
other detection source uses -- it does not push them to the bridge
directly. The bridge still gets detected-object data for such a task
from §3.1's `get_detected_objects`/`ObjectRecord.box`, exactly as when
no engine is active; this peer carries pixel data only.

**Still TBD** (tracked in §7): no concrete engine publishes this camera
topic yet, so the exact topic name/message type/per-engine config shape
can't be pinned down until a real engine (e.g. a manipulation engine with
a wrist camera) exists to look at.

### 3.4 Bridge ↔ Hardware node

No shared interface package defines this today either, and no hardware
driver node exists in this workspace yet. Unlike §3.3's engine camera
(which only exists/matters while a specific engine is active), this
telemetry -- arm joint states, chair/base speed and status -- is
**always-on and independent of engine selection**, analogous to how
§3.2's scene camera is independent of Perception. It's expected to come
from whatever node(s) directly own that hardware (e.g. an arm driver
publishing `/joint_states`, a base/chair driver publishing speed/odometry/
fault status) rather than being proxied through any engine or through the
Orchestrator's generic World-State JSON snapshot (`get_world_state`'s
`snapshot_json`, not meant for high-frequency structured telemetry).

This is the source for §5.2.B's "system status" channel.

**Still TBD** (tracked in §7): no hardware driver node/topic exists in
this workspace yet, so the exact topic names/message types can't be
pinned down until one does.


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

- **RGBD from the scene camera (§3.2)** -- color + depth; detected-object
  information for that frame is pushed separately (§5.2.D), not in-band.
- **RGB-only from an active Engine node's own camera (§3.3)** -- e.g. a
  wrist camera, with any interaction mask/overlay the engine wants to
  show already **rendered into the RGB pixels by the engine itself**
  (not a separate mask channel). Only present while that engine is
  active; detections during this time still arrive via §3.1/§5.2.D,
  exactly as with the scene-camera source -- this stream never itself
  carries detected-object semantics.

Each source gets its own channel/`stream_id`, tagged with
`source`/`role`/`group` metadata so UE can route each frame to the right
material/viewport -- same conventions as Gen1's per-camera channel
numbering (see that bridge's `image_channel`/`depth_channel`/`mask_channel`
parameters for precedent).

**Important correction re: "detected objects" transport.** The Gen1
streaming protocol itself (`StreamHeader`/`StreamMessage`) carries **no
semantic labelling** -- its metadata is just `w`/`h`/`fmt`/`group`/`role`/
`stream_id`/`name`/`material_params`. In Gen1, the actual object semantics
(e.g. "segment ID 1 = cup, state = full") were pushed **separately** via
Remote Control calls (`UpdateCupInfo`/`UpdateButtonInfo`), correlated with
the mask stream only by convention, not carried in-band. This bridge does
the same: this channel carries pixel data only (color/depth, and
optionally a plain visual segmentation overlay with no hit-test role --
see below), while all detected-object semantics (class, instance ID, 2D
bounding box) are pushed as data on the **existing detected-objects push**
(§5.2.D, §3.1's `ObjectRecord.box`), *not* on `InteractionPanel` (§5.2.C),
which stays reserved for the task/button dialog only. **Open/TBD:** the
raw RGB/depth image topics themselves (the scene camera's output, §3.2,
and an Engine node's wrist-camera output, §3.3) aren't defined by any
interface package in this workspace yet -- both the camera driver node
and any real engine's wrist-camera topic remain to be defined before
this channel can be implemented; §3.1's `ObjectRecord.box` extension
already covers the detection *semantics* independent of this.

#### B. System status (robot proprioception)

Arm joints, chair/base speed and status, and similar small/structured/
high-frequency telemetry -- pushed via Remote Control preset function
calls (like Gen1's `setJoints`/`UpdateSystemState`), **not** the
streaming channel, since this is small structured data rather than bulk
pixel payloads. Sourced via direct **bridge ↔ hardware node**
subscriptions (§3.4), not through the Orchestrator's generic World State
and not through any engine -- this telemetry is always-on and
independent of which engine (if any) is currently active, unlike §3.3's
per-engine camera. **Open/TBD:** no hardware driver node in this
workspace publishes this yet (§3.4), so the concrete topic names/shapes
can't be pinned down until a real hardware driver (e.g. an arm or
chair/base controller) exists.


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
| `UpdateDetectedObjects` | `Objects`: array of `{instance_id, perception_class, last_pose, location, opened, last_seen, box, stream_id}` -- `box`/`stream_id` are the proposed `ObjectRecord` extension (§3.1) | `~/get_detected_objects` poll result's set of `instance_id`s differs from the last pushed set, or any pushed object's `box` changes materially (exact threshold TBD at implementation time) |

Mechanics:

1. The bridge polls `~/get_world_state` every `world_state_poll_period`
   and `~/get_detected_objects` every `detected_objects_poll_period`
   (§2) -- polling, not a subscription, since the Orchestrator exposes
   these as services, not topics.
2. Each poll response is compared against what was last actually pushed
   to UE (World State `version` number; the `frozenset` of detected
   `instance_id`s, plus each object's `box` once that field exists) --
   **not** against the previous poll response, so a dropped/late poll
   can't cause a spurious re-push.
3. Only on a real change does the bridge call the corresponding UE preset
   function, once, with the full new snapshot/object list (not a diff) --
   UE always receives a complete, self-consistent payload rather than
   incremental patches to reconcile.
4. Per-object JSON shape (`_object_record_to_dict` in the reference
   implementation): `instance_id`/`perception_class` as-is;
   `last_pose_json` is JSON-decoded into a nested `last_pose` object
   (`{}` if empty); `location`/`opened`/`last_seen` passed through as-is.
   `box`/`stream_id` pass through as-is too, once `ObjectRecord` carries
   them (§3.1); empty/omitted when the object currently has no box (not
   presently visible to whichever source -- Perception or an active
   engine -- last wrote this `instance_id`).

`box` is UE's **sole** tap-to-select overlay mechanism (§3.1's
`get_detected_objects` note explains why a mask-based precise hit-test
was dropped: the freshest box may come from an engine's own camera, not
a single shared Perception mask). It's acceptable for `box` to lag
slightly behind the live video between pushes -- fine for a food-prep/
kitchen scene where objects don't move fast -- rather than needing
frame-exact sync with whatever's on screen.

This is the channel any future per-object metadata should be added to --
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

- **`ObjectRecord` needs a `box` (+ `stream_id`) extension** (§3.1's
  `get_detected_objects` design-decision note) -- an upstream
  `rammp_orchestrator_interfaces` change, populated by the Orchestrator
  from whichever of its two existing write paths last touched an
  instance: its own Perception subscription, or an engine's
  `update_object_state` call. This is the bbox gap's actual resolution
  (superseding an earlier revision's "bridge subscribes directly to
  Perception" design) -- not yet implemented.
- **Raw RGB/depth image topics for §5.2.A (camera streams, §3.2's scene
  camera and §3.3's per-engine camera)** are undefined -- no camera
  driver node, and no engine publishing its own camera, exist in this
  workspace yet. Camera streaming is an *intended* channel of this bridge
  (unlike in the prior version of this doc), but implementation is
  blocked on those source topics existing first. Note this is now purely
  a *pixel-data* gap -- the detected-object semantics (class, instance,
  box) that would otherwise ride along with a frame are already covered
  by the `ObjectRecord` extension above, independent of whether/when a
  raw camera topic exists, and independent of whether an engine is active
  (an active engine reports its own detections to the Orchestrator via
  `update_object_state`, not to the bridge directly -- see §3.3).
- **System status telemetry for §5.2.B (§3.4 Hardware node)** is
  undefined -- no hardware driver node in this workspace publishes joint
  states or chair/base speed and status today. System status is an
  *intended* channel of this bridge, but implementation is blocked on a
  real hardware driver node existing first.
- Joint-state → UE skeleton **mapping logic** (the specific joint-index
  layout the Gen1 prototype's `set_ui_joints` hardcoded for its chair+arm
  robot) does not carry over as-is, since it was specific to that
  hardware's joint layout -- §5.2.B/§3.4 needs its own mapping once a real
  hardware driver's joint-state shape is known.
- **Action button labels** (§3.1's `select_object` note) -- `ActionAvailability`
  carries only `action_id`, not the human-readable `ActionSpec.label` the
  Orchestrator already has at registration time. Needs an upstream
  `rammp_orchestrator_interfaces`/Orchestrator-core change (recommended:
  add `label`/`style_hint` to `ActionAvailability`) before `InteractionPanel`
  buttons can show real text instead of raw `action_id`s.

