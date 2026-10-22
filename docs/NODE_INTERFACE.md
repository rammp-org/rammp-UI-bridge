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

This node is a ROS2 **client only** on three separate peers -- it never
advertises any of the interfaces in this section itself (its own
server/publisher side is §4). The three peers are kept distinct because
they're different packages with different lifecycles, not just different
topics on one node:

- **§3.1 Bridge ↔ Orchestrator** -- the control/state surface: object
  selection, actions, pause/resume/cancel, world state, and the
  Orchestrator's own (coarse, world-frame) view of detected objects.
- **§3.2 Bridge ↔ Perception** -- a *direct* subscription to the
  perception stack's raw per-frame detections (pixel-space boxes/masks),
  bypassing the Orchestrator, for data the Orchestrator's `ObjectRecord`
  doesn't carry.
- **§3.3 Bridge ↔ Engine node** -- *direct* subscriptions to an
  individual engine's own telemetry/camera topics (e.g. joint states, a
  wrist camera), bypassing the Orchestrator, for data that's specific to
  one engine rather than part of generic World State.

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
`location`, `opened` (tri-state), `last_seen`. Polled at
`detected_objects_poll_period`; pushed to UE (`UpdateDetectedObjects`,
§5.2.D) only when the `instance_id` set changes from the last *pushed*
set. See the bbox note below for the (not yet available) tap-to-select
overlay data.

**Note: UE itself never sends `deselect`/`control`/`execute_action`
commands.** See §5.1 -- the bridge turns each available control/action
option into a button on the current `InteractionPanel`, and UE only ever
reports back which button was tapped (`button_press` + `button_id`). This
table's "Used for" column describes what the bridge calls once it resolves
that `button_id`, not a direct UE→Orchestrator pass-through.

**Note on tap-to-select overlays / bounding boxes -- now resolved via §3.2.**
`ObjectRecord` (this service's response) carries no camera-pixel-space
bbox -- just `instance_id`/`perception_class`/`last_pose_json` (world/
robot-frame)/`location`/`opened`/`last_seen`. That data genuinely isn't
here and was previously tracked as an upstream gap. It turns out the data
*does* already exist, just not on this service: `rammp_perception_interfaces`
publishes per-frame pixel-space boxes directly (`DetectedObject.box`,
keyed by the same `instance_id`). See §3.2 for the resolved design: the
bridge subscribes to perception's detections topic directly (bypassing
the Orchestrator) to get that data, and folds it into the same
detected-objects push (§5.2.D) by matching `instance_id`.

### 3.2 Bridge ↔ Perception

Defined in `rammp_perception_interfaces` (package `rammp-interfaces-ros2`).
The bridge is a **subscriber only** here -- it does not call
`~/detect`/`~/start_tracking`/`~/stop_tracking`/`~/renew_tracking`
(something else, e.g. a scene-management node or the Orchestrator's own
setup, owns the tracking session lifecycle; today nothing in this
workspace calls them either -- `orchestrator_node` only subscribes, same
as the bridge would).

| Perception interface | Type | Used for |
|---|---|---|
| `object_detections_topic` (default `/perception/object_detections`) | `ObjectDetections` (topic) | Subscribed continuously; per-object pixel-space `box`/`mask_id` folded into the detected-objects push (§5.2.D) alongside the Orchestrator's `ObjectRecord` fields |

`ObjectDetections` is `{header, DetectedObject[] objects, sensor_msgs/Image
label_image}`. Each `DetectedObject` carries: `instance_id` (same scheme
as `ObjectRecord.instance_id` -- `"<perception_class>_<n>"`, e.g.
`"microwave_2"`, so the two can be joined directly), `perception_class`,
`score`, `state` (`STATE_VISIBLE`/`STATE_OCCLUDED`), `box` (`[x_min, y_min,
x_max, y_max]` in source-image pixels, all-zero when not visible),
`mask_id` (its value in `label_image`, 0 when absent this frame),
`has_pose`/`pose`/`size` (3D, when perception can place it).

**This resolves the bbox gap** (formerly tracked in §3.1.1/§7): the bridge
joins each polled `ObjectRecord` (from §3.1's `get_detected_objects`,
world-frame/selection metadata) with the latest `DetectedObject` of the
same `instance_id` (from this topic, pixel-frame/overlay metadata) when
building the `UpdateDetectedObjects` push (§5.2.D), adding `box` and
whichever `stream_id` corresponds to that detection's source image. The
one remaining implementation detail (bridge-side config, not an upstream
interface gap) is the `image_topic → stream_id` mapping -- i.e. knowing
which camera stream a given `ObjectDetections.header.frame_id`/source
image corresponds to, so the box lands on the right video overlay in UE.

`label_image` (a `mono16` per-pixel `mask_id` image) is the actual
segmentation mask referenced in §5.2.A -- if a mask is streamed to UE, this
is its ROS2-side source, converted/re-encoded onto the camera-stream
channel (§5.2.A), not forwarded as-is over Remote Control.

`TrackingStatus` (published at a low fixed rate regardless of an open
session, with `objects` but no masks) is not currently needed by the
bridge, since `ObjectDetections` already carries everything the bridge
uses; it's available as a lighter-weight session-health signal if a future
need arises (e.g. surfacing "perception is not currently tracking
anything" in an error panel).

**Design decision: why this isn't instead "add `box` to `ObjectRecord`
and drop this section".** It's tempting to route `box` through the
Orchestrator too (it already subscribes to this same topic internally to
build `ObjectRecord`), so the bridge would only ever talk to one upstream
service. That gets rid of this subsection's *bbox* half, but not the rest
of it:

- `label_image` (the segmentation mask) is bulk per-frame pixel data --
  a `sensor_msgs/Image`, not something that fits `ObjectRecord`'s
  text/JSON shape or `get_detected_objects`' request/response polling
  pattern. It has to come from Perception directly regardless (it's what
  the camera-stream channel, §5.2.A, actually re-encodes and sends). So
  even with `box` added to `ObjectRecord`, the bridge would still need a
  direct `rammp_perception_interfaces` dependency for the mask -- this
  section wouldn't actually go away, just shrink to "subscribe for
  `label_image` only, ignore `DetectedObject.box`".
- Adding pixel-space geometry (`box`, and implicitly a camera-topology
  concept like `stream_id`) to `ObjectRecord` would couple the
  Orchestrator's core world-state model -- deliberately generic/
  perception-and-UI-agnostic, the same way `CapabilityManifest` is
  engine-agnostic -- to vision-specific concerns that are only meaningful
  to this one UI-bridge client. A future second UI client, or a non-visual
  consumer of World State, would have no use for it.

Given the mask dependency can't be removed either way, keeping both
`box` and `label_image` sourced from the same direct Perception
subscription (this section) is simpler overall than splitting one value
onto the Orchestrator and the other onto Perception, and keeps the
Orchestrator's interface unchanged. **This is the adopted direction** --
revisit only if a concrete future need removes the mask/pixel-hit-test
requirement entirely (see §5.2.D's design note), at which point `box`
alone moving to `ObjectRecord` would become a clean simplification.

### 3.3 Bridge ↔ Engine node

No shared interface package defines this today -- it's **per-engine, not
generic**, unlike §3.1/§3.2. The reference engine wiring
(`rammp_orchestrator.ros.engine_node.EngineNode`) only advertises
`~/activate`/`~/hibernate`/`~/control`/`~/execute_action` -- all
Orchestrator-facing, not telemetry a UI would subscribe to -- and no
mock or real engine in this workspace currently publishes joint states,
base odometry, or a wrist camera feed.

This category exists in the design because §5.2.A/B's "system status" and
"engine's own camera" sources are expected to come from **the specific
engine node that owns that hardware** (e.g. an arm-driver node's
`/joint_states`, a base driver's odometry, a manipulation engine's wrist
camera topic) rather than being proxied through the generic
Orchestrator/World-State surface, which only exposes a free-form JSON
snapshot (`get_world_state`'s `snapshot_json`) not meant for
high-frequency structured telemetry. Concretely, once a real engine
exists: the bridge would take a per-engine topic-name parameter (or a
small per-engine-id config table, since different engines publish
different shapes) and subscribe directly, independent of whether that
engine is currently activated/selected in the Orchestrator.

**Still TBD** (tracked in §7): no concrete engine publishes these topics
yet, so the exact topic names/message types/per-engine config shape can't
be pinned down until a real engine (e.g. an arm driver) exists to look at.

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
channel as pixel data only -- sourced from `ObjectDetections.label_image`
(§3.2) when perception produces one -- while any accompanying detected-
object semantics (class, instance ID, 2D bounding box, pose) are pushed as
data on the **existing detected-objects push** (§5.2.D, now including
pixel-space `box` data per §3.2), *not* on `InteractionPanel` (§5.2.C),
which stays reserved for the task/button dialog only. **Open/TBD:** the
raw RGB/depth image topics themselves (the Perception node's input, and
an Engine node's wrist-camera output) aren't defined by any interface
package in this workspace yet -- see §3.3. `ObjectDetections` (§3.2)
solves the *detections* half of this channel (masks/boxes); the *pixel*
half (the actual color/depth frames to stream) is still an open ROS2
source-topic question.

#### B. System status (robot proprioception)

Arm joints, base status, and similar small/structured/high-frequency
telemetry -- pushed via Remote Control preset function calls (like Gen1's
`setJoints`/`UpdateSystemState`), **not** the streaming channel, since this
is small structured data rather than bulk pixel payloads. Sourced via
direct **bridge ↔ engine node** subscriptions (§3.3), not through the
Orchestrator's generic World State -- each engine publishes its own
telemetry shape (e.g. `/joint_states` for an arm engine, odometry for a
base engine). **Open/TBD:** no engine in this workspace publishes this
yet (§3.3), so the concrete topic names/shapes/per-engine config can't be
pinned down until a real engine exists.

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
| `UpdateDetectedObjects` | `Objects`: array of `{instance_id, perception_class, last_pose, location, opened, last_seen, box, mask_id, stream_id}` -- the last three joined in from `ObjectDetections` (§3.2) by matching `instance_id` | `~/get_detected_objects` poll result's set of `instance_id`s differs from the last pushed set |

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
   `box`/`mask_id`/`stream_id` (from §3.2's perception join) are added if
   the bridge currently has a matching `ObjectDetections.objects[]` entry
   for that `instance_id`, omitted otherwise.

**Design note: why `box` stays on this change-triggered push instead of
going out per-frame.** `box` moves every camera frame (the object or
camera may move continuously), which would seem to need per-frame
updates -- but pushing per-frame via Remote Control RPC would defeat the
"change-triggered, not polled" design and flood UE with calls. Instead:
the **precise**, per-frame overlay mechanism is pixel hit-testing against
`label_image` (§3.2), which already streams continuously on the
camera-stream channel (§5.2.A) -- UE reads the `mask_id` at the tapped
pixel directly from the frame it's already displaying, and resolves it to
an `instance_id` via the `mask_id` this push just gave it. `box` itself is
a **coarser fallback** (e.g. for a perception mode that produces boxes but
no mask, or a simple "approximately here" highlight overlay) and is
intentionally allowed to lag slightly behind the live video between
instance-set changes -- acceptable for a food-prep/kitchen scene where
objects don't move fast, and not something UE should rely on for
pixel-exact hit-testing. If this trade-off proves wrong once real
payloads are measured, `box` can move to its own faster-cadence push
without changing `mask_id`'s role.

This is the channel §3.2's bounding-box/mask-id data and any future
per-object metadata should be added to -- `InteractionPanel` (§5.2.C) is
deliberately not used for this data, since it's keyed by `instance_id` on
its own change-triggered cadence, not tied to the task/button-dialog
lifecycle.

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

- **Raw RGB/depth image topics for §5.2.A (camera streams) and §3.3's
  engine-telemetry topics for §5.2.B (system status)** are undefined --
  no interface package in this workspace publishes camera frames, joint
  states, or base status today (see §3.3). Camera streaming and system
  status are *intended* channels of this bridge (unlike in the prior
  version of this doc), but implementation is blocked on those source
  topics/engine telemetry existing first. (The *detected-object*
  half of the camera-stream story -- masks and pixel-space boxes -- is
  resolved via §3.2's direct `ObjectDetections` subscription; it's only
  the raw pixel frames and engine-specific telemetry that remain open.)
- Joint-state → UE skeleton **mapping logic** (the specific joint-index
  layout the Gen1 prototype's `set_ui_joints` hardcoded for its chair+arm
  robot) does not carry over as-is, since it was specific to that
  hardware's joint layout -- §5.2.B/§3.3 needs its own mapping once a real
  engine's joint-state shape is known.
- **Action button labels** (§3.1's `select_object` note) -- `ActionAvailability`
  carries only `action_id`, not the human-readable `ActionSpec.label` the
  Orchestrator already has at registration time. Needs an upstream
  `rammp_orchestrator_interfaces`/Orchestrator-core change (recommended:
  add `label`/`style_hint` to `ActionAvailability`) before `InteractionPanel`
  buttons can show real text instead of raw `action_id`s.
- **`image_topic` → `stream_id` mapping** (§3.2) -- which camera stream a
  given `ObjectDetections` source image corresponds to is bridge-side
  config, not an upstream interface gap, but the concrete mapping/config
  shape isn't defined yet (depends on how many camera streams exist once
  §5.2.A's source topics are known).

