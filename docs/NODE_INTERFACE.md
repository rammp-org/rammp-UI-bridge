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

## 3. ROS2-facing interfaces

This node is a ROS2 **client only** on three of its four peers -- it never
advertises any interface to the Orchestrator, Camera, or Hardware node.
The exception is §3.3 Bridge ↔ Engine node, which is two-way: the bridge
is a *client* of the engine's camera topic, but a *server* for direct
engine-initiated user-input requests (new, this section) -- the
Orchestrator is not involved in, and has no visibility into, that
request/response exchange at all.

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
- **§3.3 Bridge ↔ Engine node** -- two parts: (a) a *direct subscription*
  to the currently-active engine's own camera stream (e.g. a wrist
  camera, with any interaction mask/overlay already rendered into the RGB
  pixels by the engine itself), strictly **additional** to §3.2's scene
  camera, not a replacement for it (§5.2.A); and (b) a *direct,
  Orchestrator-bypassing* request/response interface the bridge **serves**
  and an active engine **calls directly** whenever it needs user input --
  either a button choice (the engine supplies the panel content itself),
  an object tap on its own overlay stream from (a), or both at once. See
  the full design below.
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

**Interactive candidate objects during an active task are a separate,
Orchestrator-bypassing mechanism -- see §3.3.2, not this one.** An earlier
revision of this doc had engines push task-interactive candidates (e.g.
"what's pickable right now") via `update_object_state`, flowing through
this same `get_detected_objects`/`UpdateDetectedObjects` path. That's
superseded: soliciting the user's input during a task (whether by button
or by tapping a candidate object) is now a **direct Bridge ↔ Engine node**
exchange (§3.3.2's `RequestUserInput` action) that the Orchestrator has no
part in at all -- the engine supplies its own candidate objects directly
to the bridge as part of that request, and the bridge pushes them to UE
the same way (`UpdateDetectedObjects`, tagged with the engine's own
overlay `stream_id`), but without ever involving `ObjectRecord` or this
service. `update_object_state` remains valid for its original purpose --
an engine refining the pose/box of an object the Orchestrator is already
tracking (e.g. the one the user already selected before this action
started) -- just not for introducing new user-choosable candidates.

`stream_id` (above) is still what keeps box pixel-coordinates coherent
once the *source* camera changes: a box pushed via this service is in the
scene camera's pixel space (§3.2, the base layer in §5.2.A); a box pushed
via §3.3.2's direct mechanism is in *that engine's own camera's* pixel
space (the overlay layer) -- a different resolution/field of view in
general. UE must look up which stream a given `stream_id` refers to and
hit-test/draw the box against *that* stream's frame, not always the base
layer.

### 3.2 Bridge ↔ Camera

No shared interface package defines this today -- the camera driver
(the fleet's `rammp-suite-software` Gemini/OAK containers, per
`rammp-deployments/december_2026/sheppy-manifest.yaml`'s `scene_camera`
node) lives outside this workspace, not inside it. The bridge subscribes
directly to its raw `sensor_msgs/Image` color/depth topics (`color/
image_raw` + a depth topic, see below -- no `sensor_msgs/CameraInfo`
subscription yet, intrinsics aren't needed for anything implemented so
far) to get the actual pixel data for the "RGBD" source in §5.2.A's
camera stream. Implemented in `ui_bridge_node.py`'s
`_on_scene_camera_color`/`_on_scene_camera_depth` (params
`scene_camera_color_topic`/`scene_camera_depth_topic`); every frame is
landed and forwarded to UE as the §5.2.A base layer.

This is deliberately its own peer, not folded into §3.1 (Orchestrator,
which never touches pixel data) or §3.3 (Engine node, which owns
*engine-specific* cameras like a wrist cam tied to one engine's
activation lifecycle): the scene camera exists independent of any
engine or task, and Perception (§3.1's `get_detected_objects`/
`ObjectRecord` note) is itself only a *consumer* of this same feed
(`Detect.srv`/`StartTracking.srv` take an `image_topic` parameter rather
than publishing one) -- so the bridge reading the camera directly is
peer to Perception reading it, not a dependency on Perception.

**Topic names (resolved):** `/scene_camera/color/image_raw` for color,
always. Depth's topic name is **not** uniform across the `scene_camera`
alternatives a robot profile can select (per the sheppy manifest): the
Orbbec Gemini 336L (`gemini336l_docker`) publishes
`/scene_camera/depth/image_raw` (registered to the color frame), while
the OAK-D Pro W (`oak_d_pro_w_docker`, the current `flatbot` profile's
selection) publishes `/scene_camera/depth/image_rect_raw` instead -- the
bridge's `scene_camera_depth_topic` param defaults to the latter and
must be overridden per-robot if a different alternative is selected. The
`scene_camera_mock` alternative (used for dev/testing) only publishes
color, no depth, so depth-dependent code paths can't be exercised
against it yet.

### 3.3 Bridge ↔ Engine node

No shared interface package defines this today. This peer has two
distinct parts, one in each direction:

#### 3.3.1 Camera subscription (bridge is client)

It's **per-engine, not generic**, unlike §3.1/§3.2. The reference engine
wiring (`rammp_orchestrator.ros.engine_node.EngineNode`) only advertises
`~/activate`/`~/hibernate`/`~/control`/`~/execute_action` -- all
Orchestrator-facing, not a camera topic a UI would subscribe to -- and
no mock or real engine in this workspace currently publishes one.

While an engine is active and working a task, it may have its own camera
(e.g. a wrist cam) that's more useful to the user than the scene camera
for that moment -- and unlike §3.2's scene feed, this engine stream is
expected to already have any interaction mask/overlay the engine wants to
show **rendered directly into the RGB pixels** by the engine itself
(burned-in, not a separate mask/segmentation channel), since this is a
task-in-progress highlight for the user to watch, not a tap-to-select
surface on its own (see §3.3.2 for how tap-to-select works against it).
**It never replaces the scene camera** -- §3.2's feed keeps
subscribing/streaming continuously regardless, and this engine stream is
additive on top of it (UE composites the two, per §5.2.A) only for as
long as the engine is actually sending frames. Concretely, once a real
engine exists: the bridge would take a per-engine topic-name parameter
(or a small per-engine-id config table, since different engines may
publish different image shapes/encodings) and subscribe directly only
while that engine is activated/selected in the Orchestrator, feeding the
§5.2.A overlay layer for as long as that subscription has frames.

**Still TBD** (tracked in §7): no concrete engine publishes this camera
topic yet, so the exact topic name/message type/per-engine config shape
can't be pinned down until a real engine (e.g. a manipulation engine with
a wrist camera) exists to look at.

#### 3.3.2 User-input request (bridge is server) -- **proposed, not yet implemented**

**Design decision: an active engine asks the bridge for user input
directly, with the Orchestrator never involved.** An earlier revision of
this doc routed a mid-task object tap through a new Orchestrator service
(`provide_input`, forwarded to whichever engine holds execution
resources). That's been dropped in favor of this: the engine that's
*already* holding execution resources (so it, and only it, could
possibly be the one waiting on this) calls the bridge **directly** --
no intermediary, no Orchestrator routing logic, no extra hop. This fits
the existing division of responsibility better: the Orchestrator owns
*lifecycle* (activate/execute/pause/resume/cancel/hibernate), while the
bridge owns *everything UE-facing* -- and "ask the user something" is
squarely UE-facing.

Proposed shape (a new interfaces package, e.g. `rammp_ui_bridge_interfaces`,
since this is bridge-specific, not a generic Orchestrator concept):

```
# RequestUserInput.action (proposed)

# goal
string panel_json                              # optional: InteractionPanel-shaped JSON
                                                 # (title/message/buttons, §5.2.C) the engine
                                                 # wants shown; empty if this request is
                                                 # object-only
ObjectCandidate[] objects                       # optional: candidate objects the user can tap
                                                 # on this engine's own overlay stream (§3.3.1);
                                                 # empty if this request is panel-only
# (at least one of panel_json / objects must be non-empty)
---
# result
uint8 STATUS_BUTTON = 0      # user tapped one of panel_json's buttons
uint8 STATUS_OBJECT = 1      # user tapped one of objects
uint8 STATUS_CANCELLED = 2   # engine cancelled the goal before either happened
uint8 status
string button_id             # set iff status == STATUS_BUTTON
string instance_id           # set iff status == STATUS_OBJECT
---
# feedback: (none needed -- this is a single wait-for-one-tap request, not
# a long-running operation with intermediate progress)
```
`ObjectCandidate` (new, small, UI-facing -- not `ObjectRecord`, which is
an Orchestrator/world-state concept): just `{instance_id, perception_class,
box}`, the minimum UE needs to render and hit-test a tappable box. No
`stream_id` field is needed per-object since every object in one goal
implicitly belongs to the calling engine's own overlay stream -- the
bridge tags them all with that engine's known `stream_id` when forwarding
to UE.

Bridge behavior while a goal is active:
1. If `panel_json` is non-empty, push it to UE's `InteractionPanel`
   channel (§5.2.C) immediately -- this **takes over** whatever would
   otherwise be shown there (e.g. `ExecuteAction` progress, §3.1.1),
   reverting to the prior content once the goal ends.
2. If `objects` is non-empty, push them to UE's `UpdateDetectedObjects`
   channel (§5.2.D) tagged with the engine's own overlay `stream_id`,
   exactly like any other detected-objects push -- tappable the same way
   base-layer objects are.
3. Whichever happens first -- UE reports a `button_press` matching one of
   `panel_json`'s `button_id`s, or a `select_object` matching one of
   `objects`' `instance_id`s -- the bridge completes the goal with the
   corresponding result/status. There's only ever one result per goal.
4. The engine can cancel the goal at any time (e.g. it no longer needs
   the answer); the bridge then withdraws whatever panel/objects it had
   pushed on the engine's behalf and reports `STATUS_CANCELLED`.
5. Only one engine can have a goal active at a time, matching the
   Orchestrator's own one-engine-executing-at-a-time invariant -- a
   second concurrent goal should be rejected by the bridge.

**Bridge-side dispatch priority (§5.1):** a UE `select_object`/
`button_press` is resolved against an active `RequestUserInput` goal
*first*, if one exists and the tapped ID matches something it's
currently offering -- completing that goal rather than calling into the
Orchestrator at all. Only if there's no active goal, or the ID doesn't
match what it's offering, does the bridge fall back to the normal
Orchestrator dispatch (§3.1's `select_object`/`deselect`/`control`/
`execute_action`), unchanged from before this section existed.

**Still TBD** (tracked in §7): this entire interface is newly proposed --
the interfaces package it should live in, exact message/field names, and
how an engine discovers the bridge's own service/action namespace (a
fixed, well-known name is enough since there's exactly one bridge per
§1, but this still needs to be pinned down) are all undecided.

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

| Interface | Type | Direction | Meaning |
|---|---|---|---|
| `~/ue_connection` | `std_msgs/Bool` | published | Whether the UE WebSocket is currently connected, at `connection_publish_period` |
| `~/request_user_input` | `RequestUserInput.action` (proposed, §3.3.2) | server | Called directly by an active engine node, bypassing the Orchestrator, whenever it needs the user to tap a button or a candidate object |

Aside from `~/request_user_input` (§3.3.2), no other services or actions
are advertised by this node to other ROS2 clients; all UI interaction
besides that one engine-facing exception happens over the UE WebSocket,
not ROS2.

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

**Dispatch priority: an active engine input request (§3.3.2) wins
first.** Before doing anything else with a `select_object`/`button_press`
command, the bridge checks whether a `RequestUserInput` goal is currently
active *and* the tapped ID matches something that goal is offering (one
of its `objects`' `instance_id`s, or one of its `panel_json`'s
`button_id`s). If so, the bridge completes that goal with the matching
result and does **not** call the Orchestrator at all for this tap. Only
when there's no active goal, or the tapped ID doesn't match what it's
offering, does the table below apply.

| `cmd` | Fields | Bridge behavior (no active engine input request matches) |
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

Uses the RMSS binary TCP streaming protocol from the
[`ramms-tools`](https://github.com/rammp-org/ramms-tools) dependency
(MIT licensed; install with
`pip install git+https://github.com/rammp-org/ramms-tools.git` -- see
README.md's "Prerequisites"), imported as `ramms_tools.streaming` rather
than vendored -- the bridge only uses `StreamSender`, since it never
receives -- `StreamSender.send_image`/`send_depth_uint16` framing, not
the Remote Control WebSocket, since this is bulk pixel data. Two layers,
not two interchangeable sources:

- **Base layer -- RGBD from the scene camera (§3.2), always streaming.**
  Color + depth, continuously, regardless of whether any engine is
  active. Detected-object information for a given frame is pushed
  separately (§5.2.D), not in-band. **Implemented**:
  `_forward_scene_camera_color`/`_forward_scene_camera_depth` push every
  frame landed by §3.2's subscriptions straight through to UE, on fixed
  channels 0 (color)/1 (depth), `group="scene"`, `stream_id="scene/color"`/
  `"scene/depth"`. Depth is sent via `send_depth_uint16` for `16UC1`/
  `mono16` encodings (native camera mm depth, no conversion); any other
  depth encoding (e.g. `32FC1`) falls back to generic `send_image` with
  `fmt` set to the ROS encoding string, pending a real depth source to
  confirm which encodings actually show up.
- **Overlay layer -- RGB from an active Engine node's own camera (§3.3),
  optional and additional.** An engine only opens this second stream
  when it has something detailed to show the user (e.g. a wrist-camera
  close-up during a manipulation step); the base layer keeps streaming
  the whole time. UE composites the overlay on top of the base layer
  (with the base layer blurred/dimmed behind it) whenever frames are
  arriving on the overlay stream, and falls back to just the plain base
  layer the moment the engine stops sending (stream ends/times out) --
  it's a highlight on top, not a channel switch. Any interaction
  mask/overlay the engine wants to show is already **rendered into this
  stream's RGB pixels by the engine itself** (not a separate mask
  channel); detections during this time still arrive via §3.1/§5.2.D,
  exactly as with the base layer -- neither layer itself carries
  detected-object semantics. **Not implemented yet** -- no real engine
  publishes a wrist-camera topic to subscribe to.

Each layer gets its own channel/`stream_id`, tagged with
`source`/`role`/`group` metadata so UE can route each frame to the right
material/viewport (base vs. overlay) -- same conventions as Gen1's
per-camera channel numbering (see that bridge's
`image_channel`/`depth_channel`/`mask_channel` parameters for precedent).
The bridge is a TCP **client** of UE's RMSS streaming server (param
`ue_stream_port`, default `30030`, same host as `ue_host`); connection is
kept alive by a dedicated reconnect-loop thread
(`_stream_reconnect_loop`), independent of the Remote Control WebSocket's
own reconnection -- frames are simply dropped (not queued) while
disconnected, since only the newest frame matters for a live feed.

**Important correction re: "detected objects" transport.** The Gen1
streaming protocol itself (`StreamHeader`/`StreamMessage`) carries **no
semantic labelling** -- its metadata is just `w`/`h`/`fmt`/`group`/`role`/
`stream_id`/`name`/`material_params`. In Gen1, the actual object semantics
(e.g. "segment ID 1 = cup, state = full") were pushed **separately** via
Remote Control calls (`UpdateCupInfo`/`UpdateButtonInfo`), correlated with
the mask stream only by convention, not carried in-band. This bridge does
the same: both layers carry pixel data only, while all detected-object
semantics (class, instance ID, 2D bounding box) are pushed as data on the
**existing detected-objects push** (§5.2.D, §3.1's `ObjectRecord.box`),
*not* on `InteractionPanel` (§5.2.C), which stays reserved for the
task/button dialog only. **Open/TBD:** the overlay layer (an Engine
node's wrist-camera output, §3.3) still isn't published by any real
engine yet, so that forwarding path is untested; the exact UE-side
blur/composite mechanics (how strong, whether it's a material
parameter on the base layer or a UE post-process); §3.1's
`ObjectRecord.box` extension already covers the detection *semantics*
independent of either.

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

**Engine-driven override (§3.3.2):** while an active engine's
`RequestUserInput` goal has a non-empty `panel_json`, that content
**takes over this channel entirely**, replacing whatever would otherwise
be shown (e.g. `ExecuteAction` progress, or the action/control buttons for
the currently selected object). The bridge pushes it as-is (it's already
`InteractionPanel`-shaped) and resolves any resulting `button_press`
against that goal, not against the normal Orchestrator-call mapping
above (per §5.1's dispatch priority). Once the goal completes or is
cancelled, the bridge reverts to pushing whatever panel content the
normal Orchestrator-driven state (selected object, in-progress action,
etc.) calls for.

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
- A `button_press`/`select_object` referencing a `button_id`/`instance_id`
  the bridge no longer recognizes (panel already changed, object no
  longer detected) must not crash the node or silently no-op -- report it
  via an error-styled `InteractionPanel` (§5.2.C) so the user sees why
  their tap did nothing, rather than the UI appearing to hang.
- Loss of connectivity to any single ROS2 peer (§3.2 Camera, §3.3 Engine
  node, §3.4 Hardware node) must not crash the node or block pushes
  derived from the *other* peers -- each source's absence should degrade
  only its own UE channel (e.g. frozen/blank overlay video, stale system
  status) while Orchestrator-sourced channels (`InteractionPanel`,
  detected objects, world state) keep working normally.
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
- **Perception's start/stop lifecycle during an active task is an
  unconfirmed assumption, not a documented fact.** §3.1's
  `get_detected_objects` note assumes Perception may be paused/stopped
  while an engine is active (e.g. the scene camera's view is obstructed
  by the arm, or to free compute), which is why an active engine's own
  task-interactive candidates are offered via §3.3.2's direct
  `RequestUserInput` mechanism instead, rather than through Perception's
  base-layer feed. But nothing in this workspace currently starts/stops
  Perception's tracking session (`StartTracking.srv`/`StopTracking.srv`)
  -- not the Orchestrator, not this bridge -- so whatever node *does* own
  that decision, and the exact conditions under which it stops
  Perception, still needs to be confirmed before this behavior can be
  relied on.
- **Raw RGB image topic for §3.3.1's per-engine camera (overlay layer,
  §5.2.A)** is still undefined -- no engine publishes its own camera yet.
  The scene camera side (§3.2, base layer) is resolved and implemented:
  real topics subscribed to, frames forwarded to UE over the vendored
  Gen1 RMSS streaming protocol. The overlay layer's detected-object
  semantics (class, instance, box) are already covered by §3.3.2's
  `ObjectCandidate` independent of whether/when a real wrist-camera topic
  exists; the base layer's equivalent is the `ObjectRecord` extension
  above.
- **§3.3.2's `RequestUserInput` interface is newly proposed and entirely
  unimplemented.** No `rammp_ui_bridge_interfaces` (or equivalent)
  package exists yet to define the action/message; no engine calls it;
  the bridge doesn't serve it. Open questions: the interfaces package's
  name/location, exact field names, and how an engine resolves the
  bridge's fixed, well-known action name (presumably no discovery needed,
  since there's exactly one bridge per §1, but this hasn't been
  confirmed against how engine nodes are typically configured/launched).
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
- **Idle/default `InteractionPanel` content is undefined.** The
  `hibernate` note (§3.1.1) says the bridge clears the panel "or replaces
  it with whatever default/idle panel state is defined" once the
  Orchestrator returns to Browse mode -- but no such default panel (what
  title/message/buttons, if any, UE shows when nothing is selected and no
  task is running) has actually been specified yet. Needed before the
  bridge's startup/idle behavior is fully defined, not just its
  task-in-progress behavior.
- **Action button labels** (§3.1's `select_object` note) -- `ActionAvailability`
  carries only `action_id`, not the human-readable `ActionSpec.label` the
  Orchestrator already has at registration time. Needs an upstream
  `rammp_orchestrator_interfaces`/Orchestrator-core change (recommended:
  add `label`/`style_hint` to `ActionAvailability`) before `InteractionPanel`
  buttons can show real text instead of raw `action_id`s.

