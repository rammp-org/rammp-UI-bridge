# UI Bridge Node Interface

This document specifies the `rammp_ui_bridge` node's interface:
language-agnostic (applies equally to a Python/`rclpy` or C++/`rclcpp`
implementation). **The bridge has no dependency on
`rammp_orchestrator_interfaces` at all** -- every caller (the
Orchestrator's own always-open browse session, or an active engine) talks
to it exclusively through `rammp_ui_bridge_interfaces`'s
`RequestUserInput`/`UpdateUserInputPanel` mechanism (§3.1) plus the UE
Remote Control protocol. See `docs/LANGUAGE_DECISION.md` for the
implementation-language question itself.

## 1. Node identity

- Node name: `ui_bridge_node`
- Talks to exactly one UE instance (addressed by host/port/preset
  parameters), plus whichever ROS2 node currently holds the
  `ui_control` arbitration grant (§3.1.1) -- the Orchestrator by default,
  or an active engine while it executes an action. The bridge itself has
  no notion of "the Orchestrator" as a distinct peer anymore: it never
  calls out to it, it just serves whoever holds the grant.

## 2. Parameters

| Name | Type | Default | Meaning |
|---|---|---|---|
| `ue_host` | string | `127.0.0.1` | IP/hostname of the UE instance |
| `ue_ws_port` | int | `30020` | UE Remote Control WebSocket API port |
| `ue_preset` | string | `RAMMP` | UE Remote Control preset name to register against |
| `connection_publish_period` | double (seconds) | `1.0` | Interval between connection-status publishes |

## 3. ROS2-facing interfaces

This node is a ROS2 **client only** on two of its three peers (Camera,
Hardware node) -- it never advertises any interface to them. The
exception is §3.1 (and, within it, §3.3's engine camera subscription),
which is two-way: the bridge is a *server* for `RequestUserInput`/
`UpdateUserInputPanel`/`ui_control` arbitration, called by whichever
caller currently holds the grant, and a *client* of that caller's overlay
camera topic while it holds the grant.

- **§3.1 Bridge ↔ active RequestUserInput caller** -- the single, generic
  mechanism every caller (the Orchestrator's own always-open browse
  session, or an engine while it executes an action) uses to push a
  panel/candidate-object list to UE and receive taps back, gated by
  `ui_control` token arbitration. There is no separate, Orchestrator-
  specific command surface anymore -- the Orchestrator talks to the
  bridge exactly the way an engine does.
- **§3.2 Bridge ↔ Camera** -- a *direct* subscription to the scene
  camera's raw RGB/depth topics, for the pixel data actually streamed to
  UE (§5.2.A). This is a camera driver node (e.g. a RealSense node), not
  Perception -- Perception is itself just a consumer of this same feed
  (`Detect.srv`/`StartTracking.srv`'s `image_topic` parameter), not its
  source.
- **§3.3 Bridge ↔ Engine node** -- the engine-specific half of §3.1's
  mechanism: (a) a *direct subscription* to the currently-active engine's
  own camera stream (e.g. a wrist camera, with any interaction
  mask/overlay already rendered into the RGB pixels by the engine
  itself), strictly **additional** to §3.2's scene camera, not a
  replacement for it (§5.2.A); and (b) the engine's use, while it holds
  `ui_control`, of the same `RequestUserInput`/`UpdateUserInputPanel`
  server §3.1 describes -- nothing engine-specific about the protocol
  itself, just about *when* an engine is the caller instead of the
  Orchestrator. See the full design below.
- **§3.4 Bridge ↔ Hardware node** -- a *direct* subscription to
  always-on, hardware-level robot telemetry (arm joint states, chair/base
  speed and status) that exists independent of which engine (if any) is
  currently active, unlike §3.3's per-engine camera.

There is deliberately **no direct Bridge ↔ Perception interface, and no
direct Bridge ↔ Orchestrator interface either** -- every object/panel the
Orchestrator wants shown reaches the bridge through the same
`RequestUserInput`/`UpdateUserInputPanel` calls an engine would use (§3.1);
see the design-decision note there for why.

### 3.1 Bridge ↔ active `RequestUserInput` caller

**Design decision (final): there is exactly one caller-facing mechanism,
used identically by the Orchestrator and by any engine.** Earlier
revisions of this doc described two separate things: a Bridge ↔
Orchestrator control/state surface (`select_object`/`deselect`/`control`/
`execute_action`/`get_detected_objects`, all defined in
`rammp_orchestrator_interfaces` and polled/called directly against the
Orchestrator), and a *separate*, "Orchestrator-bypassing" §3.3.2
mechanism an active engine could use instead. **That split is gone.** The
Orchestrator does not get a privileged interface anymore -- it is simply
the caller that holds `ui_control` (§3.1.1) whenever no engine is
executing, via its own permanently-open `RequestUserInput` goal (a
"browse session", opened at startup and kept alive/refreshed for as long
as it holds the grant). An engine becomes the caller only for the
duration of one `execute_action` run, by acquiring the same grant. The
bridge's own dispatch code (`_try_resolve_active_input_goal`) has always
been caller-agnostic -- it just resolves a UE tap against whichever goal
is currently open -- so unifying the two callers onto one mechanism
required **no bridge-side protocol change**, only retiring the
now-redundant `rammp_orchestrator_interfaces`-based calls this section
used to describe. The bridge has **no dependency on
`rammp_orchestrator_interfaces` at all** as a result -- see
`rammp_ui_bridge_interfaces` for everything in this section instead.

This also means there is no more `get_detected_objects` polling: detected
objects reach UE purely as the `objects` list of whichever
`RequestUserInput` goal is currently active (the Orchestrator's own
browse session normally carries the full, continuously-`sync_objects`-
refreshed world-state object list; an executing engine's goal carries
just its own task-relevant candidates instead) -- see §3.1.1 and §5.2.D.
If no goal is active and a tap doesn't match what the active goal is
offering, the bridge simply logs a warning and drops it (§5.1) -- there
is no other dispatch path to fall back to.

#### 3.1.1 `ui_control` arbitration, and the `RequestUserInput`/`UpdateUserInputPanel` protocol

The bridge hosts a token-gated arbitration resource, **`ui_control`**,
reusing the same `AcquireControl`/`ReleaseControl`/`ControlStatus`
protocol that `rammp_arm_interfaces`/`rammp_base_interfaces` already use
for hardware arbitration (see `rammp_common_interfaces`). Whoever holds
the current token is the sole caller allowed to open/update a
`RequestUserInput` goal:

- `~/ui_control/acquire_control` (`rammp_common_interfaces/AcquireControl`) --
  called with an `owner_id` identifying the caller. Like the hardware-
  arbitration precedent, this call **seizes**: it always succeeds and
  always returns a fresh 16-byte token (or the caller's own
  pre-generated one, see below), even if another owner currently holds
  it -- a stuck/crashed owner can never permanently block a
  correctly-behaving new owner. Two callers in practice:
  - **The Orchestrator**, with `owner_id="rammp_orchestrator"`, at its
    own startup (opening the first browse-session goal) and again every
    time it reclaims control after an engine's `execute_action` goal
    finishes (regardless of outcome) -- the explicit "hand control back"
    step, and a safety net if the engine crashed without releasing.
  - **An engine**, by operational contract via the Orchestrator's own
    `~/acquire_ui_control` proxy (not this endpoint directly -- see
    `rammp-orchestrator/docs/ENGINE_DEVELOPER_GUIDE.md`), around the
    start of a pausable/cancellable `execute_action` run or any other
    time it needs to show its own panel/overlay. It holds the token for
    as long as it owns the live UI (e.g. the whole progress panel,
    including Pause/Cancel, for one `execute_action` goal) and
    releases/lets its own goal end when done. While an execution is in
    progress, the Orchestrator's acquire-proxy endpoints
    (`~/acquire_ui_control`, `~/acquire_arm_control`,
    `~/acquire_base_control`) all inject that one execution's own
    session token (`AcquireControl.Request.token` non-all-zero) instead
    of asking each resource to mint its own -- the bridge grants exactly
    that token when it sees one supplied, so an engine's `ui_control`
    token and its arm/base-control token (echoed back in messages like
    `rammp_arm_interfaces/TwistSetpoint.token`) end up identical for that
    execution.
- `~/ui_control/release_control` (`rammp_common_interfaces/ReleaseControl`) --
  called by the current owner (with its token) when it's done, e.g. from
  an engine's own `hibernate`. Refused (token mismatch) if called by
  anyone else.
- `~/ui_control/status` (`rammp_common_interfaces/ControlStatus`, published
  latched/transient-local) -- current `owner_id`/`generation`/`owned`/
  `rejected_count`, for anyone who wants to observe arbitration state
  without racing to acquire it themselves.

Once a caller holds a valid token, it uses that same token on every
`RequestUserInput`/`UpdateUserInputPanel` call below -- these two
interfaces are not described per-caller (Orchestrator vs. engine) because
they behave identically either way; only *which* caller currently holds
the token differs.

Lives in `rammp_ui_bridge_interfaces` (bridge-specific, not a generic
Orchestrator concept):

```
# RequestUserInput.action

# goal
string engine_id                                # calling owner's id -- "rammp_orchestrator"
                                                 # for the Orchestrator's own browse session, or
                                                 # an engine's own CapabilityManifest.engine_id
uint8[16] token                                  # must match the current ~/ui_control grant above
string request_json
# JSON object:
#   {
#     "panel_json": str,         # optional, default "" -- InteractionPanel-shaped JSON
#                                # (title/message/buttons/progress/style, §5.2.C) the caller
#                                # wants shown; omitted/"" if this request is object-only
#     "objects": [                # optional, default [] -- candidate objects the user can tap
#                                # (on the scene camera for the Orchestrator's own browse
#                                # session, or on the calling engine's own overlay stream,
#                                # §3.3.1); omitted/[] if this request is panel-only
#       {"instance_id": str, "perception_class": str, "box": [x_min, y_min, x_max, y_max],
#        "source": str}         # "scene" or "overlay" -- which video channel `box` is
#                                # relative to, see §5.2.D
#     ],
#     "selected_instance_id": str  # optional, default "" -- one of objects' instance_id to
#                                # show as already highlighted when the goal starts
#   }
# (at least one of panel_json / objects must be non-empty)
---
# result
string result_json
# JSON object:
#   {"status": "button", "button_id": str}   # user pressed one of panel_json's buttons
#   {"status": "cancelled"}                  # caller cancelled the goal before that happened
---
# feedback -- sent once per object tap while the goal is still active,
# never after the result above is sent. An object tap does NOT complete
# the goal by itself (see point 3 below).
string feedback_json
# JSON object: {"event_type": "object_tapped", "instance_id": str}
```

```
# UpdateUserInputPanel.srv -- called any number of times while a
# RequestUserInput goal is active, to change what's displayed without
# completing or restarting it (e.g. after an "object_tapped" feedback,
# to update the message and highlight the new selection; or, for the
# Orchestrator's own browse session, every tick, to refresh the
# candidate-object list as world state changes -- see
# `UiBridgeBrowseSession.sync_objects`).

uint8[16] token
string request_json
# JSON object, every key optional -- only present keys are updated, an
# absent key leaves that aspect unchanged:
#   {
#     "panel_json": str,          # replaces the current panel entirely, iff present
#     "objects": [                # replaces the current candidate list entirely, iff present
#       {"instance_id": str, "perception_class": str, "box": [x_min, y_min, x_max, y_max],
#        "source": str}
#     ],
#     "selected_instance_id": str  # highlight this instance_id ("" clears), iff present
#   }
---
bool accepted
string message
```

Each candidate object in `objects` is `{instance_id, perception_class,
box, source}` -- the minimum UE needs to render and hit-test a tappable
box. `box` is empty/omitted when the object currently has no known pixel
position. `source` (`"scene"`/`"overlay"`) tells UE which video channel
`box`'s coordinates are relative to -- the Orchestrator's own browse
session always sends `"scene"` (its objects come from Perception/World
State); an engine's objects default to `"overlay"` (its own camera
stream) but may be `"scene"` per-object instead if the engine reuses
World State boxes rather than publishing its own camera -- so `source` is
caller-supplied per-object, not inferred by the bridge from who the
caller is (§5.2.D).

Bridge behavior while a goal is active:
1. If `request_json`'s `panel_json` is non-empty, push it to UE's
   `InteractionPanel` channel (§5.2.C) immediately -- this **takes over**
   whatever was previously shown there, reverting only once a new
   panel is pushed (by this goal, or by whoever's goal opens next).
2. If `request_json`'s `objects` is non-empty, push them to UE's
   `UpdateDetectedObjects` channel (§5.2.D) along with
   `selected_instance_id` (initially `request_json`'s own
   `selected_instance_id`), exactly like any other detected-objects push
   -- tappable the same way.
3. **An object tap does not complete the goal.** The user may tap
   several different candidates in a row while deciding; each tap is
   reported to the caller as an `{"event_type": "object_tapped", ...}`
   feedback message instead, and the bridge's own notion of "currently
   highlighted" updates to match. The caller decides what to do about
   each tap -- typically calling `UpdateUserInputPanel` with a
   `selected_instance_id` key to reflect the new choice back to the
   user (and, for the Orchestrator's browse session, a refreshed panel
   showing that object's available actions) -- without the goal ending.
   The goal only completes when UE reports a `button_press` matching one
   of the current panel's `button_id`s (`{"status": "button", ...}`); by
   then the caller already knows (from the feedback stream) which
   object, if any, is selected, and the pressed `button_id` tells it what
   to do with that selection.
4. The caller can cancel the goal at any time (e.g. it no longer needs
   the current input); the bridge then withdraws whatever panel/objects
   it had pushed on the caller's behalf and reports
   `{"status": "cancelled"}`.
5. Only one caller can have a goal active at a time -- a second
   concurrent goal is rejected by the bridge, as is any goal whose
   `token` doesn't match the current `~/ui_control` grant. In practice
   this means: the Orchestrator's own browse-session goal is open
   whenever nothing is executing; the moment an engine acquires
   `ui_control` for an `execute_action` run, the Orchestrator's goal has
   already ended (it released the grant first) and the engine's own goal
   takes its place; once that engine's goal ends (success, failure, or
   cancellation) and the Orchestrator reclaims `ui_control`, it opens a
   fresh browse-session goal reflecting the object's now-current
   available actions (e.g. `close`/`Deselect` after `open` finishes,
   flipping back to `open`/`Deselect` after `close`).

**Verified:** live smoke test (in-process, no real UE) exercising the
full lifecycle -- acquire `~/ui_control` token, send a goal with two
candidate objects, simulate two alternating object taps (each produces
exactly one feedback message, goal stays active), call
`UpdateUserInputPanel` with the right token (accepted) and a wrong token
(rejected), simulate a `button_press` (completes the goal with
`{"status": "button", ...}`), and confirm a second goal with a
stale/wrong token is rejected outright. **Also verified against a real
engine:** `fridge_ui_demo` (`rammp_orchestrator`'s `engine_node`) drives
this path for a real confirm-dialog + held live-progress-panel
`execute_action` flow end to end -- it holds `ui_control` for the whole
`open`/`close` run, confirms via a `RequestUserInput` goal before
starting, and pushes its own live progress panel (Pause/Cancel buttons,
`pct` updates) through `UpdateUserInputPanel` calls. **Also verified for
the Orchestrator's own browse session:** the Orchestrator opens its
browse-session goal at startup, keeps it synced with world state every
tick (`sync_objects`), and reopens it (with a freshly-reclaimed token)
immediately after each engine's `execute_action` run ends -- live-verified
end to end via `mock_ue`'s HTTP API: tap fridge → select → confirm dialog
→ live progress panel → pause genuinely halts `pct` → resume continues →
completion → the Orchestrator's reclaim refreshes the panel to the
object's now-current available actions, with no more direct
Orchestrator-service fallback involved at any point.

**On action-progress feedback:** an engine holds `ui_control` for the
whole pausable/cancellable run and pushes its own live progress panel
directly via `RequestUserInput`/`UpdateUserInputPanel` calls (above) --
including whatever Pause/Cancel buttons it wants shown, on its own
schedule. The bridge has no visibility into (and no interface describing)
the Orchestrator's own internal `execute_action` goal against the engine
-- that's purely between `orchestrator_node` and `engine_node`, never
proxied through this bridge; the panel the user actually sees comes
entirely from the engine's own `RequestUserInput`/`UpdateUserInputPanel`
calls above, not from any action-feedback relay.

#### Where detected objects' pixel-space boxes come from

Each candidate object (§3.1.1's `objects` list) carries `instance_id`,
`perception_class`, and `box` (`[x_min, y_min, x_max, y_max]` pixel
coords, empty when not currently visible).

**Design decision: box comes from whichever `RequestUserInput` goal is
active, not a direct Bridge ↔ Perception subscription.** An earlier
revision of this doc had the bridge subscribe directly to
`rammp_perception_interfaces/ObjectDetections` to get pixel-space boxes.
That's superseded: **detections are not always perception-sourced.** An
object's box can be refined mid-execution by an active engine's own
wrist camera, overwriting whatever the generic Perception feed last
reported for that `instance_id` -- the Orchestrator (not the bridge)
merges both sources into one authoritative, source-agnostic view per
`instance_id`, and supplies whichever is freshest as the `objects` list
of its own browse-session goal (or, while an engine is executing, the
engine supplies its own candidates directly, which may be at a different
resolution/field of view entirely). If the bridge read boxes from
Perception directly, it would only ever see the generic/initial-scene
detection and would miss engine-sourced refinements entirely.

This also removes the need for a mask/`label_image`-based *precise*
hit-test mechanism (floated in an earlier revision): since the freshest
box may come from an engine's own camera rather than the shared scene
Perception feed, there's no single mask image guaranteed to cover every
object consistently. `box`-only hit-testing (UE does a simple
point-in-rectangle check against the latest pushed `box`) is simpler and
is the **sole** tap-to-select overlay mechanism, not a fallback.

`source` (§5.2.D) is what keeps box pixel-coordinates coherent once the
*source* camera changes: a box pushed as part of the Orchestrator's own
browse-session goal is tagged `"scene"` and is in the scene camera's pixel
space (§3.2, the base layer in §5.2.A); a box pushed as part of an
executing engine's own goal is typically tagged `"overlay"` and is in
*that engine's own camera's* pixel space (the overlay layer) -- a
different resolution/field of view in general, though an engine may tag
a given object `"scene"` instead if it's reusing a World-State/scene-
camera box rather than one from its own camera. UE must look at each
object's own `source` to know which video channel to hit-test/draw that
object's `box` against, not always the base layer.

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
engine or task, and Perception (see §3.1's "Where detected objects'
pixel-space boxes come from" note) is itself only a *consumer* of this
same feed
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

**Implemented.** It's a **single shared topic, not per-engine**, unlike
an earlier revision of this section proposed. The reference engine
wiring (`rammp_orchestrator.ros.engine_node.EngineNode`) only advertises
`~/activate`/`~/hibernate`/`~/control`/`~/execute_action` -- all
Orchestrator-facing, not a camera topic a UI would subscribe to -- so no
engine-identity information is available to the bridge via that path
either way.

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
long as some engine is actually sending frames.

**Design decision: one fixed topic name (`engine_overlay_topic`, default
`/engine_overlay/image_raw`), the bridge is its only subscriber, and
every engine that has an overlay camera publishes
`rammp_ui_bridge_interfaces/EngineOverlayImage` to that same name** --
not a per-engine topic-name parameter/config table. This is safe in the
common case because the Orchestrator's resource-locking already
guarantees **only one engine can hold execution resources (and so be
doing anything worth showing an overlay for) at a time** -- see
`core/orchestrator.py`'s resource acquisition on activate/execute -- but
a crashed or slow-to-release engine could still leave a stale publisher
open, so the bridge doesn't just trust "whatever arrives." Instead:

**Token-gated arbitration.** The bridge is the arbitration *server* for
this resource (the "UI control" resource -- it also gates §3.3.2's
`RequestUserInput`, since both represent the one currently-active
engine's exclusive right to talk to the UI), reusing the same
AcquireControl/ReleaseControl/ControlStatus protocol that
`rammp_arm_interfaces`/`rammp_base_interfaces` already use for hardware
arbitration (see `rammp_common_interfaces`). The bridge hosts three
endpoints:

- `~/ui_control/acquire_control` (`rammp_common_interfaces/AcquireControl`) --
  called with an `owner_id` identifying the caller, whenever it's about to
  start publishing overlay frames and/or calling `~/request_user_input`.
  Like the hardware-arbitration precedent, this call **seizes**: it always
  succeeds and always returns a fresh 16-byte token (or the caller's own
  pre-generated one, see below), even if another owner currently holds
  it -- the "seize" semantics mean a stuck/crashed owner can never
  permanently block a correctly-behaving new owner. In practice there are
  two distinct callers:
  - **An engine** (by operational contract, via the Orchestrator's own
    `~/acquire_ui_control` proxy, not this endpoint directly -- see
    `rammp-orchestrator/docs/ENGINE_UI_INTEGRATION_GUIDE.md` §2.1),
    around the start of a pausable/cancellable `execute_action` run or
    any other time it needs to show its own panel/overlay -- not
    necessarily tied to `activate()`. It holds the token for as long as
    it owns the live UI (e.g. the whole progress panel, including
    Pause/Cancel, for one `execute_action` goal) and releases/lets its
    own goal end when done. While an execution is in progress, the
    Orchestrator's acquire-proxy endpoints (`~/acquire_ui_control`,
    `~/acquire_arm_control`, `~/acquire_base_control`) all inject that
    one execution's own session token (`AcquireControl.Request.token`
    non-all-zero) instead of asking each resource to mint its own -- the
    bridge grants exactly that token when it sees one supplied, so an
    engine's `ui_control` token and its arm/base-control token (echoed
    back in messages like `rammp_arm_interfaces/TwistSetpoint.token`)
    end up identical for that execution, letting the engine stamp one
    token everywhere instead of tracking several.
  - **The Orchestrator itself**, with `owner_id="rammp_orchestrator"`,
    once an `execute_action` goal finishes (regardless of outcome) --
    this is both the explicit "hand control back" step and a safety net
    if the engine crashed without releasing. Granting this reclaim is
    what triggers the bridge to refresh the panel: `_on_ui_acquire_control`
    recognizes this `owner_id` and re-runs `select_object` for whatever's
    currently selected (not the activations cached from the original
    tap, which would otherwise show pre-action availability, e.g. a
    fridge still offering "open" after it was just opened), then pushes
    the refreshed `InteractionPanel`. If nothing is selected, or the
    object is no longer selectable, it falls back to the idle panel.
- `~/ui_control/release_control` (`rammp_common_interfaces/ReleaseControl`) --
  called by the current owner (with its token) when it's done, e.g. from
  its own `hibernate`. Refused (token mismatch) if called by anyone else.
- `~/ui_control/status` (`rammp_common_interfaces/ControlStatus`, published
  latched/transient-local) -- current `owner_id`/`generation`/`owned`/
  `rejected_count`, for anyone who wants to observe arbitration state
  without racing to acquire it themselves.

Every `EngineOverlayImage` frame an engine publishes carries the token it
was granted (`EngineOverlayImage.token`). `_on_engine_overlay_frame`
compares the incoming token against the one most recently granted
(`self._ui_token`) and **silently drops** (incrementing `rejected_count`,
with a throttled warning log) any frame whose token doesn't match --
including frames from a stale/already-released owner. Only frames with a
matching token are forwarded (`_forward_engine_overlay_color`) to UE,
unconditionally, on fixed channel 2, `group="overlay"`,
`stream_id="engine/overlay"` -- exactly the same shape as the base
layer's forwarding, just one channel number up. An engine's expected
lifecycle is: call `acquire_control` once it starts needing the UI,
publish `EngineOverlayImage` frames (and/or call `~/request_user_input`,
§3.3.2) stamped with that token while active, and call `release_control`
(or just let its holding goal end) when done -- at which point the
Orchestrator reclaims control per the bullet above. Every grant/mint is
logged (`ui control acquired by '<owner_id>' (minted a new/using
orchestrator-supplied token ..., generation N)`) so it's visible in the
node's log which owner holds the UI and when it changed hands. UE's own
receive-timeout still handles falling back to the base-layer-only view if
nothing is forwarded for a while (see §5.2.A).

**Not yet decided:** whether this resource also needs a `RevokeControl`
(operator-override, no-token) path for a hard crash-recovery case -- not
implemented yet, deferred until a concrete need shows up.

**Still open:** no real engine in this workspace calls
`acquire_control`/publishes to this topic yet, so this path has only been
exercised with synthetic test traffic (acquire → publish-with-good-token →
publish-with-wrong-token → release → publish-with-stale-token), not a
real wrist camera + engine. The exact image shape/encoding a real
manipulation engine's wrist camera would publish is still unconfirmed.

#### 3.3.2 User-input request (bridge is server) -- see §3.1.1

**Superseded by §3.1.1.** This subsection used to describe a separate,
engine-specific `RequestUserInput`/`UpdateUserInputPanel` mechanism,
contrasted with a (then still-existing) Orchestrator-specific
`select_object`/`deselect`/`control`/`execute_action` surface. That split
no longer exists: §3.1.1 now describes the single, caller-agnostic
protocol used identically whether the current caller is the Orchestrator
(its own always-open browse session) or an engine (while it holds
`ui_control` for an `execute_action` run). Nothing engine-specific
remains to document here beyond what §3.1.1 already covers; this heading
is kept only so existing cross-references (§5.1, §5.2.A/C/D) resolve to
the right place.

### 3.4 Bridge ↔ Hardware node -- **arm joint states implemented, chair/base still TBD**

No shared interface package defines this (it's plain `sensor_msgs/msg/
JointState`, no bridge-specific message type needed). Unlike §3.3's engine
camera (which only exists/matters while a specific engine is active),
this telemetry -- arm joint states, chair/base speed and status -- is
**always-on and independent of engine selection**, analogous to how
§3.2's scene camera is independent of Perception. It comes from whatever
node(s) directly own that hardware (e.g. an arm driver publishing
`/joint_states`, a base/chair driver publishing speed/odometry/fault
status) rather than being proxied through any engine or through the
Orchestrator's generic World-State JSON snapshot (`get_world_state`'s
`snapshot_json`, not meant for high-frequency structured telemetry).

**Implemented: arm joint states.** The bridge subscribes directly to
`arm_joint_states_topic` (default `/joint_states`,
`sensor_msgs/msg/JointState`) -- see `rammp-deployments`'
`december_2026/sheppy-manifest.yaml`'s `arm` node, whose real alternative
(`kinova_gen3_driver`/`kinova_gen3_driver_nightly`) publishes the Kinova
Gen3's 7 arm joints plus the 2F-85 gripper's own joint on this topic, and
whose mock alternative (`arm_mock`) publishes the same topic name with
empty `name`/`position` arrays, purely to keep it wired up on a
workstation profile with no arm attached. **Design decision: forward
every joint in the message by name, not a fixed index layout.** The Gen1
prototype's `set_ui_joints` built a hardcoded 53-element float array
keyed to specific skeleton-bone indices for its one chair+arm robot --
that doesn't generalize (per §7's prior note) and this bridge has no
chair/base to combine it with anyway. Instead, `_on_arm_joint_states`/
`_forward_arm_joint_states` push the message's `name`/`position` arrays
through as-is (radians converted to degrees, matching Gen1's degree
convention for skeleton authoring) via a Remote Control call,
`UpdateArmJointStates` (`{"Names": [...], "PositionsDeg": [...]}`) -- UE's
own blueprint is responsible for mapping each joint name to its own
skeleton bone, so the bridge needs no knowledge of the UE skeleton's
layout and "use that for all the arm joints" (whatever the publishing
node names them) just works without a bridge-side mapping table. An
empty `JointState` (the mock) forwards as empty arrays; UE is expected to
simply leave its skeleton unchanged on an empty update.

This is the source for §5.2.B's "system status" channel (the arm-joints
portion of it).

**Verified:** a live smoke test (bridge node + a plain publisher node in
one process) confirmed a populated `JointState` (7 arm joints + gripper)
forwards with matching names and correctly-converted degree values, and
that an empty `JointState` (the real shape of `arm_mock`'s own output)
forwards as empty arrays without error. The jetson's `arm_mock` (sheppy
node `arm`) has been confirmed running and publishing `/joint_states` on
the shared robot; the bridge's own deployed copy on that jetson has not
yet been updated to this session's commit, so end-to-end verification
against the live mock over the real ROS graph is still pending a
redeploy (not requested this session).

**Still TBD** (tracked in §7): chair/base speed and status -- a mock
base driver exists now (`rammp_orchestrator/ros/mock_base_driver_node.py`),
but it only consumes `rammp_base_interfaces/TwistSetpoint` for
resource-arbitration purposes (per the engine's `base` lock); it doesn't
publish any speed/odometry/status telemetry topic, so there's still
nothing for the bridge to subscribe to and that part of §5.2.B remains
unimplemented.


## 4. UI-facing interface (ROS2 server/publisher side)

| Interface | Type | Direction | Meaning |
|---|---|---|---|
| `~/ue_connection` | `std_msgs/Bool` | published | Whether the UE WebSocket is currently connected, at `connection_publish_period` |
| `~/request_user_input` | `RequestUserInput.action` (§3.1.1) | server | Called by whichever caller currently holds `ui_control` -- the Orchestrator's own browse session, or an active engine -- whenever it needs the user to tap a button or a candidate object |
| `~/request_user_input/update` | `UpdateUserInputPanel.srv` (§3.1.1) | server | Called by the same current `ui_control` holder to change what's displayed without ending the open goal |
| `~/ui_control/acquire_control` | `rammp_common_interfaces/AcquireControl` (§3.1.1) | server | Seizes the `ui_control` grant for a new `owner_id` |
| `~/ui_control/release_control` | `rammp_common_interfaces/ReleaseControl` (§3.1.1) | server | Releases the grant, if called by its current holder |
| `~/ui_control/status` | `rammp_common_interfaces/ControlStatus` (§3.1.1) | published (latched) | Current `ui_control` owner/generation/rejection-count |

No other services, actions, or topics advertised to ROS2 clients besides
those listed above; all UI interaction besides them happens over the UE
WebSocket, not ROS2 -- and, as of this revision, the bridge has no
ROS2-facing Orchestrator-specific interface at all (no `select_object`/
`deselect`/`control`/`execute_action`/`get_detected_objects` client calls
remain).

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
current `InteractionPanel`, §5.2.C). UE just echoes back whichever opaque
ID (`instance_id` or `button_id`) the bridge most recently assigned it;
the bridge is the only side that knows what that ID actually means and
resolves it internally.

**Dispatch: resolved purely against whichever `RequestUserInput` goal is
currently active (§3.1.1) -- there is no other dispatch path.** On a
`select_object`/`button_press` command, the bridge checks whether a
`RequestUserInput` goal is currently open *and* the tapped ID matches
something that goal is offering (one of its `objects`' `instance_id`s, or
one of its `panel_json`'s `button_id`s). A matching `select_object` is
consumed as feedback (the goal stays open, §3.1.1 point 3); a matching
`button_press` completes the goal. If no goal is open, or the tapped ID
doesn't match what it's offering, the bridge logs a warning and drops the
command -- nobody is currently offering that tap, since exactly one
caller (the Orchestrator or one executing engine) ever holds `ui_control`
at a time (§3.1.1).

Unknown `cmd` values, and values that don't parse as JSON at all, are
logged and dropped the same way -- never forwarded to anyone -- so a
UI/Blueprint bug is visible during development without crashing the node.

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
  detected-object semantics. **Implemented**: `_on_engine_overlay_color`/
  `_forward_engine_overlay_color` subscribe to the single shared
  `engine_overlay_topic` (§3.3.1) and push every frame through to UE on
  channel 2, `group="overlay"`, `stream_id="engine/overlay"` -- but no
  real engine publishes to it yet, so this has only been exercised with
  synthetic test traffic, not a real wrist camera.

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
**existing detected-objects push** (§5.2.D, fed from §3.1.1's
`objects`/`box` fields), *not* on `InteractionPanel` (§5.2.C), which stays
reserved for the task/button dialog only. **Open/TBD:** the overlay layer's
forwarding code (§3.3.1) is implemented but still unexercised against a real
engine's wrist-camera output, since no real engine publishes to
`engine_overlay_topic` yet; the exact UE-side blur/composite mechanics
(how strong, whether it's a material parameter on the base layer or a
UE post-process) are also still undecided. §3.1.1's `objects[].box`
already covers the detection *semantics* independent of either.


#### B. System status (robot proprioception)

Arm joints, chair/base speed and status, and similar small/structured/
high-frequency telemetry -- pushed via Remote Control preset function
calls (not Gen1's own `setJoints`/`UpdateSystemState` names, see §3.4 for
why: `UpdateArmJointStates` instead), **not** the streaming channel,
since this is small structured data rather than bulk pixel payloads.
Sourced via direct **bridge ↔ hardware node** subscriptions (§3.4), not
through the Orchestrator's generic World State and not through any
engine -- this telemetry is always-on and independent of which engine
(if any) is currently active, unlike §3.3's per-engine camera.
**Implemented: arm joints** (§3.4) -- forwarded by joint name, not a
hardcoded skeleton-bone index array. **Open/TBD:** chair/base speed and
status -- a mock base driver exists (§3.4), but it only consumes
`TwistSetpoint` for resource arbitration and doesn't publish any
speed/odometry/status telemetry, so the concrete topic names/shapes
can't be pinned down until something does.


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
- `progress` (optional): `{phase, pct, message, pausable}`, included at
  the pushing caller's own discretion (via `RequestUserInput`/
  `UpdateUserInputPanel`, §3.1.1); the bridge does not inspect or build
  this field itself, it just forwards whatever `panel_json` the caller
  supplied

**The bridge never builds `InteractionPanel` content itself anymore, nor
does it own any `button_id` mapping.** Earlier revisions of this section
described the bridge constructing panels from `ActionAvailability` and
owning a `button_id -> (Orchestrator call, params)` mapping it resolved
on each tap. That's gone along with the rest of the old Orchestrator-call
surface (§3.1): whichever caller holds `ui_control` (the Orchestrator's
own browse session, or an executing engine) builds its own complete
`panel_json` -- including every `button_id` and what pressing it should
mean to *that caller* -- and the bridge just pushes it to UE verbatim
(`UpdatePanel({"PanelJson": ...})`) and later tells the caller which
`button_id` was pressed (§3.1.1's `RequestUserInput.result`/
`_try_resolve_active_input_goal`). The bridge resolves a `button_press`
only against the currently active goal's own `button_ids` (§5.1) -- it
has no independent notion of what any `button_id` means.

Since only one caller ever holds `ui_control` at a time (§3.1.1), this
channel always reflects exactly one coherent panel: the Orchestrator's
browse-session panel for the current selection whenever nothing is
executing, or the active engine's own panel (confirm dialog, live
progress with Pause/Cancel, result/error) for the whole duration of its
`execute_action` run. The transition between the two is seamless from
UE's point of view -- it just keeps rendering whatever `panel_json` was
last pushed, by whichever goal is currently open.

This is deliberately a superset that can express what an earlier revision
of this doc modeled as four separate functions (`UpdateSelection` i.e.
available actions as buttons, `UpdateProgress`, `UpdateActionResult`, and
`UpdateBridgeError` i.e. an error-styled panel) -- those are superseded by
pushing an appropriately-populated `InteractionPanel` instead. A single
unified, versionable JSON schema is easier for UE to parse generically
(build N buttons from an array) and to extend later (new optional fields)
than a growing set of bespoke RPC function signatures.

**Resolved -- see `docs/USER_INPUT_JSON_SCHEMA.md` for the full schema:**
the three questions originally left open here are now settled by what the
implementation actually does:
- Transport: a Remote Control function call, `UpdatePanel({"PanelJson":
  "..."})` -- not the streaming channel.
- Exact JSON field names/casing: `title`/`message`/`buttons[]`
  (`button_id`/`label`/`style`)/`progress`/`style`, documented in full
  there, including every panel shape the system actually builds today.
- `detected_objects` stays separate (§5.2.D), never folded
  into `InteractionPanel`.
- The name `InteractionPanel` is kept as-is.


#### D. Detected objects (Remote Control preset function)

Separate from `InteractionPanel`, the bridge pushes a raw-data feed over
the same Remote Control preset-function mechanism as §5.2.B (request/
response RPC over the WebSocket, `PUT /remote/preset/<preset>/function/
<FunctionName>`) -- not the streaming channel, since it's a small,
infrequent JSON payload rather than bulk pixel data:

| Function | Parameters | Pushed when |
|---|---|---|
| `UpdateDetectedObjects` | `Objects`: array of `{instance_id, perception_class, box, source}` | The currently active `RequestUserInput` goal's `objects` list changes (§3.1.1's goal open, or any `UpdateUserInputPanel` call carrying an `objects` key) |

Mechanics (`_push_engine_objects` in the reference implementation):

1. There is no polling anymore. Objects reach the bridge purely as part
   of whichever `RequestUserInput` goal is currently open (§3.1.1) --
   either the goal's own initial `request_json.objects`, or a later
   `UpdateUserInputPanel` call's `request_json.objects`. The bridge has
   no independent notion of "the detected objects" outside of what the
   current goal says.
2. Every push sends the **full current list**, not a diff -- UE always
   receives a complete, self-consistent payload rather than incremental
   patches to reconcile.
3. Per-object JSON shape: `instance_id`/`perception_class`/`box`/`source`
   pass through as-is from whatever the caller supplied (`box` empty/
   omitted when the object currently has no known pixel position;
   `source`, `""` when the caller omitted it, meaning "unknown -- don't
   render"). The bridge does **not** fill in or infer `source` itself
   (e.g. from the goal's `engine_id`) -- it's purely caller-supplied per
   object, since a single goal's objects can mix both video channels
   (§3.1.1).
4. `SelectedInstanceId` is sent alongside `Objects` on every push,
   reflecting the goal's current `selected_instance_id` (§3.1.1).

`box`+`source` together are UE's **sole** tap-to-select overlay mechanism
(§3.1.1's box note explains why a mask-based precise hit-test was dropped:
the freshest box may come from an engine's own camera, not a single
shared Perception mask) -- `source` tells UE which of its two video
channels (`scene/color`, the §5.2.A base layer; or `engine/overlay`, the
§5.2.A overlay layer) to draw/hit-test `box` against, since the two
channels are independent video feeds that can have different resolutions/
framings. It's acceptable for `box` to lag slightly behind the live video
between pushes -- fine for a food-prep/kitchen scene where objects don't
move fast -- rather than needing frame-exact sync with whatever's on
screen.

This is the channel any future per-object metadata should be added to --
`InteractionPanel` (§5.2.C) is deliberately not used for this data, since
it's keyed by `instance_id` on its own change-triggered cadence, not tied
to the task/button-dialog lifecycle.

### 5.3 Push semantics

- Detected-objects pushes are **event-triggered by the active goal**, not
  polled -- the bridge pushes the goal's current `objects`/
  `selected_instance_id` whenever a goal opens or an `UpdateUserInputPanel`
  call changes either (§5.2.D). There is no independent poll cadence to
  tune.
- `InteractionPanel` pushes are likewise **event-triggered** -- one push
  whenever the active caller's `panel_json` changes: on a
  `RequestUserInput` goal opening with a non-empty `panel_json`, or on
  each subsequent `UpdateUserInputPanel` call that includes one (e.g.
  after an `"object_tapped"` feedback, or the Orchestrator reclaiming
  `ui_control` and opening a fresh browse-session goal once an engine's
  `execute_action` run ends, §3.1.1).
- Camera stream and system-status pushes are inherently continuous/
  event-driven at whatever rate their ROS2 sources publish, once those
  sources are defined (see §5.2.A/B "Open/TBD").

## 6. Error handling requirements

- A single malformed/invalid UE command must not crash the node or stop
  subsequent commands from being processed (isolate failures per-command).
- A `button_press`/`select_object` referencing a `button_id`/`instance_id`
  the bridge no longer recognizes (no goal currently offering it, or a
  goal that's since moved on) must not crash the node or silently
  misbehave -- the bridge logs a warning and drops the command (§5.1),
  since there is no caller left to report it to.
- Loss of connectivity to any single ROS2 peer (§3.2 Camera, §3.3 Engine
  node, §3.4 Hardware node) must not crash the node or block pushes
  derived from the *other* peers -- each source's absence should degrade
  only its own UE channel (e.g. frozen/blank overlay video, stale system
  status) while the `RequestUserInput`-driven channels (`InteractionPanel`,
  detected objects) keep working normally, since they don't depend on
  any of those three peers.
- The UE WebSocket connection must reconnect automatically on drop
  (including re-registering the preset) without requiring the node to
  restart, and `~/ue_connection` must reflect disconnection promptly.

## 7. Not yet in scope

- **`box`/`source` on candidate objects are now implemented**
  (superseding the old `ObjectRecord`-extension gap this bullet used to
  describe): §3.1.1's `request_json.objects`/`UpdateUserInputPanel`
  carry `box`/`source` directly from whichever caller is active (the
  Orchestrator's own Perception-fed browse session, or an engine's own
  camera), with no separate upstream `rammp_orchestrator_interfaces`
  message change needed -- this superseded an earlier revision's "bridge
  subscribes directly to Perception" design.
- **Perception's start/stop lifecycle during an active task is an
  unconfirmed assumption, not a documented fact.** §3.1.1's design note
  assumes Perception may be paused/stopped while an engine is active (e.g.
  the scene camera's view is obstructed
  by the arm, or to free compute), which is why an active engine's own
  task-interactive candidates are offered via §3.1.1's direct
  `RequestUserInput` mechanism instead, rather than through Perception's
  base-layer feed. But nothing in this workspace currently starts/stops
  Perception's tracking session (`StartTracking.srv`/`StopTracking.srv`)
  -- not the Orchestrator, not this bridge -- so whatever node *does* own
  that decision, and the exact conditions under which it stops
  Perception, still needs to be confirmed before this behavior can be
  relied on.
- **Shared, token-gated overlay-camera topic (§3.1.1, §5.2.A) is now
  implemented**, but unexercised against a real engine --
  `engine_overlay_topic` (default `/engine_overlay/image_raw`,
  `rammp_ui_bridge_interfaces/EngineOverlayImage`) is subscribed to and
  forwarded exactly like the base layer once its token is verified
  against the bridge's `~/ui_control/{acquire_control,
  release_control,status}` arbitration endpoints; no real engine calls
  `acquire_control`/publishes to it yet, so only synthetic test traffic
  (acquire → good-token publish → wrong-token publish → release →
  stale-token publish) has verified the path. The scene camera side
  (§3.2, base layer) is resolved and implemented too: real topics
  subscribed to, frames forwarded to UE over the `ramms_tools` RMSS
  streaming protocol (a pip dependency, see README's Prerequisites
  section). The overlay layer's detected-object semantics (class,
  instance, box) are already covered by §3.1.1's `request_json.objects`
  independent of whether/when a real wrist-camera publishes.
- **§3.1.1's `RequestUserInput`/`UpdateUserInputPanel` are now
  implemented and exercised against a real engine.** Gated by the same
  `~/ui_control` grant as the overlay camera; synthetic test traffic
  (acquire token → goal with two candidate objects → two alternating
  object taps, each producing exactly one feedback message →
  `UpdateUserInputPanel` with a matching and then a mismatched token → a
  `button_press` completing the goal → a second goal rejected on a stale
  token) verified the mechanics first, and `fridge_ui_demo`
  (`rammp_orchestrator`'s `engine_node`) now drives it for a real
  confirm-dialog + held live-progress-panel `execute_action` flow
  end to end (§3.1.1) -- live-verified via `mock_ue`'s HTTP API.
- **Arm joint states for §5.2.B (§3.4 Hardware node) are now
  implemented**, forwarding `arm_joint_states_topic` (default
  `/joint_states`) by joint name (not a hardcoded skeleton-bone index
  array like the Gen1 prototype's `set_ui_joints`), as a Remote Control
  call (`UpdateArmJointStates`). Verified via an in-process live smoke
  test (populated `JointState` forwards with matching names/correctly
  degree-converted positions; an empty one, matching the real shape of
  sheppy's `arm_mock`, forwards as empty arrays without error) and
  against the jetson's live `arm_mock` directly (confirmed running,
  publishing `/joint_states`) -- but the bridge's own copy deployed on
  that jetson predates this session's commits, so full end-to-end
  verification over the live ROS graph (jetson-deployed bridge ↔
  jetson-deployed mock ↔ real UE) is still pending a redeploy. **Chair/
  base speed and status, the other half of §5.2.B, remains unimplemented**
    -- a mock base driver now exists
    (`rammp_orchestrator/ros/mock_base_driver_node.py`), but it only
    consumes `TwistSetpoint` for resource arbitration and publishes no
    speed/odometry/status telemetry, so there's still nothing for the
    bridge to subscribe to.
- Joint-state → UE skeleton **mapping is now name-keyed, not a hardcoded
  index layout.** The Gen1 prototype's `set_ui_joints` hardcoded a
  53-element float array keyed to fixed skeleton-bone indices, specific
  to its one chair+arm robot -- as previously noted here, that doesn't
  carry over. §3.4/§5.2.B's `_forward_arm_joint_states` instead forwards
  whatever `name`/`position` pairs the arm's `JointState` message
  contains, letting UE's own blueprint resolve each name to a bone; the
  bridge needs no per-robot joint layout at all for this to work.
- **Idle/default `InteractionPanel` content is undefined.** The
  `hibernate` note (§3.1.1) says the bridge clears the panel "or replaces
  it with whatever default/idle panel state is defined" once the
  Orchestrator returns to Browse mode -- but no such default panel (what
  title/message/buttons, if any, UE shows when nothing is selected and no
  task is running) has actually been specified yet. Needed before the
  bridge's startup/idle behavior is fully defined, not just its
  task-in-progress behavior.
- **Action button labels** -- still a real gap, now entirely contained
  within the Orchestrator (not the bridge or any interface package): its
  internal `ActionAvailability` dataclass carries only `action_id`, not
  the human-readable `ActionSpec.label` the Orchestrator already has at
  registration time, so the browse-session panel it builds
    (`_build_selected_panel`) currently shows raw `action_id`s as button
  labels instead of real text. Fixing it is a same-repo Orchestrator-core
  change (add a `label` field to `ActionAvailability`, thread it through
  to the button-building code) -- no bridge change and no
  `rammp_orchestrator_interfaces` change needed, since `ActionAvailability`
  was never a ROS message in the first place.

