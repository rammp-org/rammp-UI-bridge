# Engine ↔ UI Bridge Integration Guide

How an **engine** (capability provider, in the `rammp_orchestrator` sense --
see `rammp-orchestrator/rammp_orchestrator/docs/ENGINE_DEVELOPER_GUIDE.md`)
gets input from the user, in both of the two situations that come up
constantly:

1. **"I can do action A, B, or C -- how do I let the user pick one?"**
   (§1 below) -- this is the Orchestrator's job, not the UI bridge's; your
   engine doesn't talk to the bridge at all for this.
2. **"I'm already running action A, and A needs the user to pick a *target*
   (e.g. which of three detected cups) -- how do I get that input?"** (§2
   below) -- this is the UI bridge's `RequestUserInput` action, called
   **directly by your engine, bypassing the Orchestrator entirely**. This is
   the design this doc mostly exists to explain, per the project's
   decision: *"the engine talk to the ui bridge directly when it need an
   user input ... the orchestrator does not need to know any of this or in
   the middle."*

Read `docs/NODE_INTERFACE.md` §3.3.2/§5.2.C for the bridge-side spec this
guide is a task-oriented front end for; this doc won't repeat every field,
just how to use them.

---

## 0. The two channels, at a glance

| | Who decides | Who's involved | Interface |
|---|---|---|---|
| **Picking which action to run** (§1) | The Orchestrator, from your `CapabilityManifest`/`~/activate` answer | Orchestrator + bridge; your engine just answers `~/activate` truthfully | `rammp_orchestrator_interfaces` (`Activate.srv`, `ActionAvailability`) |
| **Picking a target/parameter *within* a running action** (§2) | Your engine, mid-`execute()` | Your engine + bridge directly; **Orchestrator is not involved** | `rammp_ui_bridge_interfaces` (`RequestUserInput.action`, `UpdateUserInputPanel.srv`) + `rammp_common_interfaces` (`AcquireControl`/`ReleaseControl`) |

Never use `RequestUserInput` to let the user choose *which action to run* --
that's already the Orchestrator's `select_object` → `ActionAvailability` →
`execute_action` flow, and duplicating it here would race with the
Orchestrator's own panel. Use `RequestUserInput` only for input your engine
needs *after* it's already been picked and is executing.

---

## 1. "I can do action A, B, C -- how do I get the user's choice?"

You don't call the bridge for this at all. Declare all three actions in your
`CapabilityManifest`, and the Orchestrator + bridge render and dispatch the
choice for you:

```python
CapabilityManifest(
    engine_id="drink_dispenser",
    display_name="Drink Dispenser",
    perception_classes=["cup"],
    actions=[
        ActionSpec(action_id="fill_water", label="Fill Water"),
        ActionSpec(action_id="fill_juice", label="Fill Juice"),
        ActionSpec(action_id="fill_soda",  label="Fill Soda"),
    ],
    ...
)
```

What happens next, end to end:

1. User taps a `cup` object on screen. The bridge calls `select_object` on
   the Orchestrator, which calls your `~/activate` (`Activate.srv`).
2. You return one `ActionAvailability` per action you registered
   (`available=True/False` + `reason`, based on current World State -- e.g.
   `fill_soda` unavailable if the soda line is empty).
3. The bridge pushes an `InteractionPanel` (`docs/NODE_INTERFACE.md` §5.2.C)
   with one button per *available* action -- this is the bridge's own
   `selection_panel_buttons()` logic, not something you write.
4. User taps "Fill Juice". The bridge resolves that button to an
   `execute_action(engine_id="drink_dispenser", action_id="fill_juice", ...)`
   call on the Orchestrator, which calls your `~/execute_action`.
5. Your `execute()` generator runs. **This is where §2 below starts**, if
   `fill_juice` itself still needs to ask "which cup size?" or "which of
   the detected cups did you mean?".

Known gap (tracked in `docs/NODE_INTERFACE.md` §7): `ActionAvailability`
doesn't carry `ActionSpec.label` back, so today's buttons fall back to the
raw `action_id` as their label text. Use a human-readable `action_id`
(`fill_juice`, not `act_3`) until that's fixed upstream.

---

## 2. "Action A needs the user to pick a target -- how do I get that input?"

This is `rammp_ui_bridge_interfaces/action/RequestUserInput`, called by your
engine's own node, **from inside your `execute()` generator** while action A
is still running. The Orchestrator never sees this exchange.

### 2.0 Prerequisite: your node must be able to block without deadlocking

`execute()` is a synchronous generator (see `ENGINE_DEVELOPER_GUIDE.md` §5).
Waiting on a `RequestUserInput` goal to complete is a blocking wait on your
own node's executor. **Use a `MultiThreadedExecutor` with a
`ReentrantCallbackGroup`** for your engine node (same pattern the bridge
itself uses), so your node can still process the goal's feedback/result
callbacks while `execute()` is blocked waiting for them. A
`SingleThreadedExecutor` will deadlock here.

### 2.1 Acquire the `ui_control` token

Both `RequestUserInput` and the overlay-camera topic (`EngineOverlayImage`,
`docs/NODE_INTERFACE.md` §3.3.1) share one arbitration resource,
`~/ui_control` on the bridge node (full name: `/ui_bridge_node/ui_control/*`
at defaults). The bridge still mints and owns the token, but
`AcquireControl.srv`'s own docstring states this service is **"called by
the task orchestrator and nothing else"** -- so don't call the bridge
directly. Instead call the Orchestrator's proxy services
(`~/acquire_ui_control`/`~/release_ui_control`, default node name
`rammp_orchestrator`), which relay to the bridge on your behalf. This keeps
the Orchestrator as the single node that links an activated engine to every
resource it's granted (arm/gripper/base locks work the same way), even
though it never sees the `RequestUserInput` goal/feedback content itself.

Acquire it **before** sending a `RequestUserInput` goal, even if you're not
publishing any overlay frames this time -- the bridge checks the token on
every goal regardless:

```python
from rammp_common_interfaces.srv import AcquireControl, ReleaseControl

client = node.create_client(AcquireControl, "/rammp_orchestrator/acquire_ui_control")
req = AcquireControl.Request(owner_id="drink_dispenser")
resp = client.call(req)          # resp.accepted, resp.token (uint8[16]), resp.generation
token = resp.token                # keep this -- stamp every subsequent call with it
```

This call **seizes**: if another engine (or a stale goal) currently holds
`ui_control`, you get it anyway and their outstanding goal/overlay frames
are invalidated. Only acquire it when you're actually about to ask for
input -- don't hold it for your whole action. Release it the same way, via
`/rammp_orchestrator/release_ui_control` (`ReleaseControl.srv`), not the
bridge's own release service.

### 2.2 Decide: are the targets tappable objects, plain buttons, or both?

**Case A -- abstract options (no camera, just a choice)**, e.g. "which cup
size: Small / Medium / Large":

```python
panel_json = json.dumps({
    "title": "Choose a size",
    "message": "Which size cup?",
    "buttons": [
        {"button_id": "small",  "label": "Small",  "style": "normal"},
        {"button_id": "medium", "label": "Medium", "style": "normal"},
        {"button_id": "large",  "label": "Large",  "style": "normal"},
    ],
})
goal = RequestUserInput.Goal(
    engine_id="drink_dispenser", token=token,
    panel_json=panel_json, objects=[], selected_instance_id="",
)
```

The goal completes as soon as the user presses one of these buttons --
`result.button_id` (`"small"`/`"medium"`/`"large"`) directly tells you which
target was chosen. No feedback loop needed for this case.

**Case B -- tappable objects on your own camera stream**, e.g. "which of
these 3 detected cups": build one `ObjectCandidate` per cup (its
`instance_id`/`perception_class`/`box` in *your own camera's* pixel space --
typically what your perception step already produced), and let the user
tap directly on them:

```python
objects = [
    ObjectCandidate(instance_id="cup_1", perception_class="cup", box=[10, 20, 90, 180]),
    ObjectCandidate(instance_id="cup_2", perception_class="cup", box=[120, 30, 210, 190]),
    ObjectCandidate(instance_id="cup_3", perception_class="cup", box=[230, 15, 320, 175]),
]
panel_json = json.dumps({
    "title": "Pick a cup",
    "message": "Tap the cup you want, then press Confirm.",
    "buttons": [{"button_id": "confirm", "label": "Confirm", "style": "normal"},
                {"button_id": "cancel",  "label": "Cancel",  "style": "warning"}],
})
goal = RequestUserInput.Goal(
    engine_id="drink_dispenser", token=token,
    panel_json=panel_json, objects=objects, selected_instance_id="",
)
```

If you also publish your own overlay video (so the user sees a live camera
feed with these boxes drawn on it, not just a static frame), publish
`EngineOverlayImage` (`image`, same `token`) to `engine_overlay_topic`
(default `/engine_overlay/image_raw`) for as long as this goal is active.
This is optional -- `RequestUserInput`'s `objects` field works against
whatever's currently on screen (the base scene camera or your overlay),
UE just needs *a* box to hit-test against.

Send the goal (async, since you'll be reacting to feedback while it runs):

```python
action_client = ActionClient(node, RequestUserInput, "/ui_bridge_node/request_user_input")
send_future = action_client.send_goal_async(goal, feedback_callback=on_feedback)
```

### 2.3 React to taps (Case B only) via feedback + `~/request_user_input/update`

Every tap while the goal is active arrives as feedback
(`event_type=EVENT_OBJECT_TAPPED`, `instance_id=...`) -- it does **not**
complete the goal by itself; the user may tap several cups before deciding.
On each tap, update your own tracked "currently selected" state and tell the
bridge to reflect it (highlight + message change), via
`UpdateUserInputPanel.srv`:

```python
def on_feedback(feedback_msg):
    fb = feedback_msg.feedback
    if fb.event_type == RequestUserInput.Feedback.EVENT_OBJECT_TAPPED:
        selected = fb.instance_id
        update_client.call(UpdateUserInputPanel.Request(
            token=token,
            update_panel=True,
            panel_json=json.dumps({
                "title": "Pick a cup",
                "message": f"{selected} selected -- press Confirm to continue.",
                "buttons": [{"button_id": "confirm", "label": "Confirm", "style": "normal"},
                            {"button_id": "cancel",  "label": "Cancel",  "style": "warning"}],
            }),
            update_objects=False, objects=[],
            update_selected=True, selected_instance_id=selected,
        ))
```

`update_objects=True` also lets you replace the candidate list entirely
mid-goal (e.g. if your own perception re-detects and the boxes moved) --
set only the `update_*` flags for what actually changed; unset fields in
the request are ignored.

### 2.4 Resolve the final choice

The goal completes with `result.status == STATUS_BUTTON` when the user
presses a button (`result.button_id` is set), or `STATUS_CANCELLED` if you
cancel the goal yourself (e.g. the rest of action A timed out). For Case B,
by the time a button is pressed you already know the target from your own
tracked "currently selected" state (updated in §2.3) -- `button_id` tells
you *what to do* with it (`"confirm"` vs `"cancel"`), while the selection
itself came from the feedback stream, not from `result`.

```python
result = await send_future  # or block via .result() off the executor thread
if result.status == RequestUserInput.Result.STATUS_BUTTON:
    if result.button_id == "confirm" and selected:
        proceed_with_target(selected)
    elif result.button_id == "cancel":
        abort_step()
else:  # STATUS_CANCELLED
    abort_step()
```

### 2.5 Release control

Once you're done with this input step, release the token so another engine
(or the base scene camera) can use `ui_control` again:

```python
release_client.call(ReleaseControl.Request(token=token))  # against /rammp_orchestrator/release_ui_control
```

The bridge automatically reverts the UE panel to whatever the
Orchestrator-driven state (selected object's action buttons, or idle) calls
for once your goal ends -- you don't need to push anything to "clean up"
the panel yourself.

---

## 3. Combining both

A single action can use §1 once (to get picked at all) and then §2 any
number of times while it runs -- including more than once in sequence, e.g.
"pick a cup" (§2, Case B) then "pick a fill level" (§2, Case A again) before
finishing. Each `RequestUserInput` goal is independent; acquire/release
`ui_control` around each one (or keep it held across a short back-to-back
sequence -- just don't hold it indefinitely, since it blocks every other
engine's overlay/input requests while held).

---

## 4. Reference: message/service shapes used above

All in `rammp-interfaces-ros2`:

- `rammp_ui_bridge_interfaces/action/RequestUserInput` -- goal
  (`engine_id`, `token`, `panel_json`, `objects[]`, `selected_instance_id`),
  feedback (`event_type`, `instance_id`), result (`status`, `button_id`).
- `rammp_ui_bridge_interfaces/srv/UpdateUserInputPanel` -- request
  (`token`, `update_panel`/`panel_json`, `update_objects`/`objects[]`,
  `update_selected`/`selected_instance_id`), response (`accepted`,
  `message`).
- `rammp_ui_bridge_interfaces/msg/ObjectCandidate` -- `instance_id`,
  `perception_class`, `box` (`[x_min, y_min, x_max, y_max]` in your
  camera's own pixel space).
- `rammp_ui_bridge_interfaces/msg/EngineOverlayImage` -- `image`
  (`sensor_msgs/Image`), `token`.
- `rammp_common_interfaces/srv/{AcquireControl,ReleaseControl}` and
  `msg/ControlStatus` -- the shared token-arbitration mechanism also used
  by `rammp_arm_interfaces/JointSetpoint` and
  `rammp_base_interfaces/TwistSetpoint`.
- `InteractionPanel` JSON shape (`panel_json` string content):
  `{"title": str, "message": str, "buttons": [{"button_id": str, "label": str, "style": "normal"|"warning"|"error"}], "progress": {...}}`
  (`docs/NODE_INTERFACE.md` §5.2.C).

See also: `docs/NODE_INTERFACE.md` §3.3.2/§5.2.C (bridge-side spec this
guide is based on), `rammp-orchestrator/rammp_orchestrator/docs/
ENGINE_DEVELOPER_GUIDE.md` (the Orchestrator-side engine contract §1 above
depends on).
