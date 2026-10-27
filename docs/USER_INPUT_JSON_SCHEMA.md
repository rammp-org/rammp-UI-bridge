# `RequestUserInput` / `UpdateUserInputPanel` JSON Schema (`request_json` / `result_json` / `feedback_json`)

**Status: authoritative reference.** This documents the full JSON contract
carried by `rammp_ui_bridge_interfaces`'s `RequestUserInput.action` and
`UpdateUserInputPanel.srv` -- every field inside `request_json`,
`result_json`, and `feedback_json`, including the candidate-object shape
and the currently-selected-instance field -- reconciled with what
`ui_bridge_node.py` (`parse_request_json()`, `build_panel()`,
`panel_button_ids()`, `_push_engine_objects`,
`_try_resolve_active_input_goal`) actually sends and parses. It supersedes/
finalizes the "Open/TBD" items left in `NODE_INTERFACE.md` §5.2.C.

(This file was previously named `INTERACTION_PANEL_SCHEMA.md` and only
covered `panel_json`'s own shape; it has been renamed and expanded to cover
the full `request_json`/`result_json`/`feedback_json` envelope now that
those are the actual wire fields -- `panel_json` is one key inside
`request_json`, not a field of its own.)

All three (`request_json`, `result_json`, `feedback_json`) are JSON
**strings** (not nested messages) -- ROS2 messages don't support
open-ended/variant payloads, and this needs to be easy for UE's Remote
Control API and Blueprint JSON parsing to consume generically without a
schema migration every time a new panel/object shape is needed.

---

## 1. Where each field lives on the wire

| Interface | Field | Direction | Shape |
|---|---|---|---|
| `RequestUserInput.action` | `request_json` | goal (engine -> bridge) | §2 |
| `RequestUserInput.action` | `result_json` | result (bridge -> engine) | §4 |
| `RequestUserInput.action` | `feedback_json` | feedback (bridge -> engine) | §5 |
| `UpdateUserInputPanel.srv` | `request_json` | request (engine -> bridge) | §2 (update semantics) |

`UpdateUserInputPanel` is called any number of times while a
`RequestUserInput` goal is active, to change what's displayed without
completing or restarting the goal (e.g. after an `object_tapped` feedback,
to update the message and highlight the new selection). It has no result
field of its own -- it returns the ordinary `bool accepted, string message`
service-response pair.

**Transport for the `panel_json` value:** a Remote Control function call,
`UpdatePanel({"PanelJson": "<json string>"})` -- not the streaming channel.
Confirmed in `ui_bridge_node.py` (every `build_panel()` call site ends with
`self.ue.call_function("UpdatePanel", {"PanelJson": panel_json})`); panels
are small/sporadic enough that this was never revisited.

**Full-replace semantics:** every push replaces the entire panel/candidate
list/selection it targets -- there is no diffing/patching. The bridge's own
`_panel_buttons` button-id-to-action mapping is likewise fully replaced
(`build_panel()` returns a fresh mapping, never merged with the previous
one).

---

## 2. `request_json` -- goal (`RequestUserInput`) and update (`UpdateUserInputPanel`)

```jsonc
{
  "panel_json": "string",      // optional -- an InteractionPanel-shaped JSON *string* (§6)
  "objects": [ /* §3 */ ],      // optional -- candidate objects the user can tap
  "selected_instance_id": "string"  // optional -- one of objects[].instance_id to highlight
}
```

All three keys are optional at the schema level, but the two call sites
enforce different presence rules:

- **`RequestUserInput.Goal.request_json`:** at least one of `panel_json` /
  `objects` must be non-empty (a goal with neither would show/offer
  nothing). `selected_instance_id` may be set up front to start with an
  object already highlighted (e.g. resuming a prior partial selection).
- **`UpdateUserInputPanel.Request.request_json`:** every key is optional and
  independent -- **key presence, not a separate bool flag, signals "update
  this aspect."** A present key replaces that aspect entirely (panel,
  candidate list, or highlighted selection); an absent key leaves it
  unchanged. Sending `{}` is valid (a no-op update) but pointless.

Bridge behavior while a `RequestUserInput` goal is active:
1. If `request_json`'s `panel_json` is non-empty, push it to UE's
   `InteractionPanel` channel immediately -- this **takes over** whatever
   would otherwise be shown there (e.g. `ExecuteAction` progress),
   reverting to the prior content once the goal ends.
2. If `request_json`'s `objects` is non-empty, push them to UE's
   `UpdateDetectedObjects` channel tagged with the engine's own overlay
   `stream_id` and the current `selected_instance_id`, exactly like any
   other detected-objects push -- tappable the same way base-layer objects
   are.
3. **An object tap does not complete the goal.** The user may tap several
   different candidates in a row while deciding; each tap is reported to
   the engine as an `{"event_type": "object_tapped", ...}` feedback message
   instead (§5), and the bridge's own notion of "currently highlighted"
   updates to match. The engine decides what to do about each tap --
   typically calling `UpdateUserInputPanel` with a `selected_instance_id`
   key to reflect the new choice back to the user -- without the goal
   ending. The goal only completes when UE reports a `button_press`
   matching one of the current panel's `button_id`s (§6.3), at which point
   the engine already knows (from the feedback stream) which object, if
   any, is selected, and the pressed `button_id` tells it what to do with
   that selection.
4. The engine can cancel the goal at any time (e.g. it no longer needs the
   answer); the bridge then withdraws whatever panel/objects it had pushed
   on the engine's behalf and reports `{"status": "cancelled"}` (§4).
5. Only one engine can have a goal active at a time, matching the
   Orchestrator's own one-engine-executing-at-a-time invariant -- a second
   concurrent goal is rejected by the bridge, as is any goal whose `token`
   doesn't match the current `~/ui_control` grant.

---

## 3. Candidate object shape (`objects[]`)

```jsonc
{
  "instance_id": "string",         // opaque, unique within this one goal/stream
  "perception_class": "string",    // e.g. "cup", "food" -- engine-defined, opaque to UE
  "box": [0.0, 0.0, 0.0, 0.0]        // [x_min, y_min, x_max, y_max] in the engine's own
                                   // camera/overlay's pixel space
}
```

This is the minimum UE needs to render and hit-test a tappable box -- it is
**not** `ObjectRecord` (an Orchestrator/world-state concept) and has no
`stream_id` field of its own, since every object in one goal implicitly
belongs to the calling engine's own overlay stream -- the bridge tags them
all with that engine's known `stream_id` when forwarding to UE.

(This shape used to be its own ROS message, `ObjectCandidate.msg` -- retired
now that `objects` travels as plain JSON inside `request_json` instead of a
typed array field.)

---

## 4. `result_json` -- `RequestUserInput` result

```jsonc
{"status": "button", "button_id": "string"}   // user pressed one of panel_json's buttons
```
```jsonc
{"status": "cancelled"}                       // engine cancelled the goal before that happened
```

`status` is the only key guaranteed present; `button_id` is present iff
`status == "button"`.

---

## 5. `feedback_json` -- `RequestUserInput` feedback

Sent once per object tap while the goal is still active, never after
`result_json` is sent. An object tap does NOT complete the goal by itself
(§2, point 3).

```jsonc
{"event_type": "object_tapped", "instance_id": "string"}
```

`event_type` is the only event type implemented today; the key exists so
future event types (e.g. a drag/gesture) can be added without a new
feedback field.

---

## 6. `panel_json`'s own schema (the `InteractionPanel` object)

`panel_json` (wherever it appears -- `request_json["panel_json"]` on
either interface) is itself a JSON string holding this object:

```jsonc
{
  "title": "string",                // panel heading; "" is valid (e.g. idle panel)
  "message": "string",               // free-text status/instruction body; "" is valid
  "buttons": [ /* Button, see §6.3 */ ],   // always present, possibly empty []
  "progress": { /* Progress, see §6.4 */ },  // OPTIONAL -- omitted entirely when not provided
  "style": "normal" | "warning" | "error"  // OPTIONAL panel-wide style -- omitted when "normal" (the default)
}
```

### 6.1 Notes

- `buttons` is **always present** (defaults to `[]`), even for a pure status
  panel with no buttons (e.g. the idle panel, §7.1).
- `progress` is **only present when non-null** -- the key is omitted
  entirely rather than sent as `null`, so UE-side code can use simple key
  existence checks.
- `style` is **only present when not `"normal"`** -- i.e. absence of the key
  means `"normal"`. Today only `"normal"` and `"error"` are actually
  produced by the bridge itself (result panels and error panels); `"warning"`
  is reserved/valid at the schema level (same enum as per-button `style`,
  §6.3) but not yet emitted by any bridge code path. An engine's own
  `panel_json` may use it directly if useful.
- No panel-level `version`/`schema_version` field exists today. Not adding
  one preemptively -- see §8 for how additive-only evolution is handled
  instead.

### 6.2 Who produces each panel

| Panel kind | Producer |
|---|---|
| Orchestrator-driven panels (idle/selection/progress/result/error) | `ui_bridge_node.py`'s `build_panel()`, from Orchestrator service responses |
| Engine-driven panels (via `RequestUserInput`/`UpdateUserInputPanel`) | the active engine itself (already `InteractionPanel`-shaped); the bridge pushes it as-is to UE |

### 6.3 `buttons[]` -- `Button` object

```jsonc
{
  "button_id": "string",   // opaque identifier, see §6.3.1 for ID conventions
  "label": "string",       // text UE renders on the button
  "style": "normal" | "warning" | "error"   // always present (not optional, unlike the panel-level style)
}
```

- `button_id` is **opaque to UE** -- it's a free-form string UE must echo
  back verbatim in a `button_press` event (§5.1 of `NODE_INTERFACE.md`); UE
  attaches no meaning to its contents.
- Unlike the panel-level `style`, the per-button `style` is **always
  included**, even when `"normal"` (see `build_panel()`: every button dict
  always has all three keys).
- There is no `disabled`/`icon`/`subtitle`/similar per-button field today --
  not implemented, not currently needed by any existing panel.

#### 6.3.1 `button_id` conventions (bridge-internal, not UE-visible semantics)

These are the actual `button_id` values the bridge itself generates today.
An engine's own `panel_json` (via `RequestUserInput`) is free to use
whatever `button_id` strings it wants -- they only need to be unique within
that one panel and are resolved against that engine's own goal, never
against this table.

| `button_id` | Meaning | Resolved as |
|---|---|---|
| `action:<engine_id>:<action_id>` | one specific available action on the currently selected object | `execute_action(engine_id, action_id, params)` |
| `deselect` | "back"/deselect the current object | `deselect()` |
| `control:pause` | pause the in-progress action | `control(PAUSE)` |
| `control:resume` | resume a paused action | `control(RESUME)` |
| `control:cancel` | cancel the in-progress action | `control(CANCEL)` |
| `dismiss` | purely client-side acknowledgement of an error panel | no Orchestrator call at all |

The bridge keeps a parallel, JSON-external `button_id -> {kind, ...params}`
mapping (`self._panel_buttons`) to resolve these on tap -- **this mapping is
never part of `panel_json`/sent to UE**, it's rebuilt bridge-side every push.

### 6.4 `progress` object (optional)

Identical shape to `ExecuteAction`'s feedback message, folded into the panel
instead of a separate Remote Control call:

```jsonc
{
  "phase": "string",     // e.g. "approach", "grasp" -- engine-defined, opaque to UE
  "pct": 0.0,            // float, 0.0-100.0 (not 0.0-1.0) -- matches ExecuteAction.Feedback.progress.pct
  "message": "string",   // human-readable status text for this progress tick
  "pausable": true        // bool -- whether a Pause button should currently be offered
}
```

Only ever present while an `execute_action` is in-flight and reporting
progress (`_on_progress`-driven pushes); absent on idle/selection/result/
error panels.

---

## 7. Worked examples

### 7.1 Idle panel (nothing selected)
```json
{"title": "", "message": "Select an object or action to begin.", "buttons": []}
```

### 7.2 Selection panel (object just selected, showing its available actions)
```json
{
  "title": "fridge_1",
  "message": "What would you like to do?",
  "buttons": [
    {"button_id": "action:fridge_ui_demo:open", "label": "open", "style": "normal"},
    {"button_id": "deselect", "label": "Deselect", "style": "normal"}
  ]
}
```

### 7.3 In-progress action panel (with pause/resume/cancel + progress)
```json
{
  "title": "fridge_ui_demo: open",
  "message": "opening door",
  "buttons": [
    {"button_id": "control:pause", "label": "Pause", "style": "normal"},
    {"button_id": "control:resume", "label": "Resume", "style": "normal"},
    {"button_id": "control:cancel", "label": "Cancel", "style": "warning"}
  ],
  "progress": {"phase": "opening", "pct": 40.0, "message": "opening door", "pausable": true}
}
```

### 7.4 Result panel (action completed, success)
```json
{
  "title": "fridge_ui_demo: open -- SUCCEEDED",
  "message": "SUCCEEDED",
  "buttons": [{"button_id": "deselect", "label": "Deselect", "style": "normal"}]
}
```
(`style` key omitted -- `"normal"` is the default.)

### 7.5 Result panel (action completed, failure)
```json
{
  "title": "fridge_ui_demo: open -- FAILED",
  "message": "grasp_failed",
  "buttons": [{"button_id": "deselect", "label": "Deselect", "style": "normal"}],
  "style": "error"
}
```

### 7.6 Error panel (e.g. an Orchestrator service unavailable)
```json
{
  "title": "Error",
  "message": "'/rammp_orchestrator/execute_action' service unavailable",
  "buttons": [{"button_id": "dismiss", "label": "Dismiss", "style": "error"}],
  "style": "error"
}
```

### 7.7 Engine-driven custom panel, panel-only (`RequestUserInput.Goal.request_json`, confirm-before-action)

Shown unescaped, i.e. as Python/JSON values before `json.dumps()` is
applied at each string-field boundary (`request_json` itself, and its
nested `panel_json` string):

```jsonc
// request_json's decoded value:
{
  "panel_json": "{\"message\": \"Open the fridge door?\", \"buttons\": [{\"button_id\": \"yes\", \"label\": \"Yes\", \"style\": \"normal\"}, {\"button_id\": \"no\", \"label\": \"No\", \"style\": \"normal\"}]}"
}

// and panel_json's own decoded value:
{
  "message": "Open the fridge door?",
  "buttons": [
    {"button_id": "yes", "label": "Yes", "style": "normal"},
    {"button_id": "no", "label": "No", "style": "normal"}
  ]
}
```
(`title` omitted entirely here -- the schema treats a missing `title`/
`message` key the same as an empty string; `build_panel()` always includes
both explicitly, but engine-authored JSON isn't required to.)

### 7.8 Engine-driven panel + tappable objects (`RequestUserInput.Goal.request_json`, pick-place UI demo)
```jsonc
// request_json, unescaped for readability:
{
  "panel_json": "{\"message\": \"Tap an item to pick up\", \"buttons\": [{\"button_id\": \"cancel\", \"label\": \"Cancel\", \"style\": \"warning\"}]}",
  "objects": [
    {"instance_id": "cup_1", "perception_class": "drink", "box": [40.0, 80.0, 140.0, 260.0]},
    {"instance_id": "apple_1", "perception_class": "food", "box": [200.0, 100.0, 300.0, 240.0]},
    {"instance_id": "bottle_1", "perception_class": "drink", "box": [360.0, 60.0, 440.0, 280.0]}
  ]
}
```

### 7.9 `feedback_json` -- user taps `apple_1`
```json
{"event_type": "object_tapped", "instance_id": "apple_1"}
```

### 7.10 `UpdateUserInputPanel.Request.request_json` -- reflect that tap back (panel + selection, objects unchanged)
```jsonc
{
  "panel_json": "{\"message\": \"apple_1 selected -- press Confirm to continue.\", \"buttons\": [{\"button_id\": \"confirm\", \"label\": \"Confirm\", \"style\": \"normal\"}, {\"button_id\": \"cancel\", \"label\": \"Cancel\", \"style\": \"warning\"}]}",
  "selected_instance_id": "apple_1"
}
```
(`objects` key omitted entirely -- the candidate list stays as it was;
only `panel_json`/`selected_instance_id` are being replaced here.)

### 7.11 `result_json` -- user presses Confirm
```json
{"status": "button", "button_id": "confirm"}
```

### 7.12 `result_json` -- goal cancelled instead
```json
{"status": "cancelled"}
```

---

## 8. Extensibility / versioning

No schema-version field exists anywhere in `request_json`/`result_json`/
`feedback_json`/`panel_json`. Evolution is additive-only by convention:

- New optional keys may be added to any of these objects (the top-level
  `request_json` envelope, the panel object, the `Button` object, the
  candidate-object shape, or `progress`) at any time -- both the bridge's
  own code and UE's parser must treat unknown/missing keys as "not
  present" (default), never as an error.
- Removing or renaming an existing key is a breaking change and must be
  coordinated with the UE-side Blueprint/C++ parser explicitly (out of
  scope for this document -- that's a UE-repo concern).
- `parse_request_json()` (the bridge-side parser for `request_json`) and
  `panel_button_ids()` (the parser used to detect an engine's own panel's
  button IDs for dispatch-priority matching) both tolerate malformed/
  non-dict JSON by returning `{}` (or an empty set, for
  `panel_button_ids()`) rather than raising -- new top-level keys are
  always safe to add from the engine side without touching either parser.

---

## 9. Open items intentionally NOT resolved here

Carried over from `NODE_INTERFACE.md` §5.2.C, still genuinely open (not
schema questions, so out of scope for this doc):

- Whether `detected_objects` pushes should ever fold into this
  scheme -- **no**, per §5.2.D, they remain a separate `UpdateDetectedObjects`
  call, keyed by its own change-triggered cadence rather than the
  task/button-dialog lifecycle.
- `ActionSpec` has no `label` field yet, so `selection_panel_buttons()`
  falls back to the raw `action_id` as the button label (§7 of
  `NODE_INTERFACE.md`) -- a real UX label field is a future
  `CapabilityManifest` change, not a `panel_json` schema change.
- The overlay-camera / engine-driven panel path (§3.3.2 of
  `NODE_INTERFACE.md`) is implemented and now exercised by a mock engine
  too (`pick_place_ui_demo.py`'s `pick` action sends `request_json` with a
  non-empty `objects` list, §7.8), but still not exercised against a
  **real** engine -- functionally orthogonal to this schema, which is
  already fully exercised via the engine-driven `panel_json`-only case
  (fridge UI demo) and the `panel_json` + `objects` case (pick-place UI
  demo).

---

## 10. Source references

- `rammp-UI-bridge/rammp_ui_bridge/rammp_ui_bridge/ros/ui_bridge_node.py`:
  `parse_request_json()` (request_json parser), `build_panel()` (panel
  builder), `panel_button_ids()` (panel button-id parser),
  `selection_panel_buttons()`/`control_panel_buttons()` (button-list
  builders), `_push_engine_objects` (objects[] -> UE forwarder),
  `_try_resolve_active_input_goal`/`_on_request_user_input_execute`/
  `_on_update_user_input_panel` (goal/feedback/result/update handling),
  `_push_idle_panel`/`_push_selection_panel`/`_push_error_panel`/ the
  progress and result panel call sites (every panel this bridge actually
  produces).
- `rammp-interfaces-ros2/rammp_ui_bridge_interfaces/action/RequestUserInput.action`,
  `rammp-interfaces-ros2/rammp_ui_bridge_interfaces/srv/UpdateUserInputPanel.srv`
  -- the wire fields this document describes.
- `rammp-orchestrator/rammp_orchestrator/rammp_orchestrator/mocks/engines/pick_place_ui_demo.py`
  -- the mock engine §7.8-7.11's example is drawn from (its `_DEMO_CANDIDATES`
  and `pick` script).
- `rammp-UI-bridge/docs/NODE_INTERFACE.md` §5.2.C/§3.3.2 -- original design
  rationale (superset of the four legacy `UpdateSelection`/`UpdateProgress`/
  `UpdateActionResult`/`UpdateBridgeError` functions) and the "Open/TBD"
  list this document resolves.
- `rammp-UI-bridge/docs/ENGINE_UI_INTEGRATION_GUIDE.md` §4 -- shorter
  engine-facing summary; keep that one in sync if this schema changes.
