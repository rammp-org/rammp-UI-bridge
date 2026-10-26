# `InteractionPanel` JSON Schema (`panel_json`)

**Status: authoritative reference.** This supersedes/finalizes the "Open/TBD"
items left in `NODE_INTERFACE.md` §5.2.C (transport, exact field names,
naming) by documenting exactly what the current implementation
(`ui_bridge_node.py`'s `build_panel()`/`panel_button_ids()`) actually sends
and parses, reconciled with the two places `panel_json` appears as a wire
field: `rammp_ui_bridge_interfaces/action/RequestUserInput.goal.panel_json`
and `rammp_ui_bridge_interfaces/srv/UpdateUserInputPanel.request.panel_json`.

`panel_json` is always a JSON **string** (not a nested message) -- ROS2
messages don't support open-ended/variant payloads, and this needs to be
easy for UE's Remote Control API and Blueprint JSON parsing to consume
generically (build N buttons from an array) without a schema migration
every time a new panel shape is needed.

---

## 1. Who produces/consumes it

| Direction | Field | Producer | Consumer |
|---|---|---|---|
| Orchestrator-driven panels | n/a (bridge builds these itself from Orchestrator service responses) | `ui_bridge_node.py`'s `build_panel()` | pushed to UE via `UpdatePanel({"PanelJson": ...})` |
| Engine-driven panels | `RequestUserInput.Goal.panel_json` | the active engine (already `InteractionPanel`-shaped) | bridge pushes as-is to UE, resolves `button_press` against the goal instead of the normal Orchestrator-call mapping |
| Engine-driven panel update | `UpdateUserInputPanel.Request.panel_json` | the active engine, mid-goal | bridge pushes as-is to UE, iff `request.update_panel` is true |

**Transport (resolved):** a Remote Control function call,
`UpdatePanel({"PanelJson": "<json string>"})` -- not the streaming channel.
Confirmed in `ui_bridge_node.py` (every `build_panel()` call site ends with
`self.ue.call_function("UpdatePanel", {"PanelJson": panel_json})`); panels
are small/sporadic enough that this was never revisited.

**Full-replace semantics:** every push replaces the entire panel -- there is
no diffing/patching. The bridge's own `_panel_buttons` button-id-to-action
mapping is likewise fully replaced (`build_panel()` returns a fresh mapping,
never merged with the previous one).

---

## 2. Top-level schema

```jsonc
{
  "title": "string",                // panel heading; "" is valid (e.g. idle panel)
  "message": "string",               // free-text status/instruction body; "" is valid
  "buttons": [ /* Button, see §3 */ ],   // always present, possibly empty []
  "progress": { /* Progress, see §4 */ },  // OPTIONAL -- omitted entirely when not provided
  "style": "normal" | "warning" | "error"  // OPTIONAL panel-wide style -- omitted when "normal" (the default)
}
```

Notes:
- `buttons` is **always present** (defaults to `[]`), even for a pure status
  panel with no buttons (e.g. the idle panel, §5.1).
- `progress` is **only present when non-null** -- the key is omitted
  entirely rather than sent as `null`, so UE-side code can use simple key
  existence checks.
- `style` is **only present when not `"normal"`** -- i.e. absence of the key
  means `"normal"`. Today only `"normal"` and `"error"` are actually
  produced by the bridge itself (result panels and error panels); `"warning"`
  is reserved/valid at the schema level (same enum as per-button `style`,
  §3) but not yet emitted by any bridge code path. An engine's own
  `panel_json` may use it directly if useful.
- No panel-level `version`/`schema_version` field exists today. Not adding
  one preemptively -- see §6 for how additive-only evolution is handled
  instead.

---

## 3. `buttons[]` -- `Button` object

```jsonc
{
  "button_id": "string",   // opaque identifier, see §3.1 for ID conventions
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

### 3.1 `button_id` conventions (bridge-internal, not UE-visible semantics)

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

---

## 4. `progress` object (optional)

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

## 5. Worked examples (every panel the bridge builds today)

### 5.1 Idle panel (nothing selected)
```json
{"title": "", "message": "Select an object or action to begin.", "buttons": []}
```

### 5.2 Selection panel (object just selected, showing its available actions)
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

### 5.3 In-progress action panel (with pause/resume/cancel + progress)
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

### 5.4 Result panel (action completed, success)
```json
{
  "title": "fridge_ui_demo: open -- SUCCEEDED",
  "message": "SUCCEEDED",
  "buttons": [{"button_id": "deselect", "label": "Deselect", "style": "normal"}]
}
```
(`style` key omitted -- `"normal"` is the default.)

### 5.5 Result panel (action completed, failure)
```json
{
  "title": "fridge_ui_demo: open -- FAILED",
  "message": "grasp_failed",
  "buttons": [{"button_id": "deselect", "label": "Deselect", "style": "normal"}],
  "style": "error"
}
```

### 5.6 Error panel (e.g. an Orchestrator service unavailable)
```json
{
  "title": "Error",
  "message": "'/rammp_orchestrator/execute_action' service unavailable",
  "buttons": [{"button_id": "dismiss", "label": "Dismiss", "style": "error"}],
  "style": "error"
}
```

### 5.7 Engine-driven custom panel (via `RequestUserInput.panel_json`, confirm-before-action)
```json
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

---

## 6. Extensibility / versioning

No schema-version field exists. Evolution is additive-only by convention:

- New optional keys may be added to the panel object, the `Button` object,
  or `progress` at any time -- both the bridge's own builder and UE's parser
  must treat unknown/missing keys as "not present" (default), never as an
  error.
- Removing or renaming an existing key is a breaking change and must be
  coordinated with the UE-side Blueprint/C++ parser explicitly (out of
  scope for this document -- that's a UE-repo concern).
- `panel_button_ids()` (the bridge-side parser used to detect an engine's
  own panel's button IDs for dispatch-priority matching) only ever reads
  `buttons[].button_id` and tolerates any other shape/garbage JSON by
  returning an empty set rather than raising -- new top-level keys are
  always safe to add from the engine side without touching this parser.

---

## 7. Open items intentionally NOT resolved here

Carried over from `NODE_INTERFACE.md` §5.2.C, still genuinely open (not
panel-*schema* questions, so out of scope for this doc):

- Whether `world_state`/`detected_objects` pushes should ever fold into this
  scheme -- **no**, per §5.2.D, they remain separate `UpdateWorldState`/
  `UpdateDetectedObjects` calls, keyed by their own change-triggered cadence
  rather than the task/button-dialog lifecycle.
- `ActionSpec` has no `label` field yet, so `selection_panel_buttons()`
  falls back to the raw `action_id` as the button label (§7 of
  `NODE_INTERFACE.md`) -- a real UX label field is a future `CapabilityManifest`
  change, not a `panel_json` schema change.
- The overlay-camera / engine-driven panel path (§3.3.2) is implemented but
  has not been exercised against a real engine yet (no mock engine
  currently calls `RequestUserInput` with a non-empty `objects` list) --
  functionally orthogonal to this schema, which is already fully exercised
  via the engine-driven `panel_json`-only case (fridge/pick-place UI demos).

---

## 8. Source references

- `rammp-UI-bridge/rammp_ui_bridge/rammp_ui_bridge/ros/ui_bridge_node.py`:
  `build_panel()` (builder), `panel_button_ids()` (parser),
  `selection_panel_buttons()`/`control_panel_buttons()` (button-list
  builders), `_push_idle_panel`/`_push_selection_panel`/`_push_error_panel`/
  the progress and result panel call sites (every panel this bridge
  actually produces).
- `rammp-interfaces-ros2/rammp_ui_bridge_interfaces/action/RequestUserInput.action`,
  `rammp-interfaces-ros2/rammp_ui_bridge_interfaces/srv/UpdateUserInputPanel.srv`
  -- the two wire fields named `panel_json`.
- `rammp-UI-bridge/docs/NODE_INTERFACE.md` §5.2.C -- original design
  rationale (superset of the four legacy `UpdateSelection`/`UpdateProgress`/
  `UpdateActionResult`/`UpdateBridgeError` functions) and the "Open/TBD"
  list this document resolves.
- `rammp-UI-bridge/docs/ENGINE_UI_INTEGRATION_GUIDE.md` §4 -- shorter
  engine-facing summary; keep that one in sync if this schema changes.
