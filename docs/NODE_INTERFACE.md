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
| `<orchestrator_ns>/get_detected_objects` | `GetDetectedObjects.srv` | Polled; result pushed to UE on instance-set change |
| `<orchestrator_ns>/select_object` | `SelectObject.srv` | Called when UE sends a `select_object` command |
| `<orchestrator_ns>/deselect` | `Deselect.srv` | Called when UE sends a `deselect` command |
| `<orchestrator_ns>/control` | `Control.srv` | Called when UE sends a `control` command (pause/resume/cancel) |
| `<orchestrator_ns>/execute_action` | `ExecuteAction.action` | Called when UE sends an `execute_action` command; feedback and result both relayed to UE |

The bridge never calls `register_engine`, `write_world_state`,
`update_object_state`, or `request_intent` -- those are engine-facing, not
UI-facing (see `rammp_orchestrator_interfaces` package docs).

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
as one JSON object:

| `cmd` | Fields | Orchestrator call triggered |
|---|---|---|
| `select_object` | `instance_id` (string), `perception_class` (string) | `select_object` |
| `deselect` | *(none)* | `deselect` |
| `control` | `command`: one of `"PAUSE"`, `"RESUME"`, `"CANCEL"` | `control` |
| `execute_action` | `engine_id` (string), `action_id` (string), `params` (object, optional) | `execute_action` |

Unknown `cmd` values, and values that don't parse as JSON at all, are
logged and must **not** be forwarded to the Orchestrator; the bridge
reports them back to UE via `UpdateBridgeError` (below) rather than
silently dropping them, so a UI/Blueprint bug is visible during
development.

Examples:
```json
{"cmd": "select_object", "instance_id": "fridge_1", "perception_class": "refrigerator"}
{"cmd": "execute_action", "engine_id": "pick_place", "action_id": "place", "params": {"target": "counter_1"}}
{"cmd": "control", "command": "CANCEL"}
{"cmd": "deselect"}
```

### 5.2 bridge → UE: Remote Control functions called

The bridge calls these as UE preset functions
(`PUT /remote/preset/<preset>/function/<FunctionName>`); UE's preset must
expose a matching function for each one it needs to react to.

| Function | Parameters | Trigger |
|---|---|---|
| `UpdateWorldState` | `Version` (int), `SnapshotJson` (string, JSON-encoded World State) | `get_world_state` poll, only when `Version` changed since the last push |
| `UpdateDetectedObjects` | `Objects`: array of `{instance_id, perception_class, last_pose, location, opened, last_seen}` | `get_detected_objects` poll, only when the set of visible instance IDs changed |
| `UpdateSelection` | `Success` (bool), `Message` (string), `Activations`: array of `{engine_id, availability: [{action_id, available, reason, suggested_params}]}` | response to a `select_object` command |
| `UpdateDeselect` | `Success` (bool), `Message` (string) | response to a `deselect` command |
| `UpdateControlAck` | `Command` (string), `Accepted` (bool), `Message` (string) | response to a `control` command |
| `UpdateProgress` | `EngineId`, `ActionId`, `Phase` (string), `Pct` (float, 0-1), `Message` (string), `Pausable` (bool) | `ExecuteAction` feedback, streamed while an action runs |
| `UpdateActionResult` | `EngineId`, `ActionId`, `Status` (0=SUCCESS/1=FAILED/2=CANCELLED, or absent/null if the goal was rejected before starting), `Payload`, `ErrorCode`, `Recoverable`, `Suggestions` | `ExecuteAction` result, once the goal finishes (or is rejected) |
| `UpdateBridgeError` | `Cmd` (string), `Message` (string) | any `UserInput` command the bridge couldn't service (bad JSON, unknown `cmd`, Orchestrator service/action server unavailable, or a handler exception) |

`WorldState`/`ObjectRecord` JSON shapes are whatever
`rammp_orchestrator.core.world_state`/`interfaces` currently serialize --
intentionally not duplicated here since the bridge passes them through
verbatim; see that package's own docs for the authoritative shape.

### 5.3 Push semantics

- `UpdateWorldState`/`UpdateDetectedObjects` are **change-triggered, not
  every-poll** -- the bridge must track the last-pushed World State
  `version` and the last-pushed set of detected `instance_id`s, and only
  call the UE function when either actually changed. This bounds UE-side
  traffic to actual state changes regardless of poll frequency.
- `UpdateProgress`/`UpdateActionResult` are **event-triggered** -- one push
  per feedback message / one push on terminal result, no polling involved
  (these ride the `ExecuteAction` action's own feedback/result channel).

## 6. Error handling requirements

- If an Orchestrator service/action server is not available within
  `service_wait_timeout`, the bridge must not block indefinitely waiting
  for it -- report `UpdateBridgeError` and return.
- A single malformed/invalid UE command must not crash the node or stop
  subsequent commands from being processed (isolate failures per-command).
- The UE WebSocket connection must reconnect automatically on drop
  (including re-registering the preset) without requiring the node to
  restart, and `~/ue_connection` must reflect disconnection promptly.

## 7. Not yet in scope

Camera/depth/segmentation-mask video streaming and joint-state → UE
skeleton mapping (both present in the Gen1 prototype's
`rammp_prototype_gui.Gui_bridge`) are **not** part of this interface, since
the current Orchestrator/mock-engine system publishes neither camera frames
nor joint states. Add a `docs/NODE_INTERFACE.md` section for these if/when
a perception node or engine starts publishing that data and a real
streaming need exists -- don't speculatively implement them beforehand.
