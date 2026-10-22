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

## What this node does

1. **Connects to UE** over its Remote Control WebSocket API (a preset-based
   RPC + property-change-notification protocol UE exposes natively), and
   registers the configured preset on connect/reconnect.
2. **Translates UE user input into Orchestrator calls.** UE sets a
   `UserInput` preset property to a JSON command (e.g. "user tapped this
   object" / "user tapped this action button" / "user tapped cancel"); the
   bridge parses it and calls the matching Orchestrator service or action:
   `select_object`, `deselect`, `control` (pause/resume/cancel), and
   `execute_action`.
3. **Pushes state back to UE over three channels**: a camera video stream
   (RGBD from Perception and/or RGB from an engine's own camera, each
   optionally carrying detected-object info), system status (arm
   joints/base status), and a single dynamic `InteractionPanel` JSON
   schema (title/message/buttons/progress) that lets UE build whatever
   popup/dialog the current task calls for without hardcoding layouts per
   action. See `docs/NODE_INTERFACE.md` §5.2 for the full schema -- several
   pieces of this are still open design questions, not finalized.
4. **Reports bridge health.** Publishes a connection-status topic so other
   ROS2 nodes/operators can see whether a UE instance is currently attached.

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

This repository currently contains interface/design documentation only; no
implementation has been committed here yet. An earlier Python prototype
exists in a local, uncommitted workspace folder (`rammp-ui-bridge-local`),
built against an earlier draft of `docs/NODE_INTERFACE.md` (pre-dating the
three-channel Bridge→UE design below) -- useful as a reference for the
command/service-call plumbing, but not yet updated for the camera stream/
system status/`InteractionPanel` channels.

## Relationship to the Gen1 prototype GUI bridge

An earlier, hardware-specific GUI bridge (`rammp_prototype_gui.Gui_bridge`)
existed for the Gen1 chair+arm prototype. It streamed camera
color/depth/segmentation-mask frames to UE and mapped real robot joint
states into a UE skeleton. This bridge intends to reuse that same streaming
mechanism (see `docs/NODE_INTERFACE.md` §5.2.A/B), but the ROS2-side source
topics (camera frames, joint/base state) aren't defined yet in
`rammp_orchestrator_interfaces`/`rammp_perception_interfaces` -- see
`docs/NODE_INTERFACE.md`'s "Not yet in scope" section for what's blocked on
that.


