# Implementation Language: Python vs C++

This node's interface (`docs/NODE_INTERFACE.md`) is defined independently
of implementation language. This doc records the Python-vs-C++ discussion
so the decision (and the reasoning behind it) is visible to future
contributors, rather than re-litigated from scratch.

## TL;DR

For the command/state-dispatch portion of this node's interface (JSON
command dispatch, low-frequency polling, ROS2 service/action calls),
**C++ is unlikely to produce a measurable performance improvement** over
Python/`rclpy` -- the bottleneck in both cases is DDS/network round-trip
time and the UE WebSocket round-trip, not language execution speed. The
clearest case for C++ is the camera video-streaming channel
(`docs/NODE_INTERFACE.md` §5.2.A) -- and unlike when this doc was first
written, that channel is now a **near-term planned feature**, not a
hypothetical future one, which sharpens rather than removes the
performance question below.

## Why language doesn't matter much here, today

- **Workload shape**: `get_world_state`/`get_detected_objects` polls at
  0.5-2 Hz, sparse user-triggered commands, `ExecuteAction` feedback at
  whatever rate an engine's `Progress` messages arrive (typically a few Hz
  during an active action, idle otherwise). None of this is CPU-bound.
- **Where the latency actually comes from**: a ROS2 service call's cost is
  dominated by DDS discovery/transport and the Orchestrator's own
  processing, not by `rclpy` vs `rclcpp` call overhead -- both bind to the
  same underlying `rmw`/DDS layer. Measured service round-trip times are
  typically single-digit milliseconds in either language on the same
  machine; the UE WebSocket hop adds more latency than that.
- **GIL impact**: Python's GIL only becomes a bottleneck when CPU-bound
  work (tight numeric loops, per-pixel processing) competes with I/O on the
  same process. This node currently does neither -- it's exclusively
  I/O-bound (waiting on sockets/DDS), which Python's `asyncio` (for the UE
  WebSocket) and `rclpy`'s executor (for ROS2) both handle via
  non-blocking I/O, not busy-waiting.

## Where C++ would actually help

The camera stream channel (`docs/NODE_INTERFACE.md` §5.2.A: RGBD from
Perception and/or RGB from an engine camera, optionally with detected-
object overlays, reusing the Gen1 prototype's streaming mechanism) is
genuinely CPU/memory-bandwidth-bound work -- 30-60 Hz, large frame
payloads, per-pixel mask/intrinsics math -- where Python's overhead (object
boxing, GIL contention between the streaming thread and ROS2 callbacks) is
a real cost. This is now a planned near-term channel of this bridge (not a
hypothetical future one as earlier drafts of this doc framed it), which
makes it the most concrete argument for using C++ -- specifically for that
channel, not necessarily the whole node.

## Cost side of a full C++ rewrite

- The Gen1 prototype's `UnrealRemoteWebsocket` client (`websockets` +
  `asyncio`) is already working Python code; a C++ equivalent needs a
  websocket client library (e.g. `ixwebsocket`, Boost.Beast) plus a JSON
  library (`nlohmann/json`), and re-implements the same
  connect/reconnect/preset-registration/property-change-parsing state
  machine from scratch.
- `rclcpp` service/action client boilerplate is more verbose than `rclpy`'s
  for the same calls (see `rammp_orchestrator`'s own `mocks/ui_node.py` for
  how compact the Python client side is against this exact interface).
- None of this buys a measurable latency or throughput improvement for the
  current (control-plane-only) scope -- see above.

## Recommendation

Keep control-plane logic (command dispatch, polling, service/action calls,
the `InteractionPanel` schema) in whichever language the team iterates
fastest in -- an earlier Python reference implementation exists in a local
workspace folder (`rammp-ui-bridge-local`), built against an earlier draft
of this interface. Given the camera-streaming channel is now a concrete
near-term requirement rather than a hypothetical, it's worth deciding its
implementation language explicitly and separately from the rest of the
node -- e.g. a mixed-language package, or a dedicated streaming node/process
-- rather than letting "should we use C++" become an all-or-nothing
decision for the whole bridge.
