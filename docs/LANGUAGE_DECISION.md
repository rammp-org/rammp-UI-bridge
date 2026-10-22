# Implementation Language: Python vs C++

This node's interface (`docs/NODE_INTERFACE.md`) is defined independently
of implementation language. This doc records the Python-vs-C++ discussion
so the decision (and the reasoning behind it) is visible to future
contributors, rather than re-litigated from scratch.

## TL;DR

For the interface as currently scoped (sparse JSON command dispatch,
low-frequency polling, ROS2 service/action calls), **C++ is unlikely to
produce a measurable performance improvement** over Python/`rclpy` --
the bottleneck in both cases is DDS/network round-trip time and the UE
WebSocket round-trip, not language execution speed. The clearest case for
C++ is a future high-rate video/data streaming path, which is explicitly
**not** part of this node's current scope (see `NODE_INTERFACE.md` §7).

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

The Gen1 prototype GUI bridge's **camera/depth/segmentation-mask video
streaming** (30-60 Hz, large frame payloads, per-pixel mask/intrinsics
math) is genuinely CPU/memory-bandwidth-bound work where Python's overhead
(object boxing, GIL contention between the streaming thread and ROS2
callbacks) is a real cost. If/when this bridge grows a video streaming
path, that specific piece is the one worth writing in C++ (or offloading to
a GPU-accelerated encode library) -- not the control-plane logic described
in `NODE_INTERFACE.md`.

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

Keep control-plane logic (command dispatch, polling, service/action calls)
in whichever language the team iterates fastest in -- a Python reference
implementation already exists in a local workspace folder
(`rammp-ui-bridge-local`), built directly against this interface spec.
Revisit C++ specifically when/if a high-rate streaming requirement
materializes, and consider scoping it to just that subsystem (a
mixed-language package, or a separate streaming node) rather than rewriting
the whole bridge.
