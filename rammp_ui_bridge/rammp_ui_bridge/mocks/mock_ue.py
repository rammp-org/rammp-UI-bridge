"""Mock UE: a web-page-driven stand-in for Unreal Engine's Remote Control
WebSocket server, for exercising ``ui_bridge_node`` without a real UE
project running.

Protocol notes (mirrors ``unreal_remote_websocket/__init__.py``, the
bridge's own client for this same protocol -- read that file first if this
needs updating):

- ``ui_bridge_node`` CONNECTS OUT to this process (``ws://host:ws_port``,
  default ``127.0.0.1:30020``) -- so this mock is the WebSocket *server*,
  playing UE's role.
- On connect it sends
  ``{"MessageName": "preset.register", "Parameters": {"PresetName": ...}}``
  -- accepted unconditionally here, any preset name is fine.
- Pushes from the bridge arrive as
  ``{"MessageName": "http", "Parameters": {"Url":
  "/remote/preset/<preset>/function/<FunctionName>", "Verb": "PUT", "Body":
  {"parameters": {...}}}}``. Recognized ``FunctionName``\\ s: ``UpdatePanel``,
  ``UpdateDetectedObjects``, ``UpdateWorldState``, ``UpdateArmJointStates``
  -- see ``MockUEServer._on_function_call``. Anything else is still logged
  (not dropped silently) so a newly-added bridge function shows up here.
- "User input" flows the other way as an unsolicited
  ``{"ChangedFields": [{"PropertyLabel": "UserInput", "PropertyValue":
  <json string>}]}`` message -- see ``docs/USER_INPUT_JSON_SCHEMA.md``
  section 5 for the ``cmd`` schema this mock sends:
  ``{"cmd": "select_object", "instance_id": ..., "perception_class": ...}``
  (both fields required -- the Orchestrator's ``select_object`` service
  rejects a mismatched/missing ``perception_class``) /
  ``{"cmd": "button_press", "button_id": ...}``.

Deliberately NOT implemented: the separate RMSS video-streaming TCP channel
(``StreamSender`` in ``ui_bridge_node.py``) -- out of scope for this
control-channel mock; the web page simply doesn't show a camera feed.

Usage::

    python3 -m rammp_ui_bridge.mocks.mock_ue
    # or, once colcon-built:
    ros2 run rammp_ui_bridge mock_ue

Then point a browser at http://127.0.0.1:8000/ (both ports configurable via
--ws-port/--http-port) while ``ui_bridge_node`` is running with its default
``ue_host``/``ue_ws_port`` (``127.0.0.1:30020``) -- no bridge-side parameter
changes needed.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

import websockets

_DEFAULT_WS_PORT = 30020
_DEFAULT_HTTP_PORT = 8000


class _State:
    """Everything the web page needs to render, guarded by a plain lock
    since it's touched from both the asyncio thread (WS messages) and the
    HTTP server's worker threads (button presses, page polling)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.connected = False
        self.panel: dict[str, Any] = {"title": "", "message": "(no panel yet)", "buttons": []}
        self.objects: list[dict[str, Any]] = []
        self.selected_instance_id = ""
        self.world_state: dict[str, Any] = {}
        self.arm_joint_states: dict[str, Any] = {}
        self.log: list[str] = []

    def note(self, line: str) -> None:
        with self.lock:
            self.log.append(line)
            self.log = self.log[-50:]

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "connected": self.connected,
                "panel": self.panel,
                "objects": self.objects,
                "selected_instance_id": self.selected_instance_id,
                "world_state": self.world_state,
                "arm_joint_states": self.arm_joint_states,
                "log": list(self.log),
            }


class MockUEServer:
    """Owns the asyncio event loop and the current bridge connection; the
    HTTP thread schedules sends onto this loop via
    ``run_coroutine_threadsafe`` -- the same pattern
    ``UnrealRemoteWebsocket`` (the real client side of this protocol) uses
    for the opposite direction."""

    def __init__(self, state: _State) -> None:
        self.state = state
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._ws: Any = None

    async def _handler(self, ws) -> None:
        # Single-client assumption (one bridge at a time) matches the real
        # UE <-> bridge relationship; a second connection simply replaces
        # the first as "current".
        self._ws = ws
        self.state.connected = True
        self.state.note("UE client connected")
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except (TypeError, ValueError):
                    self.state.note(f"ignoring non-JSON message: {raw!r}")
                    continue
                self._on_message(msg)
        finally:
            if self._ws is ws:
                self._ws = None
                self.state.connected = False
                self.state.note("UE client disconnected")

    def _on_message(self, msg: dict[str, Any]) -> None:
        name = msg.get("MessageName")
        if name == "preset.register":
            preset = msg.get("Parameters", {}).get("PresetName", "")
            self.state.note(f"preset.register: '{preset}' (accepted)")
            return
        if name != "http":
            self.state.note(f"unrecognized MessageName: {name!r}")
            return
        params = msg.get("Parameters", {})
        url = params.get("Url", "")
        function_name = url.rsplit("/function/", 1)[-1] if "/function/" in url else ""
        body_params = params.get("Body", {}).get("parameters", {})
        self._on_function_call(function_name, body_params)

    def _on_function_call(self, function_name: str, params: dict[str, Any]) -> None:
        with self.state.lock:
            if function_name == "UpdatePanel":
                try:
                    self.state.panel = json.loads(params.get("PanelJson", "{}"))
                except (TypeError, ValueError):
                    self.state.panel = {"title": "", "message": "<malformed panel_json>", "buttons": []}
            elif function_name == "UpdateDetectedObjects":
                self.state.objects = params.get("Objects", [])
                self.state.selected_instance_id = params.get("SelectedInstanceId", "")
            elif function_name == "UpdateWorldState":
                try:
                    self.state.world_state = json.loads(params.get("SnapshotJson", "{}"))
                except (TypeError, ValueError):
                    self.state.world_state = {}
            elif function_name == "UpdateArmJointStates":
                self.state.arm_joint_states = {
                    "names": params.get("Names", []),
                    "positions_deg": params.get("PositionsDeg", []),
                }
            # else: unrecognized function -- still logged below so a new
            # bridge-side call_function() shows up here instead of vanishing.
        self.state.note(f"{function_name}({json.dumps(params)[:200]})")

    def send_user_input(self, command: dict[str, Any]) -> bool:
        """Sends a ``UserInput`` ``ChangedFields`` event to whatever is
        currently connected (the exact shape ``parse_value_changes`` in
        ``UnrealRemoteWebsocket`` expects). Returns False (and logs,
        instead of raising) if nothing is connected -- a browser button
        press with no bridge attached should fail soft."""
        ws = self._ws
        if ws is None or self.loop is None:
            self.state.note(f"dropped user input (not connected): {command}")
            return False
        payload = {
            "ChangedFields": [
                {"PropertyLabel": "UserInput", "PropertyValue": json.dumps(command)}
            ]
        }
        asyncio.run_coroutine_threadsafe(ws.send(json.dumps(payload)), self.loop)
        self.state.note(f"sent user input: {command}")
        return True

    async def serve_forever(self, host: str, port: int) -> None:
        self.loop = asyncio.get_running_loop()
        async with websockets.serve(self._handler, host, port):
            self.state.note(f"mock UE listening on ws://{host}:{port}")
            await asyncio.Future()  # run until cancelled


_PAGE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Mock UE</title>
<style>
  body { font-family: sans-serif; max-width: 700px; margin: 2rem auto; }
  #status { padding: 0.5rem; border-radius: 4px; margin-bottom: 1rem; }
  .connected { background: #d4f7d4; }
  .disconnected { background: #f7d4d4; }
  button { margin: 0.25rem; padding: 0.5rem 1rem; cursor: pointer; }
  button.warning { background: #ffe9a8; }
  button.error { background: #ffb3b3; }
  #objects button { background: #dbe9ff; }
  #objects button.selected { outline: 2px solid #2a6df4; }
  #progress-wrap { display: none; margin: 0.5rem 0; }
  #progress-bar { width: 100%; height: 1.2rem; }
  #progress-label { font-size: 0.8rem; color: #555; }
  #log { font-family: monospace; font-size: 0.8rem; background: #111; color: #0f0;
         padding: 0.5rem; height: 160px; overflow-y: auto; white-space: pre-wrap; }
</style>
</head>
<body>
  <h2>Mock UE -- rammp-UI-bridge control page</h2>
  <div id="status">connecting...</div>
  <h3 id="panel-title"></h3>
  <p id="panel-message"></p>
  <div id="progress-wrap">
    <progress id="progress-bar" max="1" value="0"></progress>
    <div id="progress-label"></div>
  </div>
  <div id="buttons"></div>
  <h3>Detected objects</h3>
  <div id="objects"></div>
  <h3>Log</h3>
  <div id="log"></div>

<script>
async function poll() {
  try {
    const r = await fetch('/api/state');
    const s = await r.json();
    const status = document.getElementById('status');
    status.textContent = s.connected ? 'ui_bridge_node: CONNECTED' : 'ui_bridge_node: not connected';
    status.className = s.connected ? 'connected' : 'disconnected';

    document.getElementById('panel-title').textContent = s.panel.title || '';
    document.getElementById('panel-message').textContent = s.panel.message || '';

    const progressWrap = document.getElementById('progress-wrap');
    const p = s.panel.progress;
    if (p) {
      progressWrap.style.display = 'block';
      // p.pct is a 0.0-1.0 fraction (matches UE's native ProgressBar.Percent).
      document.getElementById('progress-bar').value = p.pct;
      document.getElementById('progress-label').textContent =
        p.phase + ': ' + Math.round(p.pct * 100) + '% -- ' + (p.message || '');
    } else {
      progressWrap.style.display = 'none';
    }

    const buttonsDiv = document.getElementById('buttons');
    buttonsDiv.innerHTML = '';
    (s.panel.buttons || []).forEach(b => {
      const el = document.createElement('button');
      el.textContent = b.label;
      el.className = b.style || 'normal';
      el.onclick = () => post('/api/button_press', {button_id: b.button_id});
      buttonsDiv.appendChild(el);
    });

    const objectsDiv = document.getElementById('objects');
    objectsDiv.innerHTML = '';
    (s.objects || []).forEach(o => {
      const el = document.createElement('button');
      el.textContent = o.instance_id + ' (' + o.perception_class + ')';
      if (o.instance_id === s.selected_instance_id) el.classList.add('selected');
      el.onclick = () => post('/api/select_object', {instance_id: o.instance_id, perception_class: o.perception_class});
      objectsDiv.appendChild(el);
    });

    document.getElementById('log').textContent = (s.log || []).join('\\n');
  } catch (e) {
    console.error(e);
  }
}
async function post(url, body) {
  await fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
}
setInterval(poll, 500);
poll();
</script>
</body>
</html>
"""


class _Handler(BaseHTTPRequestHandler):
    server: "_HTTPServerWithMock"

    def log_message(self, fmt: str, *args: Any) -> None:
        pass  # silence default stderr access logging -- state.log is the log that matters here

    def do_GET(self) -> None:  # noqa: N802 (BaseHTTPRequestHandler naming)
        if self.path in ("/", ""):
            self._send_html(_PAGE)
        elif self.path == "/api/state":
            self._send_json(self.server.mock.state.snapshot())
        else:
            self.send_error(404)

    def do_POST(self) -> None:  # noqa: N802
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw or b"{}")
        except (TypeError, ValueError):
            self.send_error(400, "malformed JSON body")
            return
        if self.path == "/api/select_object":
            self.server.mock.send_user_input(
                {
                    "cmd": "select_object",
                    "instance_id": body.get("instance_id", ""),
                    "perception_class": body.get("perception_class", ""),
                }
            )
            self._send_json({"ok": True})
        elif self.path == "/api/button_press":
            self.server.mock.send_user_input(
                {"cmd": "button_press", "button_id": body.get("button_id", "")}
            )
            self._send_json({"ok": True})
        else:
            self.send_error(404)

    def _send_html(self, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, data: Any) -> None:
        body = json.dumps(data).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class _HTTPServerWithMock(ThreadingHTTPServer):
    def __init__(self, addr: tuple[str, int], handler_cls: type, mock: MockUEServer) -> None:
        super().__init__(addr, handler_cls)
        self.mock = mock


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="bind address for both servers")
    parser.add_argument("--ws-port", type=int, default=_DEFAULT_WS_PORT,
                         help="WebSocket port the bridge connects to (default: bridge's ue_ws_port default)")
    parser.add_argument("--http-port", type=int, default=_DEFAULT_HTTP_PORT,
                         help="HTTP port for the control web page")
    args = parser.parse_args()

    state = _State()
    mock = MockUEServer(state)

    http_server = _HTTPServerWithMock((args.host, args.http_port), _Handler, mock)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()
    print(f"mock UE control page:  http://{args.host}:{args.http_port}/")
    print(f"mock UE websocket:     ws://{args.host}:{args.ws_port}  (point ui_bridge_node's ue_host/ue_ws_port here)")

    try:
        asyncio.run(mock.serve_forever(args.host, args.ws_port))
    except KeyboardInterrupt:
        pass
    finally:
        http_server.shutdown()


if __name__ == "__main__":
    main()
