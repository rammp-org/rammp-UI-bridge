"""Standalone live-test tool for ``ui_bridge_node``'s ``~/request_user_input``
action, exercising it exactly the way a real engine would (see
``docs/ENGINE_UI_INTEGRATION_GUIDE.md`` section 2) but without needing a real
engine, the Orchestrator, or ``rammp_orchestrator`` on the path at all -- only
``rammp_common_interfaces``/``rammp_ui_bridge_interfaces`` are required.

Sends one ``RequestUserInput`` goal whose panel has a title, a message, and
three buttons -- one of each documented style (``normal``/``warning``/
``error``, see ``docs/USER_INPUT_JSON_SCHEMA.md`` section 6.3) -- so a single
glance at the real UE touchscreen confirms all three render distinctly.
Blocks until the user presses one (or the goal is cancelled/aborted/times
out), logs the result, then repeats with the *panel-level* ``style`` field
(section 6.1) cycled to the next of ``normal``/``warning``/``error`` in turn
-- so repeated runs also exercise panel-wide styling, not just per-button.

Acquires ``ui_control`` directly against the bridge's own
``~/ui_control/acquire_control`` service (the same approach
``rammp_orchestrator``'s ``mocks/static_overlay_node.py`` uses for the same
reason) rather than through the Orchestrator's ``~/acquire_ui_control``
proxy -- this lets the test run standalone against just ``ui_bridge_node``,
no Orchestrator required. If the Orchestrator (or another engine) is also
running and reclaims ``ui_control`` mid-test, the in-flight goal will be
aborted by the bridge and the next request will simply re-acquire -- see
``ENGINE_UI_INTEGRATION_GUIDE.md``'s ownership notes.

Usage::

    ros2 run rammp_ui_bridge test_request_user_input_node
    # or with overrides:
    ros2 run rammp_ui_bridge test_request_user_input_node --ros-args \
        -p ui_bridge_node:=ui_bridge_node \
        -p message:="Please confirm the arm is clear" \
        -p repeat_delay_s:=3.0
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rammp_common_interfaces.srv import AcquireControl, ReleaseControl
from rammp_ui_bridge_interfaces.action import RequestUserInput

_CALL_TIMEOUT_S = 10.0
_GOAL_TIMEOUT_S = 600.0  # a human may take a while to press a button; don't rush them

# Panel-level `style` (docs/USER_INPUT_JSON_SCHEMA.md section 6.1) cycles
# through these across repeated sends; "normal" is the schema's default and
# is therefore omitted from the panel dict entirely rather than sent
# explicitly (see _build_panel).
_PANEL_STYLES = ("normal", "warning", "error")

# One button of each documented per-button style (section 6.3) in every
# single panel, so all three render side by side for visual comparison.
_BUTTONS = [
    {"button_id": "ok", "label": "OK", "style": "normal"},
    {"button_id": "caution", "label": "Caution", "style": "warning"},
    {"button_id": "danger", "label": "Danger", "style": "error"},
]


class TestRequestUserInputNode(Node):
    def __init__(self) -> None:
        super().__init__("test_request_user_input_node")

        self.declare_parameter("ui_bridge_node", "ui_bridge_node")
        self.declare_parameter("engine_id", "test_request_user_input_node")
        self.declare_parameter("title", "Test Request User Input")
        self.declare_parameter(
            "message", "This is a test message with three buttons of different styles."
        )
        # How long to wait after one button press before sending the next
        # panel -- keeps this node usable as a repeatable manual-test loop
        # without needing to relaunch it for every attempt.
        self.declare_parameter("repeat_delay_s", 3.0)

        ns = f"/{self.get_parameter('ui_bridge_node').value}"
        self._engine_id = self.get_parameter("engine_id").value

        self._acquire_client = self.create_client(AcquireControl, f"{ns}/ui_control/acquire_control")
        self._release_client = self.create_client(ReleaseControl, f"{ns}/ui_control/release_control")
        self._action_client = ActionClient(self, RequestUserInput, f"{ns}/request_user_input")

        self._token: bytes | None = None
        self._panel_style_idx = 0

    # -- ui_control arbitration (direct-to-bridge, see module docstring) ----

    def _acquire_control(self) -> bytes | None:
        if not self._acquire_client.wait_for_service(timeout_sec=_CALL_TIMEOUT_S):
            self.get_logger().error("ui_bridge's ~/ui_control/acquire_control never became available")
            return None
        future = self._acquire_client.call_async(AcquireControl.Request(owner_id=self._engine_id))
        rclpy.spin_until_future_complete(self, future, timeout_sec=_CALL_TIMEOUT_S)
        response = future.result()
        if response is None or not response.accepted:
            self.get_logger().error("ui_control acquire_control rejected/timed out")
            return None
        token = bytes(response.token)
        self.get_logger().info(
            f"acquired ui_control token {token.hex()[:8]}... (generation {response.generation})"
        )
        return token

    def _release_control(self) -> None:
        if self._token is None:
            return
        if self._release_client.wait_for_service(timeout_sec=_CALL_TIMEOUT_S):
            future = self._release_client.call_async(ReleaseControl.Request(token=list(self._token)))
            rclpy.spin_until_future_complete(self, future, timeout_sec=_CALL_TIMEOUT_S)
        self.get_logger().info(f"released ui_control token {self._token.hex()[:8]}...")
        self._token = None

    # -- panel construction ---------------------------------------------------

    def _build_panel(self) -> dict:
        style = _PANEL_STYLES[self._panel_style_idx % len(_PANEL_STYLES)]
        panel: dict = {
            "title": self.get_parameter("title").value,
            "message": f"{self.get_parameter('message').value} (panel style: {style})",
            "buttons": _BUTTONS,
        }
        if style != "normal":
            panel["style"] = style
        return panel

    # -- one request/response round trip --------------------------------------

    def send_one(self) -> str | None:
        """Sends one ``RequestUserInput`` goal and blocks until a button is
        pressed, or the goal is rejected/cancelled/aborted/times out.
        Returns the pressed ``button_id``, or ``None`` for any non-button
        outcome (already logged by the time this returns)."""
        if self._token is None:
            self._token = self._acquire_control()
            if self._token is None:
                return None

        if not self._action_client.wait_for_server(timeout_sec=_CALL_TIMEOUT_S):
            self.get_logger().error("ui_bridge's ~/request_user_input server unavailable")
            return None

        panel = self._build_panel()
        self._panel_style_idx += 1
        goal = RequestUserInput.Goal(
            engine_id=self._engine_id,
            token=list(self._token),
            request_json=json.dumps({"panel_json": json.dumps(panel)}),
        )
        self.get_logger().info(
            f"sending request_user_input: {panel['message']!r} with buttons "
            f"{[b['button_id'] for b in panel['buttons']]}"
        )

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=_CALL_TIMEOUT_S)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("request_user_input goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=_GOAL_TIMEOUT_S)
        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().warning("request_user_input goal timed out waiting for a button press")
            return None

        result = json.loads(result_wrapper.result.result_json or "{}")
        status = result.get("status")
        if status == "button":
            button_id = result.get("button_id", "")
            self.get_logger().info(f"button pressed: '{button_id}'")
            return button_id
        if status == "cancelled":
            self.get_logger().info("goal cancelled")
        elif status == "aborted":
            self.get_logger().warning(f"goal aborted: {result.get('reason', '')}")
        return None

    def run_loop(self) -> None:
        delay = self.get_parameter("repeat_delay_s").value
        try:
            while rclpy.ok():
                self.send_one()
                time.sleep(delay)
        except KeyboardInterrupt:
            pass
        finally:
            self._release_control()


def main() -> None:
    rclpy.init()
    node = TestRequestUserInputNode()
    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
