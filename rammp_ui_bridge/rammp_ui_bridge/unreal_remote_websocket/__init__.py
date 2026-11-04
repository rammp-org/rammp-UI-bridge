"""Generic client for Unreal Engine's Remote Control WebSocket API.

Robot/demo-agnostic: talks to a UE preset by name, exposes ``call_function``
for pushing data/events into UE, and surfaces UE "UserInput" property
changes via a callback. Vendored unchanged from
``rammp_prototype_gui.unreal_remote_websocket`` (the Gen1 prototype GUI
bridge) -- this transport layer was never robot-specific, only the message
schemas built on top of it were.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Optional, Callable
import asyncio
import websockets
import threading
from asyncio import Lock, Queue

logger = logging.getLogger(__name__)


class UnrealRemoteError(Exception):
    """Raised when a remote control API call fails."""

    pass


UserInputCallback = Callable[[str], None]


class UnrealRemoteWebsocket:
    """
    Client for the Unreal Engine Remote Control webSocket API.

    Args:
        host: Hostname or IP of the UE instance (default: "127.0.0.1")
        ws_port: WebSocket API port (default: 30020)
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        ws_port: int = 30020,
        preset: Optional[str] = None,
        timeout: float = 1.0,
        user_input_callback: Optional[UserInputCallback] = None,
        user_input_property_label: str = "LastUIEventJSON",
        poll_ui_events: bool = True,
        ui_event_poll_rate_hz: float = 15.0,
        ui_event_poll_function: str = "PollUIEvents",
    ):
        self.base_url = f"ws://{host}:{ws_port}"
        self.loop = asyncio.new_event_loop()
        self.preset = preset
        # Name of the preset property UE's Blueprint exposes its inbound
        # command JSON through. Confirmed with the UI team as
        # "LastUIEventJSON" (not the bridge's originally-documented
        # "UserInput") -- see docs/NODE_INTERFACE.md §5.1.
        #
        # Superseded (per the UI team, 2026-11-04) by the PollUIEvents
        # polling mechanism below -- kept fully intact/running alongside it
        # (not removed) in case it's needed again; if UE never sets this
        # property under the new design, this path is simply a harmless
        # no-op.
        self.user_input_property_label = user_input_property_label
        self.ws_shutdown = False
        self.default_timeout = timeout
        self._response_queue = Queue()  # 1 response per request
        self._value_change_queue = Queue()  # For unsolicited value changes from UE
        self._send_lock = Lock()  # serialize sends
        self.ws_client = None
        self.user_input_callback = user_input_callback
        # Only print "GUI connection error"/"GUI disconnected" on the first
        # failed attempt of a disconnected streak -- without this, the retry
        # loop below (reconnect every 3s while UE isn't running) spams one
        # pair of these lines per attempt forever. Reset to False as soon as
        # a connection actually succeeds, so the next genuine disconnect is
        # still reported.
        self._disconnected_logged = False

        # -- PollUIEvents (2026-11-04 UI-team design) --------------------
        # UE no longer pushes individual taps as unsolicited property
        # changes; instead it buffers up to the last 16 button_press/
        # object_tapped events behind a monotonic sequence counter, and
        # expects the bridge to poll for them at 10-20Hz via a tunneled
        # "PollUIEvents" Remote Control function call
        # (Parameters={"SinceSequence": <last seq seen>}), each response
        # being {"LatestSequence": N, "Events": ["<json>", ...]} with only
        # events whose seq > SinceSequence included. A LatestSequence lower
        # than our own last-seen value means UE restarted -- reset to 0 (the
        # caller is responsible for re-pushing current panel/object state
        # after a restart; this transport layer only resets its own counter
        # and surfaces the restart via user_input_callback's normal event
        # dispatch is not applicable here, so callers should watch
        # is_connected()/reconnect logging for that instead).
        self.poll_ui_events = poll_ui_events
        self.ui_event_poll_rate_hz = ui_event_poll_rate_hz
        self.ui_event_poll_function = ui_event_poll_function
        self._ui_event_since_sequence = 0

        t = threading.Thread(target=self.start_async_loop, daemon=True)
        t.start()
        asyncio.run_coroutine_threadsafe(self.ws_client_handler(), self.loop)
        asyncio.run_coroutine_threadsafe(self.parse_value_changes(), self.loop)
        if self.poll_ui_events:
            asyncio.run_coroutine_threadsafe(self.poll_ui_events_loop(), self.loop)

    async def ws_client_handler(self):
        # uri = "ws://192.168.68.51:30020"
        while not self.ws_shutdown:
            try:
                async with websockets.connect(self.base_url) as ws:
                    self.ws_client = ws
                    print("GUI connected.")
                    self._disconnected_logged = False
                    await self.register_preset()  # Register preset on connect
                    print("Preset registration sent.")
                    async for message in ws:
                        command = json.loads(message)
                        if "ResponseCode" in command:
                            await self._response_queue.put(command)
                        if "ChangedFields" in command:
                            await self._value_change_queue.put(
                                command["ChangedFields"][0]
                            )
                            print(
                                f"Received value change from UE: {command['ChangedFields'][0]}"
                            )
                        if self.ws_shutdown:
                            break
            except Exception as e:
                if not self.ws_shutdown:
                    await self._response_queue.put(
                        {"_error": str(e)}
                    )  # Unblock any waiting send_and_wait calls
                    if not self._disconnected_logged:
                        print(f"GUI connection error: {e}")
            finally:
                self.ws_client = None
                if not self.ws_shutdown:
                    if not self._disconnected_logged:
                        print("GUI disconnected.")
                        self._disconnected_logged = True
                    await asyncio.sleep(3)  # Wait before trying to reconnect


    async def register_preset(self):
        if self.preset is None:
            print("No preset specified, skipping registration.")
            return
        if self.ws_client is None:
            print("WebSocket client not connected, cannot register preset.")
            return
        command = {
            "MessageName": "preset.register",
            "Parameters": {
                "PresetName": self.preset,
            },
        }
        await self.ws_client.send(json.dumps(command))

    async def send_and_wait(self, parameters, timeout=None):
        """
        Sends a tunneled HTTP request via WS and waits for the next response.
        This function holds a global send lock, guaranteeing only one in flight.
        """
        if self.ws_client is None:
            raise RuntimeError("WebSocket is not open")

        payload = {
            "MessageName": "http",
            # You *can* include Id, but UE may respond with RequestId=-1.
            "Parameters": parameters,
        }
        async with self._send_lock:
            await self.ws_client.send(json.dumps(payload))
            # try:
            #     resp = await asyncio.wait_for(
            #         self._response_queue.get(), timeout or self.default_timeout
            #     )
            # except asyncio.TimeoutError as te:
            #     raise TimeoutError(
            #         f"Timeout waiting for WS response to {parameters}"
            #     ) from te

            # if "_error" in resp:
            #     raise RuntimeError(f"WebSocket receive loop error: {resp['_error']}")
            # return resp

    async def call_function_and_wait(
        self,
        function_name: str,
        function_parameters: dict[str, Any],
        timeout=None,
    ) -> dict:
        """Same tunneled HTTP call as ``call_function``, but actually waits
        for and returns UE's response -- used by the PollUIEvents poller,
        which needs the real ``ReturnedValues`` (``LatestSequence``/
        ``Events``), not just a fire-and-forget push like ``call_function``.

        Holds ``_send_lock`` across the full send+wait round trip (not just
        the send, unlike ``send_and_wait``) so no other sender's request can
        be interleaved while we wait -- otherwise, since UE's responses
        aren't reliably correlated by request ID (see ``send_and_wait``'s
        comment), we could dequeue a response meant for a concurrent
        ``call_function`` fire-and-forget call instead of our own.
        """
        if self.ws_client is None:
            raise RuntimeError("WebSocket is not open")

        parameters = {
            "Url": "/remote/preset/" + self.preset + "/function/" + function_name,
            "Verb": "PUT",
            "Body": {
                "Parameters": function_parameters,
                "GenerateTransaction": False,
            },
        }
        payload = {"MessageName": "http", "Parameters": parameters}
        async with self._send_lock:
            # Drain any stale responses left over from prior
            # call_function() fire-and-forget calls (which never consume
            # their own response) so we don't mistakenly return one of
            # those instead of our own.
            while not self._response_queue.empty():
                self._response_queue.get_nowait()
            await self.ws_client.send(json.dumps(payload))
            try:
                resp = await asyncio.wait_for(
                    self._response_queue.get(), timeout or self.default_timeout
                )
            except asyncio.TimeoutError as te:
                raise TimeoutError(
                    f"Timeout waiting for WS response to {parameters}"
                ) from te

        if "_error" in resp:
            raise RuntimeError(f"WebSocket receive loop error: {resp['_error']}")
        return resp

    async def poll_ui_events_loop(self):
        """Background loop (2026-11-04 UI-team design): polls UE's
        ``PollUIEvents`` Remote Control function at ``ui_event_poll_rate_hz``
        and dispatches each returned event string to ``user_input_callback``
        exactly like the legacy property-watch path (``parse_value_changes``)
        does -- so downstream dispatch code needs no changes to consume
        events from either transport.

        Starts at ``SinceSequence=0`` and always passes the last
        ``LatestSequence`` it received. If UE ever reports a
        ``LatestSequence`` lower than our own last-seen value, UE has
        restarted -- reset to 0 (the caller is responsible for re-pushing
        current panel/object state after a restart, same as any other
        reconnect)."""
        period = 1.0 / self.ui_event_poll_rate_hz
        while not self.ws_shutdown:
            if self.ws_client is None or self.preset is None:
                await asyncio.sleep(period)
                continue
            try:
                resp = await self.call_function_and_wait(
                    self.ui_event_poll_function,
                    {"SinceSequence": self._ui_event_since_sequence},
                )
            except Exception as e:
                logger.debug(f"PollUIEvents error: {e!r}")
                await asyncio.sleep(period)
                continue

            if resp.get("ResponseCode", 0) != 200:
                logger.debug(f"PollUIEvents non-200 response: {resp}")
                await asyncio.sleep(period)
                continue

            returned_values = resp.get("ResponseBody", {}).get("ReturnedValues", [])
            result = returned_values[0] if returned_values else {}
            latest_sequence = result.get("LatestSequence", self._ui_event_since_sequence)
            events = result.get("Events", [])

            if latest_sequence < self._ui_event_since_sequence:
                logger.info(
                    "PollUIEvents: LatestSequence went backwards "
                    f"({latest_sequence} < {self._ui_event_since_sequence}) -- "
                    "UE restarted, resetting SinceSequence to 0"
                )
                self._ui_event_since_sequence = 0
            else:
                self._ui_event_since_sequence = latest_sequence

            for event_json in events:
                logger.debug(f"UI event via PollUIEvents: {event_json}")
                if self.user_input_callback is not None:
                    self.user_input_callback(event_json)

            await asyncio.sleep(period)

    def call_function(self, function_name: str, function_parameters: dict[str, Any]):
        Parameters = {
            "Url": "/remote/preset/" + self.preset + "/function/" + function_name,
            "Verb": "PUT",
            # Body shape matches UE's Remote Control HTTP API exactly as
            # confirmed/tested by the UI team (capitalized "Parameters" +
            # explicit "GenerateTransaction": false) -- this is the same
            # body UE's WS "http" tunnel hands to its internal HTTP
            # handler, so it must match a direct-HTTP PUT byte-for-byte.
            "Body": {
                "Parameters": function_parameters,
                "GenerateTransaction": False,
            },
        }
        try:
            asyncio.run_coroutine_threadsafe(self.send_and_wait(Parameters), self.loop)
            # resp = asyncio.run_coroutine_threadsafe(
            #     self.send_and_wait(Parameters), self.loop
            # ).result()
            # if resp.get("ResponseCode", 0) != 200:
            #     raise UnrealRemoteError(
            #         f"UE responded with error code {resp.get('ResponseCode')}: {resp.get('Body')}"
            #     )
            # # print(resp.get("ResponseBody", {}))
            # return resp.get("ResponseBody", {})
        except Exception as e:
            logger.error(
                f"Error calling function {function_name} with parameters {function_parameters}: {e}"
            )
            raise

    def get_preset_functions_porperties(self):
        Parameters = {
            "Url": "/remote/preset/" + self.preset,
            "Verb": "GET",
        }
        try:
            resp = asyncio.run_coroutine_threadsafe(
                self.send_and_wait(Parameters), self.loop
            ).result()
            if resp.get("ResponseCode", 0) != 200:
                raise UnrealRemoteError(
                    f"UE responded with error code {resp.get('ResponseCode')}: {resp.get('Body')}"
                )
            print(resp.get("ResponseBody", {}))
            return resp.get("ResponseBody", {})
        except Exception as e:
            logger.error(f"Error getting preset functions and properties: {e}")
            raise

    def start_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def parse_value_changes(self):
        while not self.ws_shutdown:
            # wait for queue to have items, but with timeout so we can check for shutdown
            try:
                changes = await asyncio.wait_for(
                    self._value_change_queue.get(), timeout=1.0
                )
                changes_json = (
                    json.loads(changes) if isinstance(changes, str) else changes
                )

                if (
                    "PropertyLabel" in changes_json and "PropertyValue" in changes_json
                ) and changes_json.get("PropertyLabel") == self.user_input_property_label:
                    if self.user_input_callback is not None:
                        self.user_input_callback(changes_json.get("PropertyValue"))
                    # print(f"Userinput: {changes_json.get('PropertyValue')}")
            except asyncio.TimeoutError:
                pass

    def __repr__(self) -> str:
        return f"UnrealRemote('{self.base_url}')"

    def is_connected(self) -> bool:
        return self.ws_client is not None

    def shutdown(self):
        self.ws_shutdown = True
        if self.ws_client is not None:
            asyncio.run_coroutine_threadsafe(self.ws_client.close(), self.loop)
