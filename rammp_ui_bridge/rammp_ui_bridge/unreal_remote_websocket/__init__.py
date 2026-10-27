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
    ):
        self.base_url = f"ws://{host}:{ws_port}"
        self.loop = asyncio.new_event_loop()
        self.preset = preset
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

        t = threading.Thread(target=self.start_async_loop, daemon=True)
        t.start()
        asyncio.run_coroutine_threadsafe(self.ws_client_handler(), self.loop)
        asyncio.run_coroutine_threadsafe(self.parse_value_changes(), self.loop)

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
                        # print(f"Received command from UE: {json.dumps(command)}")
                        if "ResponseCode" in command:
                            await self._response_queue.put(command)
                        if "ChangedFields" in command:
                            await self._value_change_queue.put(
                                command["ChangedFields"][0]
                            )
                            print(
                                f"Received value change from UE: {command['ChangedFields'][0]}"
                            )
                        # print(f"Received command from Ethernet GUI: {command}")
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

    def call_function(self, function_name: str, function_parameters: dict[str, Any]):
        Parameters = {
            "Url": "/remote/preset/" + self.preset + "/function/" + function_name,
            "Verb": "PUT",
            "Body": {
                "parameters": function_parameters,
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
                ) and changes_json.get("PropertyLabel") == "UserInput":
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
