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
# Debug-level traffic logging (every outbound command + every raw inbound
# message/response from UE) is only useful if something actually emits it --
# this module runs on its own background thread inside ui_bridge_node, which
# never configures Python's stdlib logging itself, so without a handler here
# these .debug() calls would be silently dropped by the logging module's
# WARNING-level "handler of last resort". Attach our own handler here so this
# traffic *can* be seen, but default the level to INFO (quiet) -- every
# call_function()/poll response used to log at DEBUG unconditionally, which
# floods the console at a high-frequency poll/forward rate. ui_bridge_node
# raises this back to DEBUG at startup via its own ue_log_level parameter
# when per-call WS traffic needs to be inspected.
if not logger.handlers:
    _handler = logging.StreamHandler()
    _handler.setFormatter(logging.Formatter("%(asctime)s [%(name)s] %(levelname)s: %(message)s"))
    logger.addHandler(_handler)
    logger.propagate = False
    logger.setLevel(logging.INFO)


class UnrealRemoteError(Exception):
    """Raised when a remote control API call fails."""

    pass


UserInputCallback = Callable[[str], None]

# UFUNCTION/C++ names of the RammsBridgeReceiver functions the bridge needs
# to call, used to auto-resolve each one's *current* preset + DisplayName
# (see UnrealRemoteWebsocket.resolve_functions) -- UE's Remote Control HTTP
# API routes a call by DisplayName/Label, which the UI team can rename at
# any time, but UnderlyingFunction.Name (the bare UFUNCTION name) is not
# renameable from the Remote Control UI, so it's the stable key to match
# against.
RESOLVED_FUNCTION_CPP_NAMES = frozenset(
    {
        "UpdateDetectedObjects",
        "UpdateWorldState",
        "PollUIEvents",
        "UpdatePanel",
        "OpenCameraFeed",
        "CloseCameraFeed",
    }
)


def _is_empty_poll_events_response(command: dict) -> bool:
    """True if ``command`` is a PollUIEvents response shaped like
    ``{"ResponseBody": {"ReturnedValues": [{"LatestSequence": N, "Events": []}]}}``
    with no new events -- i.e. a routine, nothing-happened poll result. Used
    to silence the generic WS traffic logging for these (they fire at
    ``ui_event_poll_rate_hz``, ~15/sec, and would otherwise drown out
    everything else in the logs)."""
    body = command.get("ResponseBody")
    if not isinstance(body, dict):
        return False
    returned_values = body.get("ReturnedValues")
    if not returned_values or not isinstance(returned_values, list):
        return False
    result = returned_values[0]
    if not isinstance(result, dict) or "LatestSequence" not in result:
        return False
    return not result.get("Events")


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
        # "Poll UIEvents" is the exposed field's Label (UE's Remote Control
        # preset routes resolve a function call against the field's
        # Label/DisplayName, not its bare UFUNCTION name -- see the
        # matching call_function() fixes in ui_bridge_node.py).
        ui_event_poll_function: str = "Poll UIEvents",
        # resolve_functions() reruns at this period, but only until it
        # fully resolves for the *current* WS connection -- a
        # freshly-(re)connected UE can report an exposed function's
        # OwnerObjects as empty for a few seconds until its
        # RammsBridgeReceiver actor finishes BeginPlay and binds, so a
        # single attempt right after connecting can race this and resolve
        # nothing (confirmed live, 2026-11-16). Once resolved, it stays
        # resolved until the next reconnect (e.g. UE restarted with a
        # different build/release that renamed a DisplayName or moved a
        # function to another preset) -- function names don't change while
        # a single UE instance keeps running, so there's no need to keep
        # re-querying UE indefinitely once they're known.
        function_resolve_period_s: float = 5.0,
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
        # Dedup state so a persistent error condition (e.g. "preset not
        # found" while UE hasn't registered it yet) logs once instead of
        # spamming one line per WS message/poll tick (~15/sec) until it
        # resolves. Reset to None as soon as a 200 response is seen again.
        self._last_logged_error_response = None
        self._last_poll_error_response = None
        # Set (while holding _send_lock) by call_function_and_wait(quiet=True)
        # right before sending, cleared by ws_client_handler as soon as it
        # consumes the one response that pairs with it. Because the lock
        # serializes all senders for the full round trip, the next
        # "ResponseCode" message to arrive is guaranteed to be that quiet
        # call's own response -- lets us suppress it unconditionally
        # (regardless of its ReturnedValues shape), unlike the old
        # shape-only _is_empty_poll_events_response() heuristic, which
        # missed UE's actual bare "ReturnedValues": [] responses and let
        # every single ~10-15Hz PollUIEvents response through.
        self._quiet_response_pending = False

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
        self.function_resolve_period_s = function_resolve_period_s
        self._last_logged_resolution: Optional[dict[str, dict[str, str]]] = None
        # Bumped by ws_client_handler every time a new WS connection is
        # established -- lets resolve_functions_loop know it's looking at a
        # fresh connection (possibly a different UE build/release) and
        # should retry resolving again, rather than treating the previous
        # connection's already-resolved result as still good forever.
        self._connection_generation = 0
        self._ui_event_since_sequence = 0
        # Populated by resolve_functions() (scheduled on every WS (re)connect,
        # alongside list_presets()): cpp_name -> {"preset": ..., "display_name": ...}
        # for each of RESOLVED_FUNCTION_CPP_NAMES actually found exposed (with
        # a non-empty OwnerObjects) in some registered preset.
        self._function_resolution: dict[str, dict[str, str]] = {}

        t = threading.Thread(target=self.start_async_loop, daemon=True)
        t.start()
        asyncio.run_coroutine_threadsafe(self.ws_client_handler(), self.loop)
        asyncio.run_coroutine_threadsafe(self.parse_value_changes(), self.loop)
        asyncio.run_coroutine_threadsafe(self.resolve_functions_loop(), self.loop)
        if self.poll_ui_events:
            asyncio.run_coroutine_threadsafe(self.poll_ui_events_loop(), self.loop)

    async def ws_client_handler(self):
        # uri = "ws://192.168.68.51:30020"
        while not self.ws_shutdown:
            try:
                async with websockets.connect(self.base_url) as ws:
                    self.ws_client = ws
                    self._connection_generation += 1
                    print("GUI connected.")
                    self._disconnected_logged = False
                    await self.register_preset()  # Register preset on connect
                    print("Preset registration sent.")
                    asyncio.create_task(self.list_presets())
                    async for message in ws:
                        command = json.loads(message)
                        response_code = command.get("ResponseCode")
                        # Routine "nothing happened" PollUIEvents responses
                        # fire at ui_event_poll_rate_hz (~15/sec) -- don't
                        # log those, only log when SinceSequence actually
                        # advanced with new events (or for any other kind
                        # of message/response). Likewise, a persistent error
                        # (e.g. "preset not found" while UE hasn't
                        # registered it yet) would otherwise re-log the same
                        # line every poll tick until it resolves -- only log
                        # it once, on first occurrence or when it changes.
                        is_quiet_response = (
                            "ResponseCode" in command
                            and response_code == 200
                            and self._quiet_response_pending
                        )
                        if "ResponseCode" in command:
                            self._quiet_response_pending = False
                        is_routine_poll = _is_empty_poll_events_response(command)
                        is_repeated_error = (
                            response_code is not None
                            and response_code != 200
                            and command == self._last_logged_error_response
                        )
                        should_log = (
                            not is_routine_poll and not is_repeated_error and not is_quiet_response
                        )
                        if should_log:
                            logger.debug(f"WS <- {message}")
                        if "ResponseCode" in command:
                            if should_log:
                                logger.debug(
                                    f"WS response: code={response_code} "
                                    f"body={command.get('ResponseBody')}"
                                )
                            self._last_logged_error_response = (
                                command if response_code != 200 else None
                            )
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
                    logger.debug(f"WS receive loop error: {e!r}")
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
        logger.debug(f"WS -> {command}")
        await self.ws_client.send(json.dumps(command))

    async def list_presets(self):
        """GET /remote/presets via the WS "http" tunnel, logged once right
        after connecting (and scheduled as its own task rather than awaited
        inline, since ``ws_client_handler``'s own ``async for`` receive loop
        hasn't started yet at that point and is the thing that actually
        fulfils this request/response round trip).

        Purely diagnostic: lets a "preset not found"/404 error from
        ``register_preset`` or the ``PollUIEvents`` poller be compared
        against what UE's Remote Control plugin actually has registered at
        that moment (e.g. a typo'd preset name, or UE not having loaded the
        map/preset yet)."""
        if self.ws_client is None:
            return
        parameters = {"Url": "/remote/presets", "Verb": "GET"}
        payload = {"MessageName": "http", "Parameters": parameters}
        async with self._send_lock:
            while not self._response_queue.empty():
                self._response_queue.get_nowait()
            await self.ws_client.send(json.dumps(payload))
            try:
                resp = await asyncio.wait_for(
                    self._response_queue.get(), self.default_timeout
                )
            except asyncio.TimeoutError:
                logger.warning("Timed out listing UE Remote Control presets")
                return

        if resp.get("ResponseCode", 0) != 200:
            logger.warning(f"Failed to list UE Remote Control presets: {resp}")
            return
        presets = resp.get("ResponseBody", {}).get("Presets", [])
        names = [p.get("Name") for p in presets]
        logger.info(f"UE Remote Control presets available: {names}")

    # -- Debug-only helpers (not used by the normal bridge runtime path) --
    # Unlike list_presets()/get_preset_functions_porperties() above, these
    # actually *return* the parsed data (instead of just logging, or
    # relying on send_and_wait()'s currently-dead/commented-out response
    # wait) so a debug-only HTTP endpoint on ui_bridge_node can expose them.
    async def get_presets(self, timeout=None) -> list[dict]:
        """GET /remote/presets via the WS "http" tunnel; returns UE's full
        list of registered presets (not just their names)."""
        if self.ws_client is None:
            raise RuntimeError("WebSocket is not open")
        parameters = {"Url": "/remote/presets", "Verb": "GET"}
        payload = {"MessageName": "http", "Parameters": parameters}
        async with self._send_lock:
            while not self._response_queue.empty():
                self._response_queue.get_nowait()
            await self.ws_client.send(json.dumps(payload))
            try:
                resp = await asyncio.wait_for(
                    self._response_queue.get(), timeout or self.default_timeout
                )
            except asyncio.TimeoutError as te:
                raise TimeoutError(f"Timeout waiting for WS response to {parameters}") from te
        if "_error" in resp:
            raise RuntimeError(f"WebSocket receive loop error: {resp['_error']}")
        if resp.get("ResponseCode", 0) != 200:
            raise UnrealRemoteError(
                f"UE responded with error code {resp.get('ResponseCode')}: {resp.get('Body')}"
            )
        return resp.get("ResponseBody", {}).get("Presets", [])

    async def get_preset(self, name: str, timeout=None) -> dict:
        """GET /remote/preset/<name> via the WS "http" tunnel; returns UE's
        full preset description (exposed functions/properties), for an
        arbitrary preset name -- unlike get_preset_functions_porperties()
        this isn't limited to ``self.preset``."""
        if self.ws_client is None:
            raise RuntimeError("WebSocket is not open")
        parameters = {"Url": "/remote/preset/" + name, "Verb": "GET"}
        payload = {"MessageName": "http", "Parameters": parameters}
        async with self._send_lock:
            while not self._response_queue.empty():
                self._response_queue.get_nowait()
            await self.ws_client.send(json.dumps(payload))
            try:
                resp = await asyncio.wait_for(
                    self._response_queue.get(), timeout or self.default_timeout
                )
            except asyncio.TimeoutError as te:
                raise TimeoutError(f"Timeout waiting for WS response to {parameters}") from te
        if "_error" in resp:
            raise RuntimeError(f"WebSocket receive loop error: {resp['_error']}")
        if resp.get("ResponseCode", 0) != 200:
            raise UnrealRemoteError(
                f"UE responded with error code {resp.get('ResponseCode')}: {resp.get('Body')}"
            )
        return resp.get("ResponseBody", {})

    async def resolve_functions(self) -> dict[str, dict[str, str]]:
        """Auto-discovers, across *every* preset UE currently has
        registered (not just ``self.preset``), which preset+DisplayName
        currently exposes each UFUNCTION in ``RESOLVED_FUNCTION_CPP_NAMES``.

        Matches by ``UnderlyingFunction.Name`` (the bare UFUNCTION/C++ name
        -- stable, can't be renamed from the Remote Control UI) rather than
        ``DisplayName``/Label (renameable by the UI team at any time, but
        the name UE's Remote Control HTTP API actually routes a call by --
        see ``call_function``). Only accepts a match whose ``OwnerObjects``
        is non-empty (an exposed function with no bound OwnerObjects can't
        actually be invoked). Called on every WS (re)connect, alongside
        ``list_presets()``, so a UE restart/Blueprint reload that moves a
        function to a different preset or renames its DisplayName is
        picked up automatically without a bridge restart.

        Keeps whatever was last resolved if this fails/can't reach UE
        rather than clearing it (same reconnect-resilience philosophy as
        the rest of this class) -- only replaces ``self._function_resolution``
        once a full pass over every preset succeeds."""
        try:
            presets = await self.get_presets()
        except Exception as e:
            logger.warning(f"resolve_functions: failed to list presets: {e!r}")
            return self._function_resolution

        resolution: dict[str, dict[str, str]] = {}
        for preset_summary in presets:
            preset_name = preset_summary.get("Name")
            if not preset_name:
                continue
            try:
                body = await self.get_preset(preset_name)
            except Exception as e:
                logger.warning(f"resolve_functions: failed to GET preset {preset_name!r}: {e!r}")
                continue
            preset_body = body.get("Preset", body)
            for group in preset_body.get("Groups", []):
                for entry in group.get("ExposedFunctions", []):
                    if not entry.get("OwnerObjects"):
                        continue
                    underlying = entry.get("UnderlyingFunction") or {}
                    cpp_name = underlying.get("Name")
                    if cpp_name not in RESOLVED_FUNCTION_CPP_NAMES:
                        continue
                    display_name = entry.get("DisplayName")
                    if not display_name:
                        continue
                    existing = resolution.get(cpp_name)
                    if existing is not None and existing["preset"] != preset_name:
                        logger.warning(
                            f"resolve_functions: {cpp_name!r} is exposed in multiple "
                            f"presets ({existing['preset']!r} and {preset_name!r}) -- "
                            f"keeping the first one found ({existing['preset']!r})"
                        )
                        continue
                    resolution[cpp_name] = {"preset": preset_name, "display_name": display_name}

        missing = RESOLVED_FUNCTION_CPP_NAMES - resolution.keys()
        if missing:
            logger.warning(
                f"resolve_functions: could not resolve {sorted(missing)} in any "
                "registered preset (checked OwnerObjects non-empty + "
                "UnderlyingFunction.Name match)"
            )
        if resolution != self._last_logged_resolution:
            logger.info(f"resolve_functions: resolved {resolution}")
            self._last_logged_resolution = dict(resolution)
        else:
            logger.debug(f"resolve_functions: resolved {resolution} (unchanged)")
        self._function_resolution = resolution
        return resolution

    async def resolve_functions_loop(self):
        """Resolves once per WS connection, not indefinitely -- UE's
        exposed function DisplayNames/preset locations only change between
        builds/releases, not while a given UE instance keeps running, so
        there's no need to keep re-querying it forever once resolved.

        Retries at ``function_resolve_period_s`` only until fully resolved
        *for the current connection*: a freshly-(re)connected UE can report
        a just-registered preset's exposed functions with an empty
        OwnerObjects for a few seconds until its RammsBridgeReceiver actor
        finishes BeginPlay and binds, so a single attempt right after
        connecting can race this and resolve nothing (confirmed live,
        2026-11-16). Once resolved, goes quiet (no more WS traffic/logging)
        until the next reconnect (``self._connection_generation`` bumps,
        e.g. UE restarted with a different build) triggers a fresh
        resolution attempt. A no-op while disconnected (``ws_client is
        None``)."""
        resolved_generation = -1
        while not self.ws_shutdown:
            if self.ws_client is not None and resolved_generation != self._connection_generation:
                try:
                    await self.resolve_functions()
                except Exception as e:
                    logger.debug(f"resolve_functions_loop: {e!r}")
                if RESOLVED_FUNCTION_CPP_NAMES <= self._function_resolution.keys():
                    resolved_generation = self._connection_generation
            await asyncio.sleep(self.function_resolve_period_s)

    async def _ensure_resolved(self, cpp_name: str) -> Optional[dict[str, str]]:
        """Returns the cached resolution for ``cpp_name`` if there is one;
        otherwise actively runs one ``resolve_functions()`` attempt right
        now instead of waiting for ``resolve_functions_loop``'s next
        ``function_resolve_period_s`` tick. Closes the real race observed
        live (2026-11-16): the very first call right after connecting can
        otherwise fire before the background loop's first attempt even
        gets a chance to run, always falling back to a (possibly stale,
        UI-team-renamed) hardcoded DisplayName."""
        resolution = self._function_resolution.get(cpp_name)
        if resolution is not None:
            return resolution
        try:
            await self.resolve_functions()
        except Exception as e:
            logger.debug(f"_ensure_resolved({cpp_name!r}): on-demand resolve failed: {e!r}")
        return self._function_resolution.get(cpp_name)

    def call_resolved_function(
        self,
        cpp_name: str,
        function_parameters: dict[str, Any],
        fallback_display_name: Optional[str] = None,
        fallback_preset: Optional[str] = None,
    ) -> None:
        """Fire-and-forget call to whichever preset+DisplayName is resolved
        for ``cpp_name`` (one of ``RESOLVED_FUNCTION_CPP_NAMES``) -- if
        nothing's cached yet, actively resolves on demand first (see
        ``_ensure_resolved``) rather than just hoping
        ``resolve_functions_loop`` got there first. If that still finds
        nothing and a ``fallback_display_name`` is given, falls back to
        that (with ``fallback_preset`` or ``self.preset``) so a resolution
        failure doesn't silently drop the call -- logs a warning either way
        so a persistently-unresolved function is visible. Runs on
        ``self.loop`` (fire-and-forget from the caller's perspective, same
        as ``call_function``)."""

        async def _do() -> None:
            resolution = await self._ensure_resolved(cpp_name)
            if resolution is not None:
                self.call_function(
                    resolution["display_name"], function_parameters, preset=resolution["preset"]
                )
                return
            if fallback_display_name is not None:
                logger.warning(
                    f"call_resolved_function({cpp_name!r}): not resolved -- falling "
                    f"back to hardcoded DisplayName {fallback_display_name!r} on "
                    f"preset {(fallback_preset or self.preset)!r}"
                )
                self.call_function(
                    fallback_display_name, function_parameters, preset=fallback_preset
                )
                return
            logger.warning(
                f"call_resolved_function({cpp_name!r}): not resolved and no fallback "
                "given -- skipping call"
            )

        asyncio.run_coroutine_threadsafe(_do(), self.loop)

    async def call_resolved_function_and_wait(
        self,
        cpp_name: str,
        function_parameters: dict[str, Any],
        timeout=None,
        quiet: bool = False,
        fallback_display_name: Optional[str] = None,
        fallback_preset: Optional[str] = None,
    ) -> dict:
        """Same as ``call_resolved_function`` but awaits and returns UE's
        response, like ``call_function_and_wait`` -- used by
        ``poll_ui_events_loop`` for ``PollUIEvents``."""
        resolution = await self._ensure_resolved(cpp_name)
        if resolution is not None:
            return await self.call_function_and_wait(
                resolution["display_name"],
                function_parameters,
                timeout=timeout,
                quiet=quiet,
                preset=resolution["preset"],
            )
        if fallback_display_name is not None:
            if not quiet:
                logger.warning(
                    f"call_resolved_function_and_wait({cpp_name!r}): not resolved -- "
                    f"falling back to hardcoded DisplayName "
                    f"{fallback_display_name!r} on preset {(fallback_preset or self.preset)!r}"
                )
            return await self.call_function_and_wait(
                fallback_display_name,
                function_parameters,
                timeout=timeout,
                quiet=quiet,
                preset=fallback_preset,
            )
        raise UnrealRemoteError(
            f"call_resolved_function_and_wait({cpp_name!r}): not resolved yet and no "
            "fallback given"
        )

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
            logger.debug(f"WS -> {payload}")
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
        quiet: bool = False,
        preset: Optional[str] = None,
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

        ``quiet=True`` (used by ``poll_ui_events_loop``) skips the "WS ->"
        request log -- the poller calls this at ``ui_event_poll_rate_hz``
        (~15/sec), and logging every single poll request would drown out
        everything else; the response is still logged by
        ``ws_client_handler`` whenever it actually carries new events.
        """
        if self.ws_client is None:
            raise RuntimeError("WebSocket is not open")

        parameters = {
            "Url": "/remote/preset/" + (preset or self.preset) + "/function/" + function_name,
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
            if not quiet:
                logger.debug(f"WS -> {payload}")
            self._quiet_response_pending = quiet
            await self.ws_client.send(json.dumps(payload))
            try:
                resp = await asyncio.wait_for(
                    self._response_queue.get(), timeout or self.default_timeout
                )
            except asyncio.TimeoutError as te:
                self._quiet_response_pending = False
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
                resp = await self.call_resolved_function_and_wait(
                    "PollUIEvents",
                    {"SinceSequence": self._ui_event_since_sequence},
                    quiet=True,
                    fallback_display_name=self.ui_event_poll_function,
                )
            except Exception as e:
                logger.debug(f"PollUIEvents error: {e!r}")
                await asyncio.sleep(period)
                continue

            if resp.get("ResponseCode", 0) != 200:
                if resp != self._last_poll_error_response:
                    logger.debug(f"PollUIEvents non-200 response: {resp}")
                    self._last_poll_error_response = resp
                await asyncio.sleep(period)
                continue
            self._last_poll_error_response = None

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

    def call_function(
        self,
        function_name: str,
        function_parameters: dict[str, Any],
        preset: Optional[str] = None,
    ):
        Parameters = {
            "Url": "/remote/preset/" + (preset or self.preset) + "/function/" + function_name,
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
        logger.debug(f"call_function({function_name!r}, {function_parameters})")
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
