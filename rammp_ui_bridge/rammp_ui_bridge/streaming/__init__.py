"""RMSS streaming client -- vendored from the Gen1 prototype
(``rammp_prototype_gui.streaming``, part of ``rammp-org/Demo-Software``)
unchanged, per §5.2.A of ``docs/NODE_INTERFACE.md``: the bridge reuses the
Gen1 TCP streaming protocol/sender for bulk pixel data (camera frames),
separate from the Remote Control WebSocket (``unreal_remote_websocket``)
used for everything else.

Only ``StreamSender`` (the "bridge is a TCP client of UE's streaming
server" direction) is vendored -- ``StreamClient`` (the receive side) has
no use here.
"""

from .protocol import Compression, MessageType, StreamHeader, StreamMessage
from .sender import StreamSender

__all__ = [
    "Compression",
    "MessageType",
    "StreamHeader",
    "StreamMessage",
    "StreamSender",
]
