"""RMSS binary streaming protocol — Python implementation.

Provides StreamClient for receiving data from UE and StreamSender for
sending data to UE, plus the low-level protocol module for
message framing and compression utilities.

Vendored unchanged (only import paths adjusted to be package-relative)
from https://github.com/rammp-org/ramms-tools 's ``ramms_tools.streaming``
(MIT licensed) -- the authoritative source for this protocol/SDK, so this
bridge doesn't need to depend on the ``ramms-tools`` PyPI/console-tool
package just for its streaming client.
"""

from .protocol import (
    HEADER_SIZE,
    MessageType,
    Compression,
    StreamHeader,
    StreamMessage,
)
from .client import StreamClient, ChannelStats
from .sender import StreamSender
from .compression import (
    has_jpeg,
    has_lz4,
    decompress_payload,
    compress_jpeg,
    compress_lz4,
)

__all__ = [
    "HEADER_SIZE",
    "MessageType",
    "Compression",
    "StreamHeader",
    "StreamMessage",
    "StreamClient",
    "ChannelStats",
    "StreamSender",
    "has_jpeg",
    "has_lz4",
    "decompress_payload",
    "compress_jpeg",
    "compress_lz4",
]
