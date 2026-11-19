// RMSS binary streaming protocol -- wire format and serialization.
//
// C++ port of ramms_tools/streaming/protocol.py, kept byte-for-byte
// compatible. All multi-byte header fields are little-endian. The 32-byte
// header layout:
//
//     Offset  Size  Field
//     ------  ----  -----
//      0       4    Magic ("RMSS")
//      4       1    Version (1)
//      5       1    MessageType
//      6       2    ChannelID
//      8       2    Flags
//     10       4    SequenceNum
//     14       8    Timestamp (us since epoch)
//     22       4    MetadataLen
//     26       4    PayloadLen
//     30       2    Reserved
//
// Packed/unpacked explicitly byte-by-byte (not via struct-layout tricks) so
// this is correct regardless of host endianness/padding.
#ifndef RAMMP_UI_BRIDGE_STREAMING__RMSS_PROTOCOL_HPP_
#define RAMMP_UI_BRIDGE_STREAMING__RMSS_PROTOCOL_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rammp_ui_bridge_streaming
{

constexpr std::size_t kHeaderSize = 32;
constexpr char kMagic[4] = {'R', 'M', 'S', 'S'};
constexpr uint8_t kVersion = 1;

// RMSS message types (subset this node actually sends).
enum class MessageType : uint8_t
{
  kNone = 0x00,
  kFrameRgb = 0x01,
  kFrameDepth = 0x02,
  kFrameRgbd = 0x03,
  kFrameMotion = 0x04,
  kPointCloud = 0x05,
  kOctoMap = 0x06,
  kImageData = 0x10,
  kFrameData = 0x11,
  kMetadataOnly = 0xF0,
  kSubscribe = 0xF1,
  kUnsubscribe = 0xF2,
  kAck = 0xFD,
  kError = 0xFE,
  kPing = 0xFF,
};

/// 32-byte RMSS message header.
struct StreamHeader
{
  MessageType message_type{MessageType::kNone};
  uint16_t channel_id{0};
  uint16_t flags{0};
  uint32_t sequence_num{0};
  int64_t timestamp{0};  // microseconds since epoch
  uint32_t metadata_len{0};
  uint32_t payload_len{0};

  [[nodiscard]] std::size_t total_message_size() const
  {
    return kHeaderSize + metadata_len + payload_len;
  }

  [[nodiscard]] std::vector<uint8_t> to_bytes() const;

  /// Returns std::nullopt on magic/version mismatch or short input, mirroring
  /// StreamHeader.from_bytes in protocol.py.
  static std::optional<StreamHeader> from_bytes(const uint8_t * data, std::size_t len);

  /// Current time as microseconds since epoch (StreamHeader.now_timestamp).
  static int64_t now_timestamp();
};

/// Complete RMSS message: header + metadata (UTF-8 JSON string) + payload.
struct StreamMessage
{
  StreamHeader header;
  std::string metadata;
  std::vector<uint8_t> payload;

  /// Serializes header (with metadata_len/payload_len filled in) + metadata
  /// + payload into one contiguous buffer, ready for a single sendall().
  [[nodiscard]] std::vector<uint8_t> serialize() const;
};

}  // namespace rammp_ui_bridge_streaming

#endif  // RAMMP_UI_BRIDGE_STREAMING__RMSS_PROTOCOL_HPP_
