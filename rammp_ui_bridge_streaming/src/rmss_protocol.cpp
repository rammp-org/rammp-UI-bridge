#include "rammp_ui_bridge_streaming/rmss_protocol.hpp"

#include <chrono>
#include <cstring>

namespace rammp_ui_bridge_streaming
{

namespace
{

void put_u16le(std::vector<uint8_t> & buf, uint16_t v)
{
  buf.push_back(static_cast<uint8_t>(v & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void put_u32le(std::vector<uint8_t> & buf, uint32_t v)
{
  for (int i = 0; i < 4; ++i) {
    buf.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  }
}

void put_i64le(std::vector<uint8_t> & buf, int64_t v)
{
  auto uv = static_cast<uint64_t>(v);
  for (int i = 0; i < 8; ++i) {
    buf.push_back(static_cast<uint8_t>((uv >> (8 * i)) & 0xFF));
  }
}

uint16_t get_u16le(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t get_u32le(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

int64_t get_i64le(const uint8_t * p)
{
  uint64_t uv = 0;
  for (int i = 7; i >= 0; --i) {
    uv = (uv << 8) | p[i];
  }
  return static_cast<int64_t>(uv);
}

}  // namespace

std::vector<uint8_t> StreamHeader::to_bytes() const
{
  std::vector<uint8_t> buf;
  buf.reserve(kHeaderSize);
  buf.insert(buf.end(), kMagic, kMagic + 4);
  buf.push_back(kVersion);
  buf.push_back(static_cast<uint8_t>(message_type));
  put_u16le(buf, channel_id);
  put_u16le(buf, flags);
  put_u32le(buf, sequence_num);
  put_i64le(buf, timestamp);
  put_u32le(buf, metadata_len);
  put_u32le(buf, payload_len);
  put_u16le(buf, 0);  // reserved
  return buf;
}

std::optional<StreamHeader> StreamHeader::from_bytes(const uint8_t * data, std::size_t len)
{
  if (len < kHeaderSize) {
    return std::nullopt;
  }
  if (std::memcmp(data, kMagic, 4) != 0) {
    return std::nullopt;
  }
  if (data[4] != kVersion) {
    return std::nullopt;
  }
  StreamHeader h;
  h.message_type = static_cast<MessageType>(data[5]);
  h.channel_id = get_u16le(data + 6);
  h.flags = get_u16le(data + 8);
  h.sequence_num = get_u32le(data + 10);
  h.timestamp = get_i64le(data + 14);
  h.metadata_len = get_u32le(data + 22);
  h.payload_len = get_u32le(data + 26);
  return h;
}

int64_t StreamHeader::now_timestamp()
{
  using namespace std::chrono;
  return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

std::vector<uint8_t> StreamMessage::serialize() const
{
  StreamHeader h = header;
  h.metadata_len = static_cast<uint32_t>(metadata.size());
  h.payload_len = static_cast<uint32_t>(payload.size());

  std::vector<uint8_t> out = h.to_bytes();
  out.reserve(out.size() + metadata.size() + payload.size());
  out.insert(out.end(), metadata.begin(), metadata.end());
  out.insert(out.end(), payload.begin(), payload.end());
  return out;
}

}  // namespace rammp_ui_bridge_streaming
