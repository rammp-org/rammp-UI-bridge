// Unit tests for the RMSS header pack/unpack, checked against known byte
// layouts derived from ramms_tools/streaming/protocol.py's struct format
// ("<4sBBHHIqIIH") to guarantee wire compatibility with the Python sender.
#include <gtest/gtest.h>

#include "rammp_ui_bridge_streaming/rmss_protocol.hpp"

using rammp_ui_bridge_streaming::kHeaderSize;
using rammp_ui_bridge_streaming::MessageType;
using rammp_ui_bridge_streaming::StreamHeader;
using rammp_ui_bridge_streaming::StreamMessage;

TEST(RmssProtocol, HeaderRoundTrip)
{
  StreamHeader h;
  h.message_type = MessageType::kImageData;
  h.channel_id = 0;
  h.flags = 0;
  h.sequence_num = 42;
  h.timestamp = 1700000000123456LL;
  h.metadata_len = 10;
  h.payload_len = 1000;

  auto bytes = h.to_bytes();
  ASSERT_EQ(bytes.size(), kHeaderSize);

  auto parsed = StreamHeader::from_bytes(bytes.data(), bytes.size());
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->message_type, MessageType::kImageData);
  EXPECT_EQ(parsed->channel_id, 0u);
  EXPECT_EQ(parsed->sequence_num, 42u);
  EXPECT_EQ(parsed->timestamp, 1700000000123456LL);
  EXPECT_EQ(parsed->metadata_len, 10u);
  EXPECT_EQ(parsed->payload_len, 1000u);
}

TEST(RmssProtocol, HeaderFieldByteOffsets)
{
  // Cross-checks the exact byte offsets documented in protocol.py's
  // HEADER_FMT = "<4sBBHHIqIIH" (and this header's own docstring) --
  // magic(4) version(1) type(1) channel(2) flags(2) seq(4) ts(8) meta(4)
  // payload(4) reserved(2) = 32 bytes total, little-endian.
  StreamHeader h;
  h.message_type = MessageType::kFrameDepth;  // 0x02
  h.channel_id = 0x0102;
  h.flags = 0x0304;
  h.sequence_num = 0x05060708;
  h.timestamp = 0x0102030405060708LL;
  h.metadata_len = 0x11121314;
  h.payload_len = 0x21222324;

  auto b = h.to_bytes();
  ASSERT_EQ(b.size(), 32u);

  EXPECT_EQ(std::string(b.begin(), b.begin() + 4), "RMSS");
  EXPECT_EQ(b[4], 1);     // version
  EXPECT_EQ(b[5], 0x02);  // message_type
  EXPECT_EQ(b[6], 0x02);  // channel_id low byte
  EXPECT_EQ(b[7], 0x01);  // channel_id high byte
  EXPECT_EQ(b[8], 0x04);  // flags low byte
  EXPECT_EQ(b[9], 0x03);  // flags high byte
  EXPECT_EQ(b[10], 0x08); // sequence_num byte 0 (LSB)
  EXPECT_EQ(b[13], 0x05); // sequence_num byte 3 (MSB)
  EXPECT_EQ(b[14], 0x08); // timestamp byte 0 (LSB)
  EXPECT_EQ(b[21], 0x01); // timestamp byte 7 (MSB)
  EXPECT_EQ(b[22], 0x14); // metadata_len byte 0
  EXPECT_EQ(b[26], 0x24); // payload_len byte 0
  EXPECT_EQ(b[30], 0x00); // reserved
  EXPECT_EQ(b[31], 0x00); // reserved
}

TEST(RmssProtocol, RejectsBadMagicOrVersion)
{
  std::vector<uint8_t> bad(kHeaderSize, 0);
  EXPECT_FALSE(StreamHeader::from_bytes(bad.data(), bad.size()).has_value());

  std::vector<uint8_t> tooShort(kHeaderSize - 1, 0);
  EXPECT_FALSE(StreamHeader::from_bytes(tooShort.data(), tooShort.size()).has_value());
}

TEST(RmssProtocol, MessageSerializeFillsLengthsAndConcatenates)
{
  StreamMessage msg;
  msg.header.message_type = MessageType::kImageData;
  msg.header.channel_id = 2;
  msg.metadata = R"({"w":4,"h":2,"fmt":"rgb8"})";
  msg.payload = {1, 2, 3, 4, 5, 6, 7, 8};

  auto bytes = msg.serialize();
  ASSERT_EQ(bytes.size(), kHeaderSize + msg.metadata.size() + msg.payload.size());

  auto parsed = StreamHeader::from_bytes(bytes.data(), bytes.size());
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->metadata_len, msg.metadata.size());
  EXPECT_EQ(parsed->payload_len, msg.payload.size());

  std::string meta_out(
    bytes.begin() + kHeaderSize, bytes.begin() + kHeaderSize + msg.metadata.size());
  EXPECT_EQ(meta_out, msg.metadata);

  std::vector<uint8_t> payload_out(
    bytes.begin() + kHeaderSize + msg.metadata.size(), bytes.end());
  EXPECT_EQ(payload_out, msg.payload);
}
