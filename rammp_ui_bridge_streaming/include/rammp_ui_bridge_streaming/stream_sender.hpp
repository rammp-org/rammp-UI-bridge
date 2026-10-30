// C++ port of ramms_tools/streaming/sender.py's StreamSender -- TCP client
// that sends camera frames TO UE's RMSS streaming server. Only the two
// send paths rammp_ui_bridge_node.py actually uses are ported:
// send_image (MessageType::kImageData) and send_depth_uint16
// (MessageType::kFrameDepth, unit="mm") -- see that file's
// _forward_scene_camera_color/_depth/_forward_engine_overlay_color.
#ifndef RAMMP_UI_BRIDGE_STREAMING__STREAM_SENDER_HPP_
#define RAMMP_UI_BRIDGE_STREAMING__STREAM_SENDER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rammp_ui_bridge_streaming/rmss_protocol.hpp"

namespace rammp_ui_bridge_streaming
{

/// Optional per-frame stream metadata, mirrors send_image/send_depth_uint16's
/// keyword args in sender.py.
struct StreamMeta
{
  std::optional<std::string> group;
  std::optional<std::string> role;
  std::optional<std::string> stream_id;
  std::optional<std::string> name;
};

class StreamSender
{
public:
  /// queue_size: max queued frames before the oldest is dropped (newest
  /// frame always wins) -- same semantics/default (4) as sender.py.
  explicit StreamSender(std::string host, uint16_t port, std::size_t queue_size = 4);
  ~StreamSender();

  StreamSender(const StreamSender &) = delete;
  StreamSender & operator=(const StreamSender &) = delete;

  /// Throws std::system_error on failure, same as Python's connect().
  void connect(int timeout_sec = 5);
  void disconnect();
  [[nodiscard]] bool is_connected() const;

  /// Raw image frame, message_type = IMAGE_DATA. fmt is the image's
  /// pixel-format string (e.g. ROS encoding, or "bgra8").
  void send_image(
    uint16_t channel, const uint8_t * data, std::size_t data_len, uint32_t width,
    uint32_t height, const std::string & fmt, const StreamMeta & meta = {});

  /// uint16 depth frame (native camera mm, no conversion), message_type =
  /// FRAME_DEPTH, fmt="16uc1", unit="mm" -- mirrors send_depth_uint16.
  void send_depth_uint16(
    uint16_t channel, const uint8_t * data, std::size_t data_len, uint32_t width,
    uint32_t height, const StreamMeta & meta = {});

private:
  void send_worker();
  void enqueue(StreamMessage msg);
  uint32_t next_seq(uint16_t channel);

  std::string host_;
  uint16_t port_;
  std::size_t queue_size_;

  mutable std::mutex sock_mutex_;
  int sock_fd_{-1};

  std::mutex seq_mutex_;
  std::map<uint16_t, uint32_t> seq_by_channel_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<StreamMessage> queue_;
  std::atomic<bool> shutdown_{true};
  std::thread worker_;
};

}  // namespace rammp_ui_bridge_streaming

#endif  // RAMMP_UI_BRIDGE_STREAMING__STREAM_SENDER_HPP_
