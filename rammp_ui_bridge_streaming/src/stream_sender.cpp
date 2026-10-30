#include "rammp_ui_bridge_streaming/stream_sender.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <system_error>

#include <nlohmann/json.hpp>

namespace rammp_ui_bridge_streaming
{

namespace
{
constexpr int kSndBufBytes = 2 * 1024 * 1024;
}  // namespace

StreamSender::StreamSender(std::string host, uint16_t port, std::size_t queue_size)
: host_(std::move(host)), port_(port), queue_size_(queue_size)
{
}

StreamSender::~StreamSender()
{
  disconnect();
}

void StreamSender::connect(int timeout_sec)
{
  std::lock_guard<std::mutex> lock(sock_mutex_);
  if (sock_fd_ >= 0) {
    throw std::runtime_error("Already connected");
  }

  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    throw std::system_error(errno, std::generic_category(), "socket() failed");
  }

  struct timeval tv{};
  tv.tv_sec = timeout_sec;
  tv.tv_usec = 0;
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
    ::close(fd);
    throw std::runtime_error("invalid host address: " + host_);
  }

  if (::connect(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0) {
    int err = errno;
    ::close(fd);
    throw std::system_error(err, std::generic_category(), "connect() failed");
  }

  // Clear the connect-timeout back to blocking (matches Python's
  // sock.settimeout(None) after connect()).
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  int sndbuf = kSndBufBytes;
  setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  sock_fd_ = fd;
  shutdown_.store(false);
  worker_ = std::thread(&StreamSender::send_worker, this);
}

bool StreamSender::is_connected() const
{
  std::lock_guard<std::mutex> lock(sock_mutex_);
  return sock_fd_ >= 0;
}

void StreamSender::disconnect()
{
  shutdown_.store(true);
  queue_cv_.notify_all();

  int fd;
  {
    std::lock_guard<std::mutex> lock(sock_mutex_);
    fd = sock_fd_;
    sock_fd_ = -1;
  }
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }

  if (worker_.joinable()) {
    worker_.join();
  }

  std::lock_guard<std::mutex> lock(queue_mutex_);
  queue_.clear();
}

uint32_t StreamSender::next_seq(uint16_t channel)
{
  std::lock_guard<std::mutex> lock(seq_mutex_);
  auto & seq = seq_by_channel_[channel];
  return seq++;
}

void StreamSender::enqueue(StreamMessage msg)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (queue_.size() >= queue_size_) {
    // Drop the oldest queued frame -- newest frame always wins, same
    // policy as sender.py's Queue.Full eviction.
    queue_.pop_front();
  }
  queue_.push_back(std::move(msg));
  queue_cv_.notify_one();
}

void StreamSender::send_worker()
{
  while (!shutdown_.load()) {
    StreamMessage msg;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_for(
        lock, std::chrono::milliseconds(500),
        [this] {return shutdown_.load() || !queue_.empty();});
      if (shutdown_.load()) {
        return;
      }
      if (queue_.empty()) {
        continue;
      }
      msg = std::move(queue_.front());
      queue_.pop_front();
    }

    int fd;
    {
      std::lock_guard<std::mutex> lock(sock_mutex_);
      fd = sock_fd_;
    }
    if (fd < 0) {
      continue;
    }

    std::vector<uint8_t> bytes = msg.serialize();
    std::size_t sent = 0;
    bool failed = false;
    while (sent < bytes.size()) {
      ssize_t n = ::send(fd, bytes.data() + sent, bytes.size() - sent, 0);
      if (n <= 0) {
        failed = true;
        break;
      }
      sent += static_cast<std::size_t>(n);
    }
    if (failed) {
      // Transition to disconnected, same as sender.py catching OSError in
      // its worker and clearing self._sock.
      std::lock_guard<std::mutex> lock(sock_mutex_);
      if (sock_fd_ == fd) {
        ::close(sock_fd_);
        sock_fd_ = -1;
      }
      return;
    }
  }
}

namespace
{

void apply_meta(nlohmann::json & j, const StreamMeta & meta)
{
  if (meta.group) {
    j["group"] = *meta.group;
  }
  if (meta.role) {
    j["role"] = *meta.role;
  }
  if (meta.stream_id) {
    j["stream_id"] = *meta.stream_id;
  }
  if (meta.name) {
    j["name"] = *meta.name;
  }
}

}  // namespace

void StreamSender::send_image(
  uint16_t channel, const uint8_t * data, std::size_t data_len, uint32_t width,
  uint32_t height, const std::string & fmt, const StreamMeta & meta)
{
  nlohmann::json j;
  j["w"] = width;
  j["h"] = height;
  j["fmt"] = fmt;
  apply_meta(j, meta);

  StreamMessage msg;
  msg.header.message_type = MessageType::kImageData;
  msg.header.channel_id = channel;
  msg.header.sequence_num = next_seq(channel);
  msg.header.timestamp = StreamHeader::now_timestamp();
  msg.metadata = j.dump();
  msg.payload.assign(data, data + data_len);

  enqueue(std::move(msg));
}

void StreamSender::send_depth_uint16(
  uint16_t channel, const uint8_t * data, std::size_t data_len, uint32_t width,
  uint32_t height, const StreamMeta & meta)
{
  nlohmann::json j;
  j["w"] = width;
  j["h"] = height;
  j["fmt"] = "16uc1";
  j["unit"] = "mm";
  j["role"] = "depth";
  apply_meta(j, meta);  // meta.role, if set, overrides the default above.

  StreamMessage msg;
  msg.header.message_type = MessageType::kFrameDepth;
  msg.header.channel_id = channel;
  msg.header.sequence_num = next_seq(channel);
  msg.header.timestamp = StreamHeader::now_timestamp();
  msg.metadata = j.dump();
  msg.payload.assign(data, data + data_len);

  enqueue(std::move(msg));
}

}  // namespace rammp_ui_bridge_streaming
