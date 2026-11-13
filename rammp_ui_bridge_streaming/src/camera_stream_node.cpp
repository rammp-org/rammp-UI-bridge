// camera_stream_node: C++ RMSS camera-streaming node, an independent
// replacement for rammp_ui_bridge's ui_bridge_node's own Python-side
// forwarding of the scene camera base layer and the engine-overlay layer
// (docs/NODE_INTERFACE.md §5.2.A). See that package's README/ENGINE_* docs
// for the full channel/stream_id contract this mirrors byte-for-byte.
//
// This node is its own independent arbitration *server* for the overlay
// camera -- it does NOT depend on ui_bridge_node at all: it subscribes
// directly to the raw engine_overlay_topic (rammp_ui_bridge_interfaces/
// EngineOverlayImage, token included) and hosts its own
// rammp_common_interfaces ~/acquire_control / ~/release_control services,
// exactly the same contract mock_arm_driver/mock_base_driver/ui_bridge_node
// already use for their own arbitrated resources. The Orchestrator pushes
// the same per-execution session token here as it pushes to ui_bridge_node's
// ui_control, so both nodes validate the identical token independently and
// in parallel -- this node has no dependency on ui_bridge_node's own
// ui_control grant or any validated/republished topic. Run ui_bridge_node
// with use_python_camera_streaming:=false alongside this node so neither
// forwards a frame to UE twice (that flag also stops ui_bridge_node from
// subscribing to any camera topic at all -- see its own docstring).
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rammp_common_interfaces/srv/acquire_control.hpp>
#include <rammp_common_interfaces/srv/release_control.hpp>
#include <rammp_ui_bridge_interfaces/msg/engine_overlay_image.hpp>

#include "rammp_ui_bridge_streaming/stream_sender.hpp"

namespace rammp_ui_bridge_streaming
{

namespace
{
// Same fixed RMSS channel numbers/stream_ids as ui_bridge_node.py -- see
// that file's _SCENE_COLOR_CHANNEL/_SCENE_DEPTH_CHANNEL/_OVERLAY_COLOR_CHANNEL.
constexpr uint16_t kSceneColorChannel = 0;
constexpr uint16_t kSceneDepthChannel = 1;
constexpr uint16_t kOverlayColorChannel = 2;

const char kSceneColorStreamId[] = "camera/scene/color";
const char kSceneDepthStreamId[] = "camera/scene/depth";
const char kOverlayColorStreamId[] = "engine/overlay";
const char kSceneGroup[] = "camera/scene";
const char kOverlayGroup[] = "overlay";

using TokenArray = std::array<uint8_t, 16>;
const TokenArray kZeroToken{};

bool is_uint16_depth_encoding(const std::string & encoding)
{
  return encoding == "16UC1" || encoding == "mono16";
}

TokenArray mint_token()
{
  // Not cryptographic-grade, just needs to be unguessable enough to tell
  // a stale publisher apart from the current grant -- same expectation as
  // os.urandom(16) on the Python side (ui_bridge_node.py's
  // _on_ui_acquire_control, resource_driver_node.py's TwistResourceDriverNode).
  static thread_local std::random_device rd;
  static thread_local std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dist(0, 255);
  TokenArray token{};
  for (auto & byte : token) {
    byte = static_cast<uint8_t>(dist(gen));
  }
  return token;
}

TokenArray to_array(const std::array<uint8_t, 16> & src)
{
  return src;
}

std::string token_hex_prefix(const TokenArray & token, size_t n = 4)
{
  static const char * kHex = "0123456789abcdef";
  std::string out;
  for (size_t i = 0; i < n && i < token.size(); ++i) {
    out.push_back(kHex[(token[i] >> 4) & 0xF]);
    out.push_back(kHex[token[i] & 0xF]);
  }
  return out;
}

}  // namespace

class CameraStreamNode : public rclcpp::Node
{
public:
  CameraStreamNode()
  : Node("camera_stream_node")
  {
    this->declare_parameter<std::string>("ue_host", "127.0.0.1");
    this->declare_parameter<int>("ue_stream_port", 30030);
    this->declare_parameter<double>("ue_stream_reconnect_period", 3.0);
    this->declare_parameter<std::string>(
      "scene_camera_color_topic", "/scene_camera/color/image_raw");
    this->declare_parameter<std::string>(
      "scene_camera_depth_topic", "/scene_camera/depth/image_rect_raw");
    // Raw, direct subscription -- same shared topic every engine with its
    // own camera publishes to and the one ui_bridge_node also subscribes to
    // (when use_python_camera_streaming is True). This node runs its own
    // independent token check (see on_acquire_control/on_overlay below), not
    // ui_bridge_node's validated republish.
    this->declare_parameter<std::string>(
      "engine_overlay_topic", "/engine_overlay/image_raw");

    const auto ue_host = this->get_parameter("ue_host").as_string();
    const auto ue_stream_port =
      static_cast<uint16_t>(this->get_parameter("ue_stream_port").as_int());
    reconnect_period_ = std::chrono::duration<double>(
      this->get_parameter("ue_stream_reconnect_period").as_double());

    sender_ = std::make_unique<StreamSender>(ue_host, ue_stream_port);

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    scene_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("scene_camera_color_topic").as_string(), qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_scene_color(msg);});
    scene_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("scene_camera_depth_topic").as_string(), qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_scene_depth(msg);});
    overlay_sub_ = this->create_subscription<rammp_ui_bridge_interfaces::msg::EngineOverlayImage>(
      this->get_parameter("engine_overlay_topic").as_string(), qos,
      [this](rammp_ui_bridge_interfaces::msg::EngineOverlayImage::ConstSharedPtr msg) {
        on_overlay(msg);
      });

    acquire_control_srv_ = this->create_service<rammp_common_interfaces::srv::AcquireControl>(
      "~/acquire_control",
      [this](
        const std::shared_ptr<rammp_common_interfaces::srv::AcquireControl::Request> request,
        std::shared_ptr<rammp_common_interfaces::srv::AcquireControl::Response> response) {
        on_acquire_control(request, response);
      });
    release_control_srv_ = this->create_service<rammp_common_interfaces::srv::ReleaseControl>(
      "~/release_control",
      [this](
        const std::shared_ptr<rammp_common_interfaces::srv::ReleaseControl::Request> request,
        std::shared_ptr<rammp_common_interfaces::srv::ReleaseControl::Response> response) {
        on_release_control(request, response);
      });

    reconnect_shutdown_ = false;
    reconnect_thread_ = std::thread(&CameraStreamNode::reconnect_loop, this);

    RCLCPP_INFO(
      this->get_logger(),
      "camera_stream_node ready; forwarding scene camera color='%s' depth='%s' "
      "engine overlay='%s' (own acquire_control/release_control arbitration) -> UE %s:%d",
      this->get_parameter("scene_camera_color_topic").as_string().c_str(),
      this->get_parameter("scene_camera_depth_topic").as_string().c_str(),
      this->get_parameter("engine_overlay_topic").as_string().c_str(),
      ue_host.c_str(), static_cast<int>(ue_stream_port));
  }

  ~CameraStreamNode() override
  {
    reconnect_shutdown_ = true;
    if (reconnect_thread_.joinable()) {
      reconnect_thread_.join();
    }
  }

private:
  // --------------------------------------------------------------------
  // Overlay-camera arbitration server (NODE_INTERFACE.md §3.3.1): this
  // node's own ~/acquire_control / ~/release_control, the same contract
  // mock_arm_driver/mock_base_driver/ui_bridge_node's own ui_control use.
  // SEIZE semantics -- a new acquire_control call always succeeds and
  // immediately invalidates whatever token was previously granted, no
  // explicit release needed first (see resource_driver_node.py's
  // TwistResourceDriverNode, the Python reference for this pattern).
  // --------------------------------------------------------------------
  void on_acquire_control(
    const std::shared_ptr<rammp_common_interfaces::srv::AcquireControl::Request> & request,
    const std::shared_ptr<rammp_common_interfaces::srv::AcquireControl::Response> & response)
  {
    TokenArray granted_token;
    bool minted_fresh = false;
    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      TokenArray requested_token = to_array(request->token);
      if (requested_token != kZeroToken) {
        // AcquireControl.srv's "pre-generated key" field -- the Orchestrator
        // shares one session token across every resource linked to an
        // execution rather than asking each to mint its own; honor it.
        token_ = requested_token;
      } else {
        token_ = mint_token();
        minted_fresh = true;
      }
      owner_id_ = request->owner_id;
      ++generation_;
      granted_token = token_;
      generation = generation_;
    }
    response->accepted = true;
    response->token = granted_token;
    response->generation = generation;
    response->message = "granted to '" + request->owner_id + "'";
    RCLCPP_INFO(
      this->get_logger(),
      "overlay control acquired by '%s' (%s token %s..., generation %lu)",
      request->owner_id.c_str(),
      minted_fresh ? "minted a new" : "using orchestrator-supplied",
      token_hex_prefix(granted_token).c_str(),
      static_cast<unsigned long>(generation));
  }

  void on_release_control(
    const std::shared_ptr<rammp_common_interfaces::srv::ReleaseControl::Request> & request,
    const std::shared_ptr<rammp_common_interfaces::srv::ReleaseControl::Response> & response)
  {
    std::string released_owner;
    bool released = false;
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      if (to_array(request->token) == token_ && !owner_id_.empty()) {
        released_owner = owner_id_;
        owner_id_.clear();
        token_ = kZeroToken;
        released = true;
      } else {
        ++rejected_count_;
      }
    }
    response->released = released;
    response->message = released ? "" : "token mismatch or already released";
    if (released) {
      RCLCPP_INFO(this->get_logger(), "overlay control released by '%s'", released_owner.c_str());
    }
  }

  void on_scene_color(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!sender_->is_connected()) {
      return;
    }
    StreamMeta meta;
    meta.group = kSceneGroup;
    meta.role = "color";
    meta.stream_id = kSceneColorStreamId;
    meta.name = "Scene camera";
    try {
      sender_->send_image(
        kSceneColorChannel, msg->data.data(), msg->data.size(), msg->width, msg->height,
        msg->encoding, meta);
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        this->get_logger(), "failed to forward scene camera color frame to UE: %s", exc.what());
    }
  }

  void on_scene_depth(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!sender_->is_connected()) {
      return;
    }
    try {
      if (is_uint16_depth_encoding(msg->encoding)) {
        StreamMeta meta;
        meta.group = kSceneGroup;
        meta.role = "depth";
        meta.stream_id = kSceneDepthStreamId;
        meta.name = "Scene camera depth";
        sender_->send_depth_uint16(
          kSceneDepthChannel, msg->data.data(), msg->data.size(), msg->width, msg->height, meta);
      } else {
        // Non-uint16-mm encodings (e.g. 32FC1 metres): passthrough as
        // generic image data, same as ui_bridge_node.py's else branch.
        StreamMeta meta;
        meta.group = kSceneGroup;
        meta.role = "depth";
        meta.stream_id = kSceneDepthStreamId;
        meta.name = "Scene camera depth";
        sender_->send_image(
          kSceneDepthChannel, msg->data.data(), msg->data.size(), msg->width, msg->height,
          msg->encoding, meta);
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        this->get_logger(), "failed to forward scene camera depth frame to UE: %s", exc.what());
    }
  }

  void on_overlay(
    const rammp_ui_bridge_interfaces::msg::EngineOverlayImage::ConstSharedPtr & msg)
  {
    bool granted;
    uint64_t rejected = 0;
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      granted = !owner_id_.empty() && to_array(msg->token) == token_;
      if (!granted) {
        rejected = ++rejected_count_;
      }
    }
    if (!granted) {
      if (rejected == 1 || rejected % 100 == 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "engine overlay frame dropped: token doesn't match the current overlay "
          "control grant (rejected #%lu)",
          static_cast<unsigned long>(rejected));
      }
      return;
    }

    if (!sender_->is_connected()) {
      return;
    }
    const auto & image = msg->image;
    StreamMeta meta;
    meta.group = kOverlayGroup;
    meta.role = "color";
    meta.stream_id = kOverlayColorStreamId;
    meta.name = "Engine overlay camera";
    try {
      sender_->send_image(
        kOverlayColorChannel, image.data.data(), image.data.size(), image.width, image.height,
        image.encoding, meta);
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        this->get_logger(), "failed to forward engine overlay frame to UE: %s", exc.what());
    }
  }

  void reconnect_loop()
  {
    while (!reconnect_shutdown_.load()) {
      if (!sender_->is_connected()) {
        try {
          sender_->connect();
          RCLCPP_INFO(this->get_logger(), "connected to UE streaming server");
        } catch (const std::exception & exc) {
          RCLCPP_DEBUG(
            this->get_logger(), "UE streaming server connect failed: %s", exc.what());
        }
      }
      // Wake up promptly on shutdown instead of sleeping the full period.
      auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(reconnect_period_);
      while (!reconnect_shutdown_.load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  std::unique_ptr<StreamSender> sender_;
  std::chrono::duration<double> reconnect_period_{3.0};
  std::atomic<bool> reconnect_shutdown_{true};
  std::thread reconnect_thread_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr scene_color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr scene_depth_sub_;
  rclcpp::Subscription<rammp_ui_bridge_interfaces::msg::EngineOverlayImage>::SharedPtr
    overlay_sub_;

  rclcpp::Service<rammp_common_interfaces::srv::AcquireControl>::SharedPtr acquire_control_srv_;
  rclcpp::Service<rammp_common_interfaces::srv::ReleaseControl>::SharedPtr release_control_srv_;

  // Overlay-control arbitration state, guarded by control_mutex_ -- mutated
  // from the acquire_control/release_control service callbacks and read
  // from the overlay subscription callback, all of which run on the same
  // single-threaded default executor (see main()) but kept behind a mutex
  // for robustness against any future multi-threaded executor change.
  std::mutex control_mutex_;
  TokenArray token_{};
  std::string owner_id_;
  uint64_t generation_{0};
  uint64_t rejected_count_{0};
};

}  // namespace rammp_ui_bridge_streaming

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rammp_ui_bridge_streaming::CameraStreamNode>());
  rclcpp::shutdown();
  return 0;
}
