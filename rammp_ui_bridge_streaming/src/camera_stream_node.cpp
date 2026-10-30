// camera_stream_node: C++ RMSS camera-streaming node, an optional
// replacement for rammp_ui_bridge's ui_bridge_node's own Python-side
// forwarding of the scene camera base layer and the engine-overlay layer
// (docs/NODE_INTERFACE.md §5.2.A). See that package's README/ENGINE_* docs
// for the full channel/stream_id contract this mirrors byte-for-byte.
//
// This node intentionally knows nothing about ui_control arbitration --
// ui_bridge_node remains the arbitration server and republishes only
// already-validated overlay frames (plain sensor_msgs/Image, no token) on
// validated_engine_overlay_topic, which this node subscribes to directly.
// Run ui_bridge_node with use_python_camera_streaming:=false alongside this
// node to avoid double-forwarding every frame to UE.
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

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

const char kSceneColorStreamId[] = "scene/color";
const char kSceneDepthStreamId[] = "scene/depth";
const char kOverlayColorStreamId[] = "engine/overlay";
const char kSceneGroup[] = "scene";
const char kOverlayGroup[] = "overlay";

bool is_uint16_depth_encoding(const std::string & encoding)
{
  return encoding == "16UC1" || encoding == "mono16";
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
    this->declare_parameter<std::string>(
      "validated_engine_overlay_topic", "/ui_bridge_node/validated_engine_overlay");

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
    overlay_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("validated_engine_overlay_topic").as_string(), qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_overlay(msg);});

    reconnect_shutdown_ = false;
    reconnect_thread_ = std::thread(&CameraStreamNode::reconnect_loop, this);

    RCLCPP_INFO(
      this->get_logger(),
      "camera_stream_node ready; forwarding scene camera color='%s' depth='%s' "
      "validated engine overlay='%s' -> UE %s:%d",
      this->get_parameter("scene_camera_color_topic").as_string().c_str(),
      this->get_parameter("scene_camera_depth_topic").as_string().c_str(),
      this->get_parameter("validated_engine_overlay_topic").as_string().c_str(),
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

  void on_overlay(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!sender_->is_connected()) {
      return;
    }
    StreamMeta meta;
    meta.group = kOverlayGroup;
    meta.role = "color";
    meta.stream_id = kOverlayColorStreamId;
    meta.name = "Engine overlay camera";
    try {
      sender_->send_image(
        kOverlayColorChannel, msg->data.data(), msg->data.size(), msg->width, msg->height,
        msg->encoding, meta);
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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr overlay_sub_;
};

}  // namespace rammp_ui_bridge_streaming

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rammp_ui_bridge_streaming::CameraStreamNode>());
  rclcpp::shutdown();
  return 0;
}
