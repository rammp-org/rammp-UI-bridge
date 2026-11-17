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
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

// Quaternion -> Euler(roll,pitch,yaw) in degrees, ZYX extraction order --
// hand-ported from ui_bridge_node.py's _quat_to_euler_zyx_deg (itself a
// scipy-free port of the Gen1 prototype bridge's rotation_matrix_to_euler_zyx).
// Must stay numerically identical between the Python and C++ sides.
struct EulerDeg
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

EulerDeg quat_to_euler_zyx_deg(double x, double y, double z, double w)
{
  // Quaternion -> 3x3 rotation matrix (only the entries ZYX extraction
  // actually needs).
  const double r00 = 1.0 - 2.0 * (y * y + z * z);
  const double r01 = 2.0 * (x * y - z * w);
  const double r10 = 2.0 * (x * y + z * w);
  const double r11 = 1.0 - 2.0 * (x * x + z * z);
  const double r20 = 2.0 * (x * z - y * w);
  const double r21 = 2.0 * (y * z + x * w);
  const double r22 = 1.0 - 2.0 * (x * x + y * y);

  constexpr double kEpsilon = 1e-8;
  const double sy = -r20;
  EulerDeg euler;
  if (std::abs(sy) >= 1.0 - kEpsilon) {
    // Gimbal lock: pitch is +/-90 degrees, roll/yaw become coupled -- fold
    // the remaining rotation into yaw, roll fixed to 0 (same convention as
    // the Python/Gen1 reference implementations).
    const double pitch = std::copysign(M_PI / 2.0, sy);
    const double yaw = std::atan2(-r01, r11);
    euler.roll = 0.0;
    euler.pitch = pitch * 180.0 / M_PI;
    euler.yaw = yaw * 180.0 / M_PI;
  } else {
    const double pitch = std::asin(sy);
    const double roll = std::atan2(r21, r22);
    const double yaw = std::atan2(r10, r00);
    euler.roll = roll * 180.0 / M_PI;
    euler.pitch = pitch * 180.0 / M_PI;
    euler.yaw = yaw * 180.0 / M_PI;
  }
  return euler;
}

// TransformStamped -> UE-space extrinsics: meters->centimeters, Y position
// negated, pitch/yaw negated (UE's Y axis is flipped vs ROS; roll is NOT
// negated) -- same sign convention as ui_bridge_node.py's
// _tf_to_ue_extrinsics / the Gen1 prototype bridge's _tf_to_ue_extrinsics.
// Must stay identical between the Python and C++ sides.
StreamMeta::Extrinsics tf_to_ue_extrinsics(
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform.translation;
  const auto & q = transform.transform.rotation;
  const EulerDeg euler = quat_to_euler_zyx_deg(q.x, q.y, q.z, q.w);

  StreamMeta::Extrinsics extrinsics;
  extrinsics.x = t.x * 100.0;
  extrinsics.y = -(t.y * 100.0);
  extrinsics.z = t.z * 100.0;
  extrinsics.roll = euler.roll;
  extrinsics.pitch = -euler.pitch;
  extrinsics.yaw = -euler.yaw;
  extrinsics.space = "relative";
  return extrinsics;
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
    this->declare_parameter<std::string>(
      "scene_camera_depth_camera_info_topic", "/scene_camera/depth/camera_info");
    this->declare_parameter<std::string>("tf_base_frame", "base_link");
    this->declare_parameter<std::string>("scene_camera_tf_frame", "scene_camera");
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

    tf_base_frame_ = this->get_parameter("tf_base_frame").as_string();
    scene_camera_tf_frame_ = this->get_parameter("scene_camera_tf_frame").as_string();
    // Zero-ROS-node-argument constructor: creates its own internal node +
    // dedicated executor thread, independent of this node's own
    // single-threaded rclcpp::spin() in main().
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    scene_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("scene_camera_color_topic").as_string(), qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_scene_color(msg);});
    scene_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("scene_camera_depth_topic").as_string(), qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_scene_depth(msg);});
    scene_depth_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      this->get_parameter("scene_camera_depth_camera_info_topic").as_string(), qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        latest_scene_depth_camera_info_ = msg;
      });
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
    // §7: attach this depth frame's own intrinsics (from its CameraInfo)
    // and the scene camera's extrinsics (TF pose relative to tf_base_frame_)
    // so UE can reproject depth pixels into world space. Both are
    // best-effort -- a frame is still forwarded (bare depth, no 3D
    // placement) if either hasn't resolved yet.
    std::optional<StreamMeta::Intrinsics> intrinsics;
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      if (latest_scene_depth_camera_info_) {
        const auto & k = latest_scene_depth_camera_info_->k;
        intrinsics = StreamMeta::Intrinsics{k[0], k[4], k[2], k[5]};
      }
    }
    const std::optional<StreamMeta::Extrinsics> transform = lookup_scene_camera_extrinsics();

    try {
      if (is_uint16_depth_encoding(msg->encoding)) {
        StreamMeta meta;
        meta.group = kSceneGroup;
        meta.role = "depth";
        meta.stream_id = kSceneDepthStreamId;
        meta.name = "Scene camera depth";
        meta.intrinsics = intrinsics;
        meta.transform = transform;
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
        meta.intrinsics = intrinsics;
        meta.transform = transform;
        sender_->send_image(
          kSceneDepthChannel, msg->data.data(), msg->data.size(), msg->width, msg->height,
          msg->encoding, meta);
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        this->get_logger(), "failed to forward scene camera depth frame to UE: %s", exc.what());
    }
  }

  /// Looks up the scene camera's current pose relative to tf_base_frame_,
  /// converted to the UE-metadata shape via tf_to_ue_extrinsics(). Returns
  /// the last successfully-resolved pose (or nullopt if there's never been
  /// one) rather than nullopt on a transient lookup failure -- e.g. the
  /// camera's static transform not having been broadcast yet right after
  /// startup -- same "keep stale rather than drop" convention the Gen1
  /// prototype bridge's own camera-extrinsics lookup used.
  std::optional<StreamMeta::Extrinsics> lookup_scene_camera_extrinsics()
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        tf_base_frame_, scene_camera_tf_frame_, tf2::TimePointZero,
        tf2::durationFromSec(1.0));
      std::lock_guard<std::mutex> lock(extrinsics_mutex_);
      last_scene_extrinsics_ = tf_to_ue_extrinsics(transform);
      return last_scene_extrinsics_;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF2 lookup failed for '%s' -> '%s': %s",
        tf_base_frame_.c_str(), scene_camera_tf_frame_.c_str(), exc.what());
      std::lock_guard<std::mutex> lock(extrinsics_mutex_);
      return last_scene_extrinsics_;
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
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr scene_depth_camera_info_sub_;
  rclcpp::Subscription<rammp_ui_bridge_interfaces::msg::EngineOverlayImage>::SharedPtr
    overlay_sub_;

  // §7 depth intrinsics/extrinsics state.
  std::string tf_base_frame_;
  std::string scene_camera_tf_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex camera_info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_scene_depth_camera_info_;
  std::mutex extrinsics_mutex_;
  std::optional<StreamMeta::Extrinsics> last_scene_extrinsics_;

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
