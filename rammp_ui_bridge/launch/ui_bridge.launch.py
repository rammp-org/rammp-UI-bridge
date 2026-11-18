from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ue_host",
                default_value="127.0.0.1",
                description="IP address of the UE instance running the Remote Control preset",
            ),
            DeclareLaunchArgument(
                "ue_ws_port",
                default_value="30020",
                description="UE Remote Control WebSocket API port",
            ),
            DeclareLaunchArgument(
                "ue_stream_port",
                default_value="30030",
                description="UE RMSS streaming server port (§5.2.A camera base layer)",
            ),
            DeclareLaunchArgument(
                "ue_preset",
                default_value="RAMMP",
                description="UE Remote Control preset name to register against",
            ),
            DeclareLaunchArgument(
                "ue_log_level",
                default_value="INFO",
                description=(
                    "Log level for the UE WebSocket traffic/streaming-connection "
                    "loggers (unreal_remote_websocket, ramms_tools.streaming); "
                    "set to DEBUG to see every WS send/receive"
                ),
            ),
            DeclareLaunchArgument(
                "scene_camera_color_topic",
                default_value="/scene_camera/color/image_raw",
                description="Scene camera's raw color image topic (§3.2)",
            ),
            DeclareLaunchArgument(
                "scene_camera_depth_topic",
                default_value="/scene_camera/depth/image_rect_raw",
                description=(
                    "Scene camera's raw depth image topic (§3.2) -- depends on the "
                    "scene_camera alternative selected in the robot's sheppy profile "
                    "(e.g. Orbbec Gemini 336L: depth/image_raw; OAK-D Pro W: "
                    "depth/image_rect_raw)"
                ),
            ),
            DeclareLaunchArgument(
                "scene_camera_depth_camera_info_topic",
                default_value="/scene_camera/depth/camera_info",
                description=(
                    "Scene camera depth's CameraInfo topic -- intrinsics "
                    "(fx/fy/cx/cy) attached to every depth frame sent to UE (§7)"
                ),
            ),
            DeclareLaunchArgument(
                "tf_base_frame",
                default_value="base_link",
                description="TF root frame the scene camera's extrinsics are resolved relative to (§7)",
            ),
            DeclareLaunchArgument(
                "scene_camera_tf_frame",
                default_value="scene_camera",
                description="Scene camera's own static TF frame, looked up for its extrinsics (§7)",
            ),
            DeclareLaunchArgument(
                "engine_overlay_topic",
                default_value="/engine_overlay/image_raw",
                description=(
                    "Single shared topic every engine publishes its own camera to "
                    "(§3.3.1 overlay layer) -- not per-engine, since only one engine "
                    "can hold execution resources at a time"
                ),
            ),
            DeclareLaunchArgument(
                "use_python_camera_streaming",
                default_value="true",
                description=(
                    "Set false when a separate rammp_ui_bridge_streaming "
                    "(camera_stream_node, C++) is running instead, to avoid "
                    "forwarding every frame to UE twice"
                ),
            ),
            DeclareLaunchArgument(
                "arm_joint_states_forward_rate_hz",
                default_value="2.0",
                description=(
                    "Max rate (Hz) SetJoints is actually sent to UE, "
                    "independent of /joint_states'/'base_joint_states_topic's own "
                    "publish rate -- avoids flooding the Remote Control WebSocket/log "
                    "with every driver tick. 0 or negative disables throttling "
                    "(forwards every message)."
                ),
            ),
            DeclareLaunchArgument(
                "base_joint_states_topic",
                default_value="/base/joint_states",
                description=(
                    "Chair/base driver's own JointState topic (foot control, "
                    "rotation, left/right drive, carriage, wheel) -- combined with "
                    "arm_joint_states_topic into one SetJoints call (§3.4)"
                ),
            ),
            Node(
                package="rammp_ui_bridge",
                executable="ui_bridge_node",
                name="ui_bridge_node",
                output="screen",
                emulate_tty=True,
                respawn=False,
                parameters=[
                    {
                        "ue_host": LaunchConfiguration("ue_host"),
                        "ue_ws_port": LaunchConfiguration("ue_ws_port"),
                        "ue_stream_port": LaunchConfiguration("ue_stream_port"),
                        "ue_preset": LaunchConfiguration("ue_preset"),
                        "ue_log_level": LaunchConfiguration("ue_log_level"),
                        "scene_camera_color_topic": LaunchConfiguration(
                            "scene_camera_color_topic"
                        ),
                        "scene_camera_depth_topic": LaunchConfiguration(
                            "scene_camera_depth_topic"
                        ),
                        "scene_camera_depth_camera_info_topic": LaunchConfiguration(
                            "scene_camera_depth_camera_info_topic"
                        ),
                        "tf_base_frame": LaunchConfiguration("tf_base_frame"),
                        "scene_camera_tf_frame": LaunchConfiguration(
                            "scene_camera_tf_frame"
                        ),
                        "engine_overlay_topic": LaunchConfiguration(
                            "engine_overlay_topic"
                        ),
                        "use_python_camera_streaming": LaunchConfiguration(
                            "use_python_camera_streaming"
                        ),
                        "arm_joint_states_forward_rate_hz": LaunchConfiguration(
                            "arm_joint_states_forward_rate_hz"
                        ),
                        "base_joint_states_topic": LaunchConfiguration(
                            "base_joint_states_topic"
                        ),
                    }
                ],
            ),
        ]
    )
