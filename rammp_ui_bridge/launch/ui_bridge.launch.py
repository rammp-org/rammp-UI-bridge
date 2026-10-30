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
                "validated_engine_overlay_topic",
                default_value="~/validated_engine_overlay",
                description=(
                    "Post-arbitration, token-stripped republish of EngineOverlayImage "
                    "frames -- rammp_ui_bridge_streaming's camera_stream_node "
                    "subscribes to this instead of the raw, token-gated topic"
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
                        "scene_camera_color_topic": LaunchConfiguration(
                            "scene_camera_color_topic"
                        ),
                        "scene_camera_depth_topic": LaunchConfiguration(
                            "scene_camera_depth_topic"
                        ),
                        "engine_overlay_topic": LaunchConfiguration(
                            "engine_overlay_topic"
                        ),
                        "use_python_camera_streaming": LaunchConfiguration(
                            "use_python_camera_streaming"
                        ),
                        "validated_engine_overlay_topic": LaunchConfiguration(
                            "validated_engine_overlay_topic"
                        ),
                    }
                ],
            ),
        ]
    )
