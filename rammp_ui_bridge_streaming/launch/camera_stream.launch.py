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
                description="IP address of the UE instance running the RMSS streaming server",
            ),
            DeclareLaunchArgument(
                "ue_stream_port",
                default_value="30030",
                description="UE RMSS streaming server port (§5.2.A camera base layer)",
            ),
            DeclareLaunchArgument(
                "ue_stream_reconnect_period",
                default_value="3.0",
                description="Seconds between reconnect attempts while disconnected from UE",
            ),
            DeclareLaunchArgument(
                "scene_camera_color_topic",
                default_value="/scene_camera/color/image_raw",
                description="Scene camera's raw color image topic (§3.2)",
            ),
            DeclareLaunchArgument(
                "scene_camera_depth_topic",
                default_value="/scene_camera/depth/image_rect_raw",
                description="Scene camera's raw depth image topic (§3.2)",
            ),
            DeclareLaunchArgument(
                "validated_engine_overlay_topic",
                default_value="/ui_bridge_node/validated_engine_overlay",
                description=(
                    "ui_bridge_node's post-arbitration, token-stripped republish of "
                    "EngineOverlayImage frames (§3.3.1) -- must match that node's own "
                    "validated_engine_overlay_topic parameter"
                ),
            ),
            DeclareLaunchArgument(
                "use_validated_overlay_topic",
                default_value="true",
                description=(
                    "True (default): subscribe to validated_engine_overlay_topic, same "
                    "as today. False: validate EngineOverlayImage frames in this node "
                    "itself via engine_overlay_topic + ui_control_token_topic instead "
                    "(see NODE_INTERFACE.md §5.2.A)"
                ),
            ),
            DeclareLaunchArgument(
                "engine_overlay_topic",
                default_value="/engine_overlay/image_raw",
                description=(
                    "Raw (pre-arbitration) EngineOverlayImage topic -- only subscribed "
                    "to when use_validated_overlay_topic is false; must match "
                    "ui_bridge_node's own engine_overlay_topic parameter"
                ),
            ),
            DeclareLaunchArgument(
                "ui_control_token_topic",
                default_value="/ui_bridge_node/ui_control/token",
                description=(
                    "ui_bridge_node's latched UiControlToken publisher -- only "
                    "subscribed to when use_validated_overlay_topic is false"
                ),
            ),
            Node(
                package="rammp_ui_bridge_streaming",
                executable="camera_stream_node",
                name="camera_stream_node",
                output="screen",
                emulate_tty=True,
                respawn=False,
                parameters=[
                    {
                        "ue_host": LaunchConfiguration("ue_host"),
                        "ue_stream_port": LaunchConfiguration("ue_stream_port"),
                        "ue_stream_reconnect_period": LaunchConfiguration(
                            "ue_stream_reconnect_period"
                        ),
                        "scene_camera_color_topic": LaunchConfiguration(
                            "scene_camera_color_topic"
                        ),
                        "scene_camera_depth_topic": LaunchConfiguration(
                            "scene_camera_depth_topic"
                        ),
                        "validated_engine_overlay_topic": LaunchConfiguration(
                            "validated_engine_overlay_topic"
                        ),
                        "use_validated_overlay_topic": LaunchConfiguration(
                            "use_validated_overlay_topic"
                        ),
                        "engine_overlay_topic": LaunchConfiguration("engine_overlay_topic"),
                        "ui_control_token_topic": LaunchConfiguration(
                            "ui_control_token_topic"
                        ),
                    }
                ],
            ),
        ]
    )
