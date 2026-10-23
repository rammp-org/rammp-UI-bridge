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
                "orchestrator_ns",
                default_value="/rammp_orchestrator",
                description="Node namespace the Orchestrator's services/action are advertised under",
            ),
            DeclareLaunchArgument(
                "world_state_poll_period",
                default_value="0.5",
                description="Seconds between ~/get_world_state polls",
            ),
            DeclareLaunchArgument(
                "detected_objects_poll_period",
                default_value="0.5",
                description="Seconds between ~/get_detected_objects polls",
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
                        "orchestrator_ns": LaunchConfiguration("orchestrator_ns"),
                        "world_state_poll_period": LaunchConfiguration(
                            "world_state_poll_period"
                        ),
                        "detected_objects_poll_period": LaunchConfiguration(
                            "detected_objects_poll_period"
                        ),
                        "scene_camera_color_topic": LaunchConfiguration(
                            "scene_camera_color_topic"
                        ),
                        "scene_camera_depth_topic": LaunchConfiguration(
                            "scene_camera_depth_topic"
                        ),
                        "engine_overlay_topic": LaunchConfiguration(
                            "engine_overlay_topic"
                        ),
                    }
                ],
            ),
        ]
    )
