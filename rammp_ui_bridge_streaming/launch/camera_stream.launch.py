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
                default_value="scene_link",
                description="Scene camera's own static TF frame, looked up for its extrinsics (§7)",
            ),
            DeclareLaunchArgument(
                "engine_overlay_topic",
                default_value="/engine_overlay/image_raw",
                description=(
                    "Raw shared engine-overlay camera topic (§3.3.1) -- this node "
                    "validates each frame's token itself via its own "
                    "acquire_control/release_control, independent of ui_bridge_node"
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
                    }
                ],
            ),
        ]
    )
