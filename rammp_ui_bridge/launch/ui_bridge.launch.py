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
                        "ue_preset": LaunchConfiguration("ue_preset"),
                        "orchestrator_ns": LaunchConfiguration("orchestrator_ns"),
                        "world_state_poll_period": LaunchConfiguration(
                            "world_state_poll_period"
                        ),
                        "detected_objects_poll_period": LaunchConfiguration(
                            "detected_objects_poll_period"
                        ),
                    }
                ],
            ),
        ]
    )
