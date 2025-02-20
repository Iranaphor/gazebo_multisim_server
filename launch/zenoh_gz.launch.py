from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Zenoh Bridge with Namespace
        ExecuteProcess(
            cmd=["zenoh-bridge-ros2dds", "--connect", "tcp/172.17.0.1:7447"],
            output="screen",
            additional_env={"ROS_NAMESPACE": "bob"}  # Apply namespace
        ),

        # ROS2 Remapping Node: /bob/spawn → /spawn
        Node(
            package="topic_tools",  # Ensure topic_tools is installed
            executable="relay",
            name="spawn_relay",
            remappings=[("/bob/spawn", "/spawn")],  # Forward /bob/spawn → /spawn
            output="screen"
        )
    ])
