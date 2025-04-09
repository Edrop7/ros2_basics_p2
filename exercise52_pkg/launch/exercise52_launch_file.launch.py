from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exercise52_pkg',
            executable='exercise52_node',
            output='screen'),
    ])