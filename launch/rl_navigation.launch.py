from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rl_navigation',
            executable='rl_navigation_node',
            name='rl_navigation'
        )
    ])
