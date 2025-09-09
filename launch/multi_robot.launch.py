from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi_robot_orchestrator',
            executable='fleet_manager_node',
            name='fleet_manager'
        ),
        Node(
            package='multi_robot_orchestrator',
            executable='task_scheduler_node',
            name='task_scheduler'
        ),
    ])
