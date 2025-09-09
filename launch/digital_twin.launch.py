from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # Launch Gazebo World
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'gazebo_worlds/multi_robot_world.sdf'],
            output='screen'
        ),

        # Launch Digital Twin Core
        Node(
            package='digital_twin_core',
            executable='twin_sync_node',
            name='twin_sync'
        ),
        Node(
            package='digital_twin_core',
            executable='sensor_stream_node',
            name='sensor_stream'
        ),
        Node(
            package='digital_twin_core',
            executable='anomaly_detection_node',
            name='anomaly_detection'
        ),

        # Launch Multi-Robot Orchestrator
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

        # Launch RL Navigation
        Node(
            package='rl_navigation',
            executable='rl_navigation_node',
            name='rl_navigation'
        )
    ])
