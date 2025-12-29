from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_obstacle_detector',
            executable='obstacle_detector_node_exe',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('input_scan', '/scan')]
        )
    ])