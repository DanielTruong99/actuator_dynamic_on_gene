from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_controller',
            executable='joint_controller',
            name='joint_controller',
            output='screen',
            # parameters=[{'use_sim_time': False}]
        )
    ])