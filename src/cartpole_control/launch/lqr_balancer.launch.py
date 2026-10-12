from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartpole_control',
            executable='lqr_balancer',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('cartpole_control'), 'config', 'lqr_balancer.yaml']),
            ]
        )
    ])
