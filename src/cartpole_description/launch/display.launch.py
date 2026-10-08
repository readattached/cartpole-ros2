import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    robot_description_content = Command(['xacro ',
        PathJoinSubstitution([FindPackageShare('cartpole_description'), 'urdf', 'cartpole.urdf.xacro'])])
        

    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('cartpole_control'),
        'config',
        'cartpole_controllers.yaml',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])