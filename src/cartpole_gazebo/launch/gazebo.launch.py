import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = [PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])]

    cartpole_gz_pkg_path = FindPackageShare('cartpole_gazebo')
    gz_args = [PathJoinSubstitution([cartpole_gz_pkg_path, 'worlds', 'cartpole_world.sdf']), ' -r']
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    cartpole_desc_pkg_path = FindPackageShare('cartpole_description')
    robot_description_content = Command(['xacro ', PathJoinSubstitution([cartpole_desc_pkg_path, 'urdf', 'cartpole.urdf.xacro'])])
    robot_description = {'robot_description': robot_description_content}

    cartpole_ctrl_pkg_path = FindPackageShare('cartpole_control')
    robot_controllers = PathJoinSubstitution([cartpole_ctrl_pkg_path, 'config', 'cartpole_controllers.yaml'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', robot_description, 
            '-name', 'cartpole', 
            '-allow_renaming', 'true'
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    cart_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'cart_effort_controller',
            '--param-file', robot_controllers,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': gz_args,
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[cart_effort_controller_spawner],
            )
        ),
        bridge,
        robot_state_publisher,
        gz_spawn_entity,
        # rviz,

        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])