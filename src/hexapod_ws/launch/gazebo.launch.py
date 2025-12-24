#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare 
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription , TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare('hexapod_ws')
    urdf_path = PathJoinSubstitution([pkg_share, 'description', 'hexapod.urdf.xacro'])
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'hexapod.sdf'])
    parameters_file = PathJoinSubstitution([pkg_share, 'config', 'parameters.yaml'])
    
    # 1. Launch Gazebo Harmonic using the official launch script
    # This is more robust than ExecuteProcess for Jazzy
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str) 
        }],
        output='screen'
    )

    # 3. Spawn the Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'hexapod', '-z', '0.5'],
        output='screen'
    )

    # 4. Spawners with TimerAction (Essential for stability)
    # This prevents "Service not available" errors by waiting for Gazebo to finish loading
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    hexapod_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_controller', '--param-file', parameters_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        # Wait 5 seconds for Gazebo to load before spawning controllers
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=8.0, actions=[hexapod_controller_spawner]),
    ])