#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Paths & Setup
    pkg_name = 'hexapod_ws'
    pkg_share = get_package_share_directory(pkg_name)
    
    urdf_path = os.path.join(pkg_share, 'description', 'hexapod.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'hexapod.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    params_file = os.path.join(pkg_share, 'config', 'parameters.yaml')

    # 2. Robot State Publisher (Processes Xacro -> URDF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }]
    )

    # 3. Gazebo Harmonic (Physics Engine)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [f'-r -v -v4 {world_path}'],'on_exit_shutdown': 'true'}.items()
    )

    # 4. Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'hexapod', '-z', '0.2'],
        output='screen'
    )

    # 5. Parameter Bridge (LiDAR, IMU, Clock, JointStates)
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}',
        ],
        output='screen',
    )

    # 6. Controllers (Timed for stability)
    # Joint State Broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Hexapod Controller
    hex_ctrl_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_controller', '--param-file', params_file],
        output='screen'
    )

    # 7. User Interfaces (Teleop & Brain)
    hexapod_brain = Node(
        package=pkg_name,
        executable='hexapod_runner.py',
        name='hexapod_walker',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. Assembly with Timers
    # We stagger the startup to allow Gazebo to fully initialize the Hardware Interface
    return LaunchDescription([
        rsp,        # Start Skeleton
        gazebo,     # Start Physics
        spawn_robot, # Start Body
        start_gazebo_ros_bridge_cmd,     # Start Comms
        
        # Wait 10s for Gazebo to parse URDF and start the Controller Manager
        TimerAction(period=5.0, actions=[jsb_spawner]),
        
        # Wait 15s before starting the main movement controller
        TimerAction(period=10.0, actions=[hex_ctrl_spawner]),
        
        # Finally start the Brain/Teleop
        TimerAction(period=15.0, actions=[hexapod_brain]),
    ])