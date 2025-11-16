#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription , TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_ws')

    # Gazebo launch (you can change to your custom launch if needed)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )

    # Main hexapod gait node
    hexapod_node = Node(
        package='hexapod_ws',
        executable='hexapod_runner.py',
        name='hexapod_walker',
        output='screen'
    )
    
    teleop_node = Node(
        package='hexapod_ws',
        executable='teleop_hexapod.py',
        name='teleop',
        output='screen',
        prefix=['xterm ', '-hold ', '-e']
    )

    delayed_teleop_node = TimerAction(
        period=10.0,
        actions=[teleop_node]
    )

    return LaunchDescription([
        gazebo_launch,
        hexapod_node,
        delayed_teleop_node
    ])
