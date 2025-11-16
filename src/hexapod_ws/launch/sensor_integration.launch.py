#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription , TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_ws')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    controller_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hexapod_movement.launch.py')
        )
    )
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_sensor_bridge',
        arguments=[
            # ✅ Time synchronization
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # ✅ Sensors
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',

            # ✅ Contact sensors (Gazebo → ROS)
            '/leg_1_foot_contact@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts',
            '/leg_2_foot_contact@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts',
            '/leg_3_foot_contact@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts',
            '/leg_4_foot_contact@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts',
            '/leg_5_foot_contact@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts',
            '/leg_6_foot_contact@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts',
        ],
        output='screen'
    )


    return LaunchDescription([
        controller_launch,
        sensor_bridge
    ])