# Hexapod_controller-and-simulation

A six-legged hexapod robot simulation built using ROS 2 Jazzy, ros2_control, and Gazebo Harmonic.
The project focuses on joint-level actuation, sensor integration, and testing custom locomotion algorithms for a 3-DOF-per-leg walking robot.

This project is based on Ros2 Jazzy and Gazebo Harmonic 

Also this is a project by a newbie so there might be some issues. Please feel free to use this project


## Features
- 18-DOF Hexapod (coxa, femur, tibia joints)
- ROS 2 Jazzy as the middleware
- Gazebo Harmonic for physics & visualization
- ros2_control integration using position/velocity controllers
- URDF + Xacro robot description
- IMU, LiDAR, and touch sensors (WIP)
- Custom walking controller node
## Getting Started
```bash
git clone https://github.com/Not-NeoN-sup/Hexapod_controller-and-simulation
cd hexapod_ws
```
```bash
#install dependencies
rosdep install --from-paths src -y --ignore-src
```
```bash
colcon build
source install/setup.bash

#to only launch gazebo and view urdf
ros2 launch hexapod_ws gazebo.launch.py

#to see the movement 
ros2 launch hexapod_ws hexapod_movement.launch.py
```
## Roadmap
- improve the movement
- implement gait patterns (tripod, ripple, wave)
- Obstacle-aware walking
- Complete sensor integration
- more will be added later