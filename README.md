# CIIMA ROS 2 Omnidirectional Mobile Robot Project

## Overview

This project contains the ROS 2 implementation of a mobile robot
with two active steerable OmniDrive modules.

It includes:
- Custom ROS 2 controller
- Robot kinematics and odometry
- Robot description and STL meshes
- Goal pose controller
- JSON-based trajectory planner
- RViz visualization

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- colcon
- rosdep

Additional ROS 2 dependencies may be required.

## 1. Clone the Repository

Open a terminal:

    cd ~
    git clone <REPOSITORY_URL> ros2_ws  ## include the name of workspace you work on it
    cd ~/ros2_ws

Replace <REPOSITORY_URL> with the Git repository URL.

## 2. Install Dependencies

    source /opt/ros/humble/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y


## 3. Build the Project

    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

## 4. Launch the Robot in RViz

Open Terminal 1:

    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch bringup bringup.launch.py

The robot model should appear in RViz.

## 5. Execute the JSON Trajectory

Open Terminal 2:

    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run trajectory_planner trajectory_planner

The trajectory planner automatically reads:

    ~/ros2_ws/trajectory_config/trajectory.json
    ~/ros2_ws/trajectory_config/poses.json

It then sends the configured target poses to the goal controller.

