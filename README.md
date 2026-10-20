# CIIMA ROS 2 Mobile Robot Project

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
- Robot control GUI

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
    git clone <REPOSITORY_URL> ros2_ws
    cd ~/ros2_ws

Replace <REPOSITORY_URL> with the Git repository URL.

## 2. Install Dependencies

    source /opt/ros/humble/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

Note: Some package manifests declare optional dependencies,
including navigation and sensor packages. If rosdep reports
missing packages, review the errors before proceeding.

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

## 6. Expected Result

The robot should execute the sequence of target poses
defined in trajectory.json.

The current test includes:

- move_to_A
- move_to_B
- move_to_C
- move_to_D
- move_to_E
- move_to_home

Observe the robot movement in RViz and check terminal
output for errors.

## 7. Troubleshooting

If a ROS 2 package cannot be found:

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

If the trajectory planner cannot find JSON files,
verify that the trajectory_config directory exists
directly inside ~/ros2_ws.

If compilation fails, record the complete error output.

## 8. Notes

This project currently demonstrates robot motion using
ROS 2 controllers and RViz visualization.

RViz is a visualization tool, not a physics simulator.
The project does not require Gazebo for this test.

The Siemens ArgoDrive simulation is a separate activity.

## 9. Feedback

Please report:
- Whether the project builds successfully
- Whether the robot appears in RViz
- Whether the JSON trajectory executes correctly
- Any missing dependencies or unclear instructions
