# CIIMA ROS 2 Omnidirectional Autonomus Mobile Robot Controller

It Includes:
  1. custom_controller_kinematics
  2. custom_controller
  3. bringup files
  4. goal_pose_controller
  5. JSON files - Trajectory implementation 

## 1. Supported environment

- **Ubuntu 22.04 (Jammy)**, native installation recommended
- **ROS 2 Humble** (ROS 2 Humble packages are available for Ubuntu 22.04)
- Internet connection and access to this repository

Other platforms are not validated for this project.

## 2. Prepare a clean Ubuntu installation

### 2.1 Install ROS 2 Humble

Follow the official **Ubuntu (Debian packages)** installation instructions for ROS 2 Humble:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Install the **Desktop** variant (`ros-humble-desktop`) so RViz is available. Complete the official repository setup steps first; do not attempt to install ROS packages before configuring the ROS 2 apt repository.

Confirm ROS is installed:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
echo $ROS_DISTRO
```

### 2.2 Install development and build tools

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-ament-package
```

Initialize `rosdep` only if it has not already been initialized:

```bash
sudo rosdep init
```

If this reports that rosdep is already initialized, continue:

```bash
rosdep update
```

### 2.3 Install core ROS packages used by the RViz demo

```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2
```

These are the core tools for the demonstrated workflow, **not a verified exhaustive dependency list**. Additional dependencies are declared in each package's `package.xml`.

## 3. Clone the project

```bash
cd ~
git clone https://github.com/Thusitha-Madushanaka/ciima_project.git ros2_ws
cd ~/ros2_ws
```

Expected workspace layout:

```text
workspace_name/
├── src/
│   ├── ciima_bringup/                 # ROS package name: bringup
│   ├── ciima_description/
│   ├── custom_controller/
│   ├── custom_controller_kinematics/
│   ├── goal_pose_controller/
│   ├── robot_control_gui/
│   ├── trajectory_interfaces/
│   └── trajectory_planner/
└── trajectory_config/
    ├── trajectory.json
    └── poses.json
```

## 4. Resolve dependencies and build

```bash
cd ~/workspace_name
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

Check that the packages are discoverable:

```bash
colcon list --names-only
```

Expected package names: `bringup`, `ciima_description`, `custom_controller`, `custom_controller_kinematics`, `goal_pose_controller`, `robot_control_gui`, `trajectory_interfaces`, and `trajectory_planner`.

## 5. Launch the robot and RViz

In **Terminal 1**:

```bash
cd ~/workspace_name
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch bringup bringup.launch.py
```

Wait for RViz to show the robot and for `custom_controller` and `joint_state_broadcaster` to become active. This launch also starts `goal_pose_controller`; **do not start another instance**.

## 6. Execute the JSON trajectory

In **Terminal 2**:

```bash
cd ~/workspace_name
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run trajectory_planner trajectory_planner
```

The current planner uses `execute_on_start=True` and automatically sends the trajectory once the `/execute_trajectory_target` action server is ready.


**ENJOY the CIIMA ROBOT**

**Made by: Thusitha Madushanaka @ InnoBotics**
**@All right reserved**