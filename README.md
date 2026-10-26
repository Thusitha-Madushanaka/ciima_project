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


### 7 CMake / `ament_cmake` errors on a fresh computer

If a build fails with a message such as:

```text
Could not find a package configuration file provided by "ament_cmake"
```

first verify that the correct ROS installation is present and sourced **in the same terminal**:

```bash
ls /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
ros2 pkg prefix ament_cmake
```

The ROS distribution should be `humble`, and the last command should resolve to a ROS installation path. If `ament_cmake` is genuinely missing, check/install its Ubuntu package:

```bash
sudo apt update
sudo apt install ros-humble-ament-cmake
```

Then source `/opt/ros/humble/setup.bash` again and retry the build. **Do not edit the project's `find_package(ament_cmake REQUIRED)` lines or install ament using arbitrary pip commands to work around an unsourced ROS environment.**

If the error instead says `No module named 'ament_package'`, check the Python environment and install the Ubuntu-provided Python module:

```bash
which python3
python3 -c "import ament_package; print(ament_package.__file__)"
sudo apt install python3-ament-package
```

A custom Python virtual environment, Conda environment, or unexpected `PYTHONPATH` can hide Ubuntu/ROS Python packages. Use a clean terminal with the system Python when diagnosing build failures.

If CMake reports that another `*Config.cmake` file cannot be found, **copy the exact package name from the first error**. Source ROS and run `rosdep` first; the missing dependency may be specific to this project.

### 7.1 Capture the exact failing package and first error

Run the build with output printed directly to the terminal and also saved to a log:

```bash
cd ~/workspace_name
source /opt/ros/humble/setup.bash
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee ~/ros2_build.log
```

Review the first `CMake Error`, `ModuleNotFoundError`, or `Failed <<<` message. The **first actual error** is usually more useful than the final build summary. Send the full relevant error block to the project developer; do not assume the problem is `ament_cmake` merely because the final summary mentions CMake.

For a failed first attempt on a newly transferred workspace, also check that `build/`, `install/`, and `log/` were **not copied from another computer**. These are generated directories and should be rebuilt locally. If stale build artifacts are suspected, first save the error log and then, only in the transferred/test workspace, run:

```bash
cd ~/workspace_name
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --event-handlers console_direct+
```

**Caution:** The cleanup deletes generated workspace artifacts. Do not use it on the original working development workspace unless you intend to rebuild everything.

### 7.2 Information to include when asking for help

```bash
lsb_release -ds
echo "ROS_DISTRO=$ROS_DISTRO"
cmake --version | head -n 1
which python3
colcon list --names-only
```

Include the terminal output and the relevant portion of `~/ros2_build.log`. The exact build error observed on the independent tester's computer is **not yet known**, so the steps above are diagnostic suggestions, not a confirmed repair.


## 8. Repository updates

After modifying and testing code in `~/ros2_ws`:

```bash
git status
git add <files-you-want-to-commit>
git commit -m "Describe the change"
git push
```

On a separate test machine, update with `git pull` and rebuild using `colcon build --symlink-install`.

## 9. Access and confidentiality

This project is intended for authorized collaborators. Do not redistribute company code, robot models, or configuration files without the owner's approval. Private repository access must be granted explicitly.
