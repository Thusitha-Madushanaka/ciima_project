CIIMA ROS 2 Mobile Robot Controller
ROS 2 Humble project for a mobile robot with two active steerable OmniDrive modules. This repository includes a custom ros2_control controller, kinematics/odometry, robot description and meshes, goal-pose controller, JSON trajectory planner, RViz configuration, and an experimental GUI.

Scope: The procedure below demonstrates the controller and JSON trajectory in RViz. RViz is not a physics simulator. This guide does not cover Gazebo, Siemens ArgoDrive simulation, or commissioning a physical robot.
1. Supported environment
- Ubuntu 22.04 (Jammy), native installation recommended
- ROS 2 Humble (ROS 2 Humble packages are available for Ubuntu 22.04)
- Internet connection and access to this repository
Other platforms are not validated for this project.
2. Prepare a clean Ubuntu installation
2.1 Install ROS 2 Humble
Follow the official Ubuntu (Debian packages) installation instructions for ROS 2 Humble:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Install the Desktop variant (ros-humble-desktop) so RViz is available. Complete the official repository setup steps first; do not attempt to install ROS packages before configuring the ROS 2 apt repository.

Confirm ROS is installed:
source /opt/ros/humble/setup.bash
ros2 --help
2.2 Install development and build tools
sudo apt update
sudo apt install -y git build-essential cmake python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-ament-package
Initialize rosdep only if it has not already been initialized:
sudo rosdep init
If this reports that rosdep is already initialized, continue:
rosdep update
2.3 Install core ROS packages used by the RViz demo
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2
These are the core tools for the demonstrated workflow, not a verified exhaustive dependency list. Additional dependencies are declared in each package's package.xml.
3. Clone the project
The repository is private: request collaborator access from its owner before cloning.

Important: The trajectory planner's current default JSON paths require the workspace directory to be named ~/ros2_ws.
cd ~
git clone https://github.com/Thusitha-Madushanaka/ciima_project.git ros2_ws
cd ~/ros2_ws
Expected workspace layout:
ros2_ws/
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
4. Resolve dependencies and build
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
Dependency caveat: Some package manifests declare additional Nav2, SLAM, scanner, or camera-related dependencies that are not needed by the current RViz launch. rosdep may attempt to install these anyway. If dependency resolution fails, retain the full error message and determine whether the dependency is actually needed; do not delete package declarations blindly. The full clean-machine dependency installation has not yet been independently validated.

Check that the packages are discoverable:
colcon list --names-only
Expected package names: bringup, ciima_description, custom_controller, custom_controller_kinematics, goal_pose_controller, robot_control_gui, trajectory_interfaces, and trajectory_planner.
5. Launch the robot and RViz
In Terminal 1:
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bringup bringup.launch.py
Wait for RViz to show the robot and for custom_controller and joint_state_broadcaster to become active. This launch also starts goal_pose_controller; do not start another instance.
6. Execute the JSON trajectory
In Terminal 2:
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run trajectory_planner trajectory_planner
The current planner uses execute_on_start=True and automatically sends the trajectory once the /execute_trajectory_target action server is ready.

The JSON files are:
- ~/ros2_ws/trajectory_config/trajectory.json: ordered movement commands
- ~/ros2_ws/trajectory_config/poses.json: named target positions
The supplied test sequence is A → B → C → D → E → Home, with these targets in the odom frame (x/y in metres, yaw in radians):
Target	x	y	yaw
A	2.0	0.0	0.0
B	0.0	2.0	0.0
C	0.0	1.0	0.0
D	0.5	0.5	0.0
E	0.0	0.0	3.14
Home	0.0	0.0	0.0


Confirm that RViz displays the motion and the planner reports the six targets completed in order. This is a visualization/controller test, not a physical-dynamics validation.
7. Troubleshooting
- ros2: command not found — source /opt/ros/humble/setup.bash in that terminal.
- Package not found — check the build succeeded and source ~/ros2_ws/install/setup.bash.
- JSON file not found — clone the repository as ~/ros2_ws or update the planner's trajectory_file and poses_file parameters; cloning under a different folder name will not work with the defaults.
- Planner waits for server — confirm Terminal 1 is running and /execute_trajectory_target is available.
- RViz robot missing — inspect launch output, robot description, and RViz display settings.
- rosdep or build failure — save the complete output, including the first error, OS version, and ROS distribution.
Stop the trajectory planner with Ctrl+C in Terminal 2 and the launch with Ctrl+C in Terminal 1. Ctrl+C is not a physical robot emergency stop.
7.1 CMake / ament_cmake errors on a fresh computer
If a build fails with a message such as:
Could not find a package configuration file provided by "ament_cmake"
first verify that the correct ROS installation is present and sourced in the same terminal:
ls /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
ros2 pkg prefix ament_cmake
The ROS distribution should be humble, and the last command should resolve to a ROS installation path. If ament_cmake is genuinely missing, check/install its Ubuntu package:
sudo apt update
sudo apt install ros-humble-ament-cmake
Then source /opt/ros/humble/setup.bash again and retry the build. Do not edit the project's find_package(ament_cmake REQUIRED) lines or install ament using arbitrary pip commands to work around an unsourced ROS environment.

If the error instead says No module named 'ament_package', check the Python environment and install the Ubuntu-provided Python module:
which python3
python3 -c "import ament_package; print(ament_package.__file__)"
sudo apt install python3-ament-package
A custom Python virtual environment, Conda environment, or unexpected PYTHONPATH can hide Ubuntu/ROS Python packages. Use a clean terminal with the system Python when diagnosing build failures.

If CMake reports that another *Config.cmake file cannot be found, copy the exact package name from the first error. Source ROS and run rosdep first; the missing dependency may be specific to this project.
7.2 Capture the exact failing package and first error
Run the build with output printed directly to the terminal and also saved to a log:
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee ~/ros2_build.log
Review the first CMake Error, ModuleNotFoundError, or Failed <<< message. The first actual error is usually more useful than the final build summary. Send the full relevant error block to the project developer; do not assume the problem is ament_cmake merely because the final summary mentions CMake.

For a failed first attempt on a newly transferred workspace, also check that build/, install/, and log/ were not copied from another computer. These are generated directories and should be rebuilt locally. If stale build artifacts are suspected, first save the error log and then, only in the transferred/test workspace, run:
cd ~/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --event-handlers console_direct+
Caution: The cleanup deletes generated workspace artifacts. Do not use it on the original working development workspace unless you intend to rebuild everything.
7.3 Information to include when asking for help
lsb_release -ds
echo "ROS_DISTRO=$ROS_DISTRO"
cmake --version | head -n 1
which python3
colcon list --names-only
Include the terminal output and the relevant portion of ~/ros2_build.log. The exact build error observed on the independent tester's computer is not yet known, so the steps above are diagnostic suggestions, not a confirmed repair.
8. Independent tester checklist
Please report:
- Ubuntu and ROS 2 versions used
- Whether the repository cloned successfully
- Whether dependency installation completed
- Whether all eight packages built successfully
- Whether RViz showed the robot
- Whether all six JSON trajectory commands completed in order
- Any errors, missing prerequisites, or unclear instructions
Validation status: The developer has tested the controller/trajectory on the original Ubuntu workspace and has reported a successful build of a fresh clone on the same machine. A clean Ubuntu installation on a separate computer has not yet been verified. Treat the independent setup test as the next validation milestone.
9. Repository updates
After modifying and testing code in ~/ros2_ws:
git status
git add <files-you-want-to-commit>
git commit -m "Describe the change"
git push
On a separate test machine, update with git pull and rebuild using colcon build --symlink-install.
10. Access and confidentiality
This project is intended for authorized collaborators. Do not redistribute company code, robot models, or configuration files without the owner's approval. Private repository access must be granted explicitly.