import json
import math
import os
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from trajectory_interfaces.action import ExecuteTrajectoryTarget


class TrajectoryPlannerNode(Node):

    def __init__(self):
        super().__init__('trajectory_planner')

        # Configuration parameters.
        # Leave trajectory_config_dir empty to discover it automatically.
        self.declare_parameter('trajectory_config_dir', '')
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('poses_file', '')
        self.declare_parameter('execute_on_start', True)

        config_dir = self.get_parameter(
            'trajectory_config_dir'
        ).value

        trajectory_file = self.get_parameter(
            'trajectory_file'
        ).value

        poses_file = self.get_parameter(
            'poses_file'
        ).value

        self.execute_on_start = self.get_parameter(
            'execute_on_start'
        ).value

        # Discover or validate the configuration file paths.
        self.trajectory_file, self.poses_file = (
            self.discover_configuration_files(
                config_dir=config_dir,
                trajectory_file=trajectory_file,
                poses_file=poses_file,
            )
        )

        self.get_logger().info(
            f'Trajectory file: {self.trajectory_file}'
        )
        self.get_logger().info(
            f'Poses file: {self.poses_file}'
        )

        # ROS 2 action client.
        self.action_client = ActionClient(
            self,
            ExecuteTrajectoryTarget,
            '/execute_trajectory_target'
        )

        # Execution state.
        self.commands = []
        self.current_index = 0
        self.goal_in_progress = False
        self.execution_started = False
        self.start_timer = None

        try:
            self.load_configuration()

        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(
                f'Could not load trajectory configuration: {exc}'
            )
            raise

        if not self.execute_on_start:
            self.get_logger().info(
                'Dry-run mode: configuration validated; '
                'no movement commands will be sent.'
            )
            self.print_trajectory()

        else:
            self.get_logger().info(
                'Execution enabled. Waiting for the action server.'
            )
            self.start_timer = self.create_timer(
                0.5, self.try_start_execution
            )

    # =========================================================
    # Configuration file discovery
    # =========================================================

    def discover_configuration_files(
        self,
        config_dir='',
        trajectory_file='',
        poses_file='',
    ):
        """
        Find trajectory.json and poses.json automatically.

        Search order:
        1. Explicit trajectory_file and poses_file parameters.
        2. Explicit trajectory_config_dir parameter.
        3. Common workspace/configuration locations.
        4. Search the user's home directory.

        Both JSON files must exist in the same configuration directory
        when automatic discovery is used.
        """

        # Option 1: explicit file paths.
        if trajectory_file or poses_file:
            if not trajectory_file or not poses_file:
                raise ValueError(
                    'Set both trajectory_file and poses_file, '
                    'or use trajectory_config_dir instead.'
                )

            trajectory_path = Path(
                trajectory_file
            ).expanduser()

            poses_path = Path(
                poses_file
            ).expanduser()

            if not trajectory_path.is_file():
                raise FileNotFoundError(
                    f'Trajectory file not found: {trajectory_path}'
                )

            if not poses_path.is_file():
                raise FileNotFoundError(
                    f'Poses file not found: {poses_path}'
                )

            self.get_logger().info(
                'Using explicitly configured JSON file paths.'
            )

            return str(trajectory_path), str(poses_path)

        # Option 2: explicitly configured directory.
        if config_dir:
            directory = Path(config_dir).expanduser()

            trajectory_path = directory / 'trajectory.json'
            poses_path = directory / 'poses.json'

            missing = [
                str(path)
                for path in (trajectory_path, poses_path)
                if not path.is_file()
            ]

            if missing:
                raise FileNotFoundError(
                    'Configuration directory is missing required '
                    f'files: {", ".join(missing)}'
                )

            self.get_logger().info(
                f'Using configured directory: {directory}'
            )

            return str(trajectory_path), str(poses_path)

        home = Path.home()
        cwd = Path.cwd()

        # Option 3: likely locations.
        candidates = [
            home / 'ros2_ws' / 'trajectory_config',
            home / 'trajectory_config',
            cwd / 'trajectory_config',
            cwd / 'ros2_ws' / 'trajectory_config',
        ]

        # Also inspect parent directories of the current working directory.
        candidates.extend(
            parent / 'trajectory_config'
            for parent in cwd.parents
        )

        checked = set()

        for directory in candidates:
            try:
                directory = directory.resolve()
            except OSError:
                continue

            if directory in checked:
                continue

            checked.add(directory)

            trajectory_path = directory / 'trajectory.json'
            poses_path = directory / 'poses.json'

            if (
                trajectory_path.is_file()
                and poses_path.is_file()
            ):
                self.get_logger().info(
                    f'Automatically found configuration: {directory}'
                )

                return str(trajectory_path), str(poses_path)

        # Option 4: search the user's home directory.
        # Skip common directories that do not normally contain project
        # configuration and may make the search unnecessarily expensive.
        ignored_directories = {
            '.cache',
            '.local',
            '.ros',
            '.git',
            '.venv',
            'venv',
            'build',
            'install',
            'log',
            'node_modules',
            '__pycache__',
        }

        for root, directories, files in os.walk(
            home,
            followlinks=False,
        ):
            directories[:] = [
                directory
                for directory in directories
                if directory not in ignored_directories
                and not directory.startswith('.')
            ]

            if Path(root).name != 'trajectory_config':
                continue

            if (
                'trajectory.json' in files
                and 'poses.json' in files
            ):
                directory = Path(root)

                self.get_logger().info(
                    'Automatically discovered configuration: '
                    f'{directory}'
                )

                return (
                    str(directory / 'trajectory.json'),
                    str(directory / 'poses.json'),
                )

        raise FileNotFoundError(
            'Could not find both trajectory.json and poses.json '
            'inside a trajectory_config directory. '
            'Place the directory in a searchable location under '
            'your home directory, or set the '
            'trajectory_config_dir ROS parameter explicitly.'
        )

    # =========================================================
    # Load and validate JSON configuration
    # =========================================================

    def load_configuration(self):
        with open(self.trajectory_file, 'r') as file:
            trajectory = json.load(file)

        with open(self.poses_file, 'r') as file:
            pose_data = json.load(file)

        poses = pose_data.get('poses', {})
        raw_commands = trajectory.get('commands', [])

        if not isinstance(poses, dict):
            raise ValueError("'poses' must be a JSON object.")

        if not isinstance(raw_commands, list) or not raw_commands:
            raise ValueError("'commands' must be a non-empty list.")

        supported_types = {
            'forward',
            'rotate',
            'shift',
            'combined',
            'completed',
            'diagonal',
            'omnidirectional',
        }

        for index, command in enumerate(raw_commands):
            if (
                not isinstance(command, dict)
                or 'move_to' not in command
            ):
                raise ValueError(
                    f'Command {index + 1} must contain "move_to".'
                )

            move = command['move_to']

            if not isinstance(move, dict):
                raise ValueError(
                    f'Command {index + 1}: "move_to" must be an '
                    'object.'
                )

            pose_name = move.get('pose')

            movement_type = str(
                move.get('movement_type', 'omnidirectional')
            ).strip().lower()

            if movement_type not in supported_types:
                raise ValueError(
                    f'Command {index + 1}: unsupported movement type '
                    f"'{movement_type}'."
                )

            if pose_name not in poses:
                raise ValueError(
                    f'Command {index + 1}: pose "{pose_name}" '
                    'does not exist in the poses file.'
                )

            pose = poses[pose_name]

            if not isinstance(pose, dict):
                raise ValueError(
                    f'Pose "{pose_name}" must be a JSON object.'
                )

            if 'x' not in pose or 'y' not in pose:
                raise ValueError(
                    f'Pose "{pose_name}" must contain x and y.'
                )

            x = float(pose['x'])
            y = float(pose['y'])

            use_target_yaw = 'yaw' in pose
            yaw = float(pose['yaw']) if use_target_yaw else 0.0

            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError(
                    f'Pose "{pose_name}" contains a non-finite value.'
                )

            if movement_type == 'rotate' and not use_target_yaw:
                raise ValueError(
                    f'Rotate command for "{pose_name}" requires a yaw.'
                )

            # Preserve compatibility with older JSON commands.
            if movement_type == 'completed':
                movement_type = 'combined'

            self.commands.append({
                'pose_name': pose_name,
                'x': x,
                'y': y,
                'yaw': yaw,
                'use_target_yaw': use_target_yaw,
                'movement_type': movement_type,
                'trajectory_type': move.get(
                    'traj', 'deterministic'
                ),
            })

        self.trajectory_name = trajectory.get(
            'trajectory_name', 'unnamed_trajectory'
        )

        self.get_logger().info(
            f'Loaded trajectory "{self.trajectory_name}" '
            f'with {len(self.commands)} commands.'
        )

    # =========================================================
    # Display trajectory
    # =========================================================

    def print_trajectory(self):
        for index, command in enumerate(self.commands, start=1):
            self.get_logger().info(
                f'Command {index}: pose={command["pose_name"]}, '
                f'type={command["movement_type"]}, '
                f'x={command["x"]:.3f}, '
                f'y={command["y"]:.3f}, '
                f'use_target_yaw={command["use_target_yaw"]}'
            )

    # =========================================================
    # Start trajectory execution
    # =========================================================

    def try_start_execution(self):
        if self.execution_started:
            return

        if not self.action_client.server_is_ready():
            self.get_logger().info(
                'Waiting for /execute_trajectory_target...'
            )
            return

        self.execution_started = True
        self.start_timer.cancel()

        self.get_logger().info(
            f'Starting trajectory "{self.trajectory_name}".'
        )

        self.send_next_goal()

    # =========================================================
    # Send next trajectory goal
    # =========================================================

    def send_next_goal(self):
        if self.current_index >= len(self.commands):
            self.get_logger().info(
                f'Trajectory "{self.trajectory_name}" completed.'
            )
            return

        if self.goal_in_progress:
            return

        command = self.commands[self.current_index]

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'odom'
        target_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        target_pose.pose.position.x = command['x']
        target_pose.pose.position.y = command['y']
        target_pose.pose.position.z = 0.0

        half_yaw = command['yaw'] / 2.0

        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = math.sin(half_yaw)
        target_pose.pose.orientation.w = math.cos(half_yaw)

        goal = ExecuteTrajectoryTarget.Goal()
        goal.target_pose = target_pose
        goal.movement_type = command['movement_type']
        goal.use_target_yaw = command['use_target_yaw']

        self.goal_in_progress = True

        self.get_logger().info(
            f'Sending command {self.current_index + 1}/'
            f'{len(self.commands)}: {command["pose_name"]} '
            f'({command["movement_type"]})'
        )

        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    # =========================================================
    # Handle goal response
    # =========================================================

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()

        except Exception as exc:
            self.goal_in_progress = False

            self.get_logger().error(
                f'Could not send trajectory goal: {exc}'
            )
            return

        if not goal_handle.accepted:
            self.goal_in_progress = False

            self.get_logger().error(
                f'Command {self.current_index + 1} was rejected.'
            )
            return

        self.get_logger().info(
            'Goal accepted by controller.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # =========================================================
    # Handle trajectory result
    # =========================================================

    def result_callback(self, future):
        self.goal_in_progress = False

        try:
            wrapped_result = future.result()
            result = wrapped_result.result

        except Exception as exc:
            self.get_logger().error(
                f'Could not receive trajectory result: {exc}'
            )
            return

        if not result.success:
            self.get_logger().error(
                f'Command {self.current_index + 1} failed: '
                f'{result.message}. Remaining commands stopped.'
            )
            return

        self.get_logger().info(
            f'Command {self.current_index + 1} succeeded: '
            f'{result.message}'
        )

        self.current_index += 1
        self.send_next_goal()

    # =========================================================
    # Action feedback
    # =========================================================

    def feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback

        self.get_logger().debug(
            f'State: {feedback.state}; '
            f'distance remaining: '
            f'{feedback.distance_remaining:.3f} m'
        )


# =============================================================
# Main
# =============================================================

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TrajectoryPlannerNode()
        rclpy.spin(node)

    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
    ) as exc:
        if node is None:
            print(f'Trajectory planner startup failed: {exc}')

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()