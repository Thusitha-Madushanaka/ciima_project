import json
import math
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from trajectory_interfaces.action import ExecuteTrajectoryTarget


class TrajectoryPlannerNode(Node):

    def __init__(self):
        super().__init__('trajectory_planner')

        self.declare_parameter(
            'trajectory_file',
            str(Path.home() / 'ros2_ws/trajectory_config/trajectory.json')
        )
        self.declare_parameter(
            'poses_file',
            str(Path.home() / 'ros2_ws/trajectory_config/poses.json')
        )
        self.declare_parameter('execute_on_start', True) # set is as true to run automatically without argument

        self.trajectory_file = self.get_parameter(
            'trajectory_file'
        ).value
        self.poses_file = self.get_parameter('poses_file').value
        self.execute_on_start = self.get_parameter(
            'execute_on_start'
        ).value

        self.action_client = ActionClient(
            self,
            ExecuteTrajectoryTarget,
            '/execute_trajectory_target'
        )

        self.commands = []
        self.current_index = 0
        self.goal_in_progress = False
        self.execution_started = False

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
            if not isinstance(command, dict) or 'move_to' not in command:
                raise ValueError(
                    f'Command {index + 1} must contain "move_to".'
                )

            move = command['move_to']
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

            # "completed" is retained for compatibility with older JSON.
            if movement_type == 'completed':
                movement_type = 'combined'

            self.commands.append({
                'pose_name': pose_name,
                'x': x,
                'y': y,
                'yaw': yaw,
                'use_target_yaw': use_target_yaw,
                'movement_type': movement_type,
                'trajectory_type': move.get('traj', 'deterministic'),
            })

        self.trajectory_name = trajectory.get(
            'trajectory_name', 'unnamed_trajectory'
        )

        self.get_logger().info(
            f'Loaded trajectory "{self.trajectory_name}" '
            f'with {len(self.commands)} commands.'
        )

    def print_trajectory(self):
        for index, command in enumerate(self.commands, start=1):
            self.get_logger().info(
                f'Command {index}: pose={command["pose_name"]}, '
                f'type={command["movement_type"]}, '
                f'x={command["x"]:.3f}, y={command["y"]:.3f}, '
                f'use_target_yaw={command["use_target_yaw"]}'
            )

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
        target_pose.header.stamp = self.get_clock().now().to_msg()

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

        self.get_logger().info('Goal accepted by controller.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

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

    def feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().debug(
            f'State: {feedback.state}; '
            f'distance remaining: '
            f'{feedback.distance_remaining:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TrajectoryPlannerNode()
        rclpy.spin(node)
    except (OSError, ValueError, KeyError, TypeError):
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
