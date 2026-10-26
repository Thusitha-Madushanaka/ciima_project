import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.task import Future

from trajectory_interfaces.action import ExecuteTrajectoryTarget
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class GoalPoseController(Node):

    def __init__(self):
        super().__init__('goal_pose_controller')

        # =====================================================
        # Controller parameters
        # =====================================================

        self.declare_parameter('kp_x', 0.5)
        self.declare_parameter('kp_y', 0.5)
        self.declare_parameter('kp_yaw', 1.0)

        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_omega', 0.5)

        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('yaw_tolerance', 0.02)
        self.declare_parameter('steering_tolerance', 0.05)
        self.declare_parameter('action_timeout', 120.0)

        self.kp_x = self.get_parameter('kp_x').value
        self.kp_y = self.get_parameter('kp_y').value
        self.kp_yaw = self.get_parameter('kp_yaw').value

        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_omega = self.get_parameter('max_omega').value

        self.position_tolerance = (
            self.get_parameter('position_tolerance').value
        )
        self.yaw_tolerance = (
            self.get_parameter('yaw_tolerance').value
        )
        self.steering_tolerance = (
            self.get_parameter('steering_tolerance').value
        )
        self.action_timeout = (
            self.get_parameter('action_timeout').value
        )

        # =====================================================
        # ROS interfaces
        # =====================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # =====================================================
        # Robot state
        # =====================================================

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # =====================================================
        # Goal state
        # =====================================================

        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        self.goal_mode = 'normal'
        self.travel_yaw = None

        # =====================================================
        # Steering feedback
        # =====================================================

        self.front_steer_angle = None
        self.rear_steer_angle = None

        # =====================================================
        # Action state
        # =====================================================

        self.active_goal_handle = None
        self.active_movement_type = None
        self.action_start_yaw = None
        self.action_start_time = None
        self.action_use_target_yaw = False
        self.action_tick_future = None

        self.state = 'WAITING_FOR_GOAL'

        # =====================================================
        # Load legacy goal mode
        # =====================================================

        json_path = (
            Path.home()
            / 'ros2_ws'
            / 'trajectory_config'
            / 'sideways_goal.json'
        )

        try:
            with open(json_path, 'r', encoding='utf-8') as file:
                data = json.load(file)

            self.goal_mode = data['goal']['type']

            if self.goal_mode not in ('normal', 'sideways'):
                self.get_logger().warning(
                    f'Unknown legacy goal mode: {self.goal_mode}. '
                    'Using normal mode.'
                )
                self.goal_mode = 'normal'

            self.get_logger().info(
                f'Legacy goal mode: {self.goal_mode}'
            )

        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(
                f'Could not load goal configuration: {exc}. '
                'Using normal mode.'
            )
            self.goal_mode = 'normal'

        # =====================================================
        # Action server
        # =====================================================

        self.trajectory_action_server = ActionServer(
            self,
            ExecuteTrajectoryTarget,
            'execute_trajectory_target',
            execute_callback=self.execute_target,
            goal_callback=self.action_goal_callback,
            cancel_callback=self.action_cancel_callback
        )

        # Main control loop: 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Goal pose controller started.'
        )

        self.get_logger().info(
            'Trajectory action server ready: '
            '/execute_trajectory_target'
        )

    # =========================================================
    # Utility functions
    # =========================================================

    def normalize_angle(self, angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def clamp(self, value, minimum, maximum):
        return max(
            minimum,
            min(value, maximum)
        )

    def publish_cmd_vel(self, vx, vy, omega):
        msg = Twist()

        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(omega)

        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd_vel(
            0.0,
            0.0,
            0.0
        )

    def get_yaw_from_pose(self, pose):
        q = pose.orientation

        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def world_error_to_robot(self, dx, dy):
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        error_x_robot = (
            cos_yaw * dx + sin_yaw * dy
        )

        error_y_robot = (
            -sin_yaw * dx + cos_yaw * dy
        )

        return error_x_robot, error_y_robot

    # =========================================================
    # Odometry callback
    # =========================================================

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        self.current_yaw = self.get_yaw_from_pose(
            msg.pose.pose
        )

    # =========================================================
    # Joint state callback
    # =========================================================

    def joint_state_callback(self, msg):
        for i, joint_name in enumerate(msg.name):

            if i >= len(msg.position):
                continue

            if joint_name == 'fake_robot_front_steer_joint':
                self.front_steer_angle = msg.position[i]

            elif joint_name == 'fake_robot_rear_steer_joint':
                self.rear_steer_angle = msg.position[i]

    # =========================================================
    # Legacy /goal_pose topic interface
    # =========================================================

    def goal_callback(self, msg):

        if self.active_goal_handle is not None:
            self.get_logger().warning(
                'Ignoring /goal_pose because an action is active.'
            )
            return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = self.get_yaw_from_pose(
            msg.pose
        )

        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
        ):
            self.get_logger().warning(
                'Goal received, but odometry is not available.'
            )
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.hypot(dx, dy)

        if self.goal_mode == 'sideways':

            self.travel_yaw = None

            self.state = (
                'ALIGN_STEERING'
                if distance > self.position_tolerance
                else 'GOAL_REACHED'
            )

        elif distance > self.position_tolerance:

            self.travel_yaw = math.atan2(dy, dx)
            self.state = 'ROTATE_TO_TRAVEL'

        else:

            self.travel_yaw = None
            self.state = 'ROTATE_TO_GOAL'

        self.get_logger().info(
            f'Legacy goal received: '
            f'x={self.goal_x:.3f}, '
            f'y={self.goal_y:.3f}, '
            f'yaw={self.goal_yaw:.3f}'
        )

        self.get_logger().info(
            f'State: {self.state}'
        )

    # =========================================================
    # Action callbacks
    # =========================================================

    def action_goal_callback(self, goal_request):

        movement_type = (
            goal_request.movement_type.strip().lower()
        )

        supported_types = (
            'forward',
            'rotate',
            'shift',
            'combined',
            'diagonal',
            'omnidirectional'
        )

        if movement_type not in supported_types:
            self.get_logger().warning(
                f'Rejecting unsupported movement type: '
                f'{movement_type}'
            )
            return GoalResponse.REJECT

        if self.active_goal_handle is not None:
            self.get_logger().warning(
                'Rejecting goal: another trajectory is active.'
            )
            return GoalResponse.REJECT

        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
        ):
            self.get_logger().warning(
                'Rejecting trajectory: odometry is not available.'
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def action_cancel_callback(self, goal_handle):
        self.get_logger().info(
            'Trajectory cancellation accepted.'
        )

        return CancelResponse.ACCEPT

    # =========================================================
    # Execute trajectory action
    # =========================================================

    async def execute_target(self, goal_handle):

        result = ExecuteTrajectoryTarget.Result()
        request = goal_handle.request

        movement_type = (
            request.movement_type.strip().lower()
        )

        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
        ):
            result.success = False
            result.message = 'Odometry is not available.'

            goal_handle.abort()
            return result

        self.active_goal_handle = goal_handle
        self.active_movement_type = movement_type

        self.action_start_yaw = self.current_yaw
        self.action_start_time = time.monotonic()
        self.action_use_target_yaw = (
            request.use_target_yaw
        )

        try:
            self.goal_x = (
                request.target_pose.pose.position.x
            )

            self.goal_y = (
                request.target_pose.pose.position.y
            )

            requested_yaw = self.get_yaw_from_pose(
                request.target_pose.pose
            )

            if movement_type == 'rotate':

                # Rotation only: hold current position.
                self.goal_x = self.current_x
                self.goal_y = self.current_y
                self.goal_yaw = requested_yaw

                # A rotate action always requires target yaw.
                self.action_use_target_yaw = True

            elif request.use_target_yaw:

                self.goal_yaw = requested_yaw

            else:

                # Position-only action:
                # preserve the starting heading.
                self.goal_yaw = self.action_start_yaw

            # =================================================
            # Validate constrained movement types
            # =================================================

            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y

            local_x, local_y = (
                self.world_error_to_robot(dx, dy)
            )

            if movement_type == 'forward':

                if (
                    abs(local_y) > self.position_tolerance
                    or local_x < -self.position_tolerance
                ):
                    result.success = False
                    result.message = (
                        'Forward target is not along the '
                        'current forward direction.'
                    )

                    goal_handle.abort()
                    return result

            elif movement_type == 'shift':

                if abs(local_x) > self.position_tolerance:
                    result.success = False
                    result.message = (
                        'Shift target is not along the '
                        'current lateral direction.'
                    )

                    goal_handle.abort()
                    return result

            elif movement_type == 'diagonal':

                if (
                    abs(local_x) <= self.position_tolerance
                    or abs(local_y) <= self.position_tolerance
                ):
                    result.success = False
                    result.message = (
                        'Diagonal target must require both '
                        'forward and lateral displacement.'
                    )

                    goal_handle.abort()
                    return result

            self.state = 'ACTION_CONTROL'

            self.get_logger().info(
                f'Starting action: {movement_type}, '
                f'use_target_yaw={self.action_use_target_yaw}'
            )

            # =================================================
            # Wait until control loop reaches the goal
            # =================================================

            while rclpy.ok():

                if goal_handle.is_cancel_requested:

                    self.stop_robot()
                    goal_handle.canceled()

                    result.success = False
                    result.message = (
                        'Trajectory target cancelled.'
                    )

                    return result

                elapsed = (
                    time.monotonic() - self.action_start_time
                )

                if elapsed > self.action_timeout:

                    self.stop_robot()
                    goal_handle.abort()

                    result.success = False
                    result.message = (
                        'Trajectory target timed out.'
                    )

                    self.get_logger().warning(
                        result.message
                    )

                    return result

                if self.state == 'GOAL_REACHED':

                    self.stop_robot()
                    goal_handle.succeed()

                    result.success = True
                    result.message = (
                        'Trajectory target reached.'
                    )

                    self.get_logger().info(
                        result.message
                    )

                    return result

                feedback = (
                    ExecuteTrajectoryTarget.Feedback()
                )

                feedback.state = self.state

                feedback.distance_remaining = math.hypot(
                    self.goal_x - self.current_x,
                    self.goal_y - self.current_y
                )

                goal_handle.publish_feedback(feedback)

                # Wait for next 20 Hz control tick.
                self.action_tick_future = Future()
                tick_future = self.action_tick_future

                try:
                    await tick_future

                finally:
                    if self.action_tick_future is tick_future:
                        self.action_tick_future = None

            result.success = False
            result.message = (
                'ROS 2 stopped before target completion.'
            )

            goal_handle.abort()
            return result

        except Exception as exc:

            self.get_logger().error(
                f'Trajectory execution failed: {exc}'
            )

            result.success = False
            result.message = (
                f'Trajectory execution failed: {exc}'
            )

            if goal_handle.is_active:
                goal_handle.abort()

            return result

        finally:

            self.stop_robot()

            self.active_goal_handle = None
            self.active_movement_type = None
            self.action_start_yaw = None
            self.action_start_time = None
            self.action_use_target_yaw = False
            self.action_tick_future = None

            self.state = 'WAITING_FOR_GOAL'

    # =========================================================
    # Action movement controller
    #
    # UPDATED:
    # Stage 1: Translation
    # Stage 2: Rotation
    # Stage 3: Goal reached
    # =========================================================

    def control_action_goal(self):

        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
        ):
            self.stop_robot()
            return

        if self.state == 'GOAL_REACHED':
            self.stop_robot()
            return

        movement_type = self.active_movement_type

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.hypot(dx, dy)

        error_x_robot, error_y_robot = (
            self.world_error_to_robot(dx, dy)
        )

        # Determine desired heading.
        desired_yaw = (
            self.goal_yaw
            if (
                movement_type == 'rotate'
                or self.action_use_target_yaw
            )
            else self.action_start_yaw
        )

        yaw_error = self.normalize_angle(
            desired_yaw - self.current_yaw
        )

        position_reached = (
            distance <= self.position_tolerance
        )

        yaw_reached = (
            abs(yaw_error) <= self.yaw_tolerance
        )

        # =====================================================
        # Rotation-only action
        # =====================================================

        if movement_type == 'rotate':

            if yaw_reached:

                self.state = 'GOAL_REACHED'
                self.stop_robot()
                return

            omega = self.clamp(
                self.kp_yaw * yaw_error,
                -self.max_omega,
                self.max_omega
            )

            self.publish_cmd_vel(
                0.0,
                0.0,
                omega
            )

            return

        # =====================================================
        # Goal reached
        # =====================================================

        if position_reached and yaw_reached:

            self.state = 'GOAL_REACHED'
            self.stop_robot()
            return

        # =====================================================
        # Translation + rotation actions
        # =====================================================

        if movement_type in (
            'forward',
            'shift',
            'diagonal',
            'omnidirectional',
            'combined'
        ):

            vx = 0.0
            vy = 0.0

            # =================================================
            # STAGE 1: TRANSLATION ONLY
            # =================================================

            if not position_reached:

                if movement_type == 'forward':

                    vx = self.clamp(
                        self.kp_x * error_x_robot,
                        0.0,
                        self.max_vx
                    )

                elif movement_type == 'shift':

                    vy = self.clamp(
                        self.kp_y * error_y_robot,
                        -self.max_vy,
                        self.max_vy
                    )

                elif movement_type == 'diagonal':

                    vx = self.clamp(
                        self.kp_x * error_x_robot,
                        -self.max_vx,
                        self.max_vx
                    )

                    vy = self.clamp(
                        self.kp_y * error_y_robot,
                        -self.max_vy,
                        self.max_vy
                    )

                else:

                    # Omnidirectional and combined:
                    # translate along both axes.
                    vx = self.clamp(
                        self.kp_x * error_x_robot,
                        -self.max_vx,
                        self.max_vx
                    )

                    vy = self.clamp(
                        self.kp_y * error_y_robot,
                        -self.max_vy,
                        self.max_vy
                    )

                # Keep angular velocity zero during translation.
                self.publish_cmd_vel(
                    vx,
                    vy,
                    0.0
                )

                return

            # =================================================
            # STAGE 2: ROTATION ONLY
            # =================================================

            if not yaw_reached:

                omega = self.clamp(
                    self.kp_yaw * yaw_error,
                    -self.max_omega,
                    self.max_omega
                )

                self.publish_cmd_vel(
                    0.0,
                    0.0,
                    omega
                )

                return

            # =================================================
            # STAGE 3: GOAL REACHED
            # =================================================

            self.state = 'GOAL_REACHED'
            self.stop_robot()
            return

        self.stop_robot()

    # =========================================================
    # Main control loop
    # =========================================================

    def control_loop(self):

        # Wake the action callback at 20 Hz.
        if (
            self.action_tick_future is not None
            and not self.action_tick_future.done()
        ):
            self.action_tick_future.set_result(None)
            self.action_tick_future = None

        # Action control has priority.
        if self.active_goal_handle is not None:

            self.control_action_goal()
            return

        if self.state == 'WAITING_FOR_GOAL':

            self.stop_robot()
            return

        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
        ):
            self.stop_robot()
            return

        # =====================================================
        # Legacy: Rotate toward travel direction
        # =====================================================

        if self.state == 'ROTATE_TO_TRAVEL':

            travel_error = self.normalize_angle(
                self.travel_yaw - self.current_yaw
            )

            if abs(travel_error) <= self.yaw_tolerance:

                self.state = 'ALIGN_STEERING'
                self.stop_robot()
                return

            omega = self.clamp(
                self.kp_yaw * travel_error,
                -self.max_omega,
                self.max_omega
            )

            self.publish_cmd_vel(
                0.0,
                0.0,
                omega
            )

            return

        # =====================================================
        # Legacy: Steering alignment
        # =====================================================

        if self.state == 'ALIGN_STEERING':

            self.stop_robot()

            if (
                self.front_steer_angle is None
                or self.rear_steer_angle is None
            ):
                return

            front_error = self.normalize_angle(
                self.front_steer_angle
            )

            rear_error = self.normalize_angle(
                self.rear_steer_angle
            )

            if (
                abs(front_error) <= self.steering_tolerance
                and abs(rear_error) <= self.steering_tolerance
            ):
                self.state = 'MOVE_STRAIGHT'

            return

        # =====================================================
        # Legacy: Position control
        # =====================================================

        if self.state == 'MOVE_STRAIGHT':

            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y

            distance = math.hypot(dx, dy)

            if distance <= self.position_tolerance:

                self.stop_robot()

                if self.goal_mode == 'sideways':
                    self.state = 'GOAL_REACHED'
                else:
                    self.state = 'ROTATE_TO_GOAL'

                return

            error_x_robot, error_y_robot = (
                self.world_error_to_robot(dx, dy)
            )

            vx = self.clamp(
                self.kp_x * error_x_robot,
                -self.max_vx,
                self.max_vx
            )

            vy = self.clamp(
                self.kp_y * error_y_robot,
                -self.max_vy,
                self.max_vy
            )

            self.publish_cmd_vel(
                vx,
                vy,
                0.0
            )

            return

        # =====================================================
        # Legacy: Final orientation
        # =====================================================

        if self.state == 'ROTATE_TO_GOAL':

            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y

            distance = math.hypot(dx, dy)

            yaw_error = self.normalize_angle(
                self.goal_yaw - self.current_yaw
            )

            if (
                distance <= self.position_tolerance
                and abs(yaw_error) <= self.yaw_tolerance
            ):

                self.state = 'GOAL_REACHED'
                self.stop_robot()
                return

            omega = self.clamp(
                self.kp_yaw * yaw_error,
                -self.max_omega,
                self.max_omega
            )

            self.publish_cmd_vel(
                0.0,
                0.0,
                omega
            )

            return

        if self.state == 'GOAL_REACHED':
            self.stop_robot()


def main(args=None):

    rclpy.init(args=args)

    node = GoalPoseController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
