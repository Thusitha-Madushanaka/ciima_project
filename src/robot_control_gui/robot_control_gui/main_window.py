import sys
import math
import json
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from trajectory_interfaces.action import ExecuteTrajectoryTarget
from action_msgs.msg import GoalStatus

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QProgressBar, QScrollArea,
    QVBoxLayout, QWidget, QPlainTextEdit,
)

POSES_FILE = Path.home() / 'ros2_ws/trajectory_config/poses.json'

DESTINATIONS = {
    'A': 'move_to_A',
    'B': 'move_to_B',
    'C': 'move_to_C',
    'D': 'move_to_D',
    'E': 'move_to_E',
    'F': 'move_to_F',
    'G': 'move_to_G',
    'Home': 'move_to_home',
}

STYLESHEET = '''
                QMainWindow, QWidget#root, QScrollArea, QWidget#scrollContent {
                    background: #101010; color: #eeeeee;
                }
                QScrollArea { border: none; }
                QFrame#panel {
                    background: #101010;
                    border: 1px solid #343434;
                    border-radius: 15px;
                }
                QLabel {
                    color: #eeeeee;
                    background: transparent;
                }
                QLabel#muted {
                    color: #aaaaaa;
                }
                QLabel#metric {
                    font-size: 24px;
                    font-weight: 700;
                }
                QLabel#sectionTitle {
                    font-size: 19px;
                    font-weight: 650;
                }
                QPushButton {
                    background: #222222;
                    color: #f3f3f3;
                    border: 1px solid #404040;
                    border-radius: 9px;
                    font-size: 15px;
                    padding: 12px;
                }
                QPushButton:hover {
                    background: #303030;
                    border-color: #777777;
                }
                QPushButton:pressed {
                    background: #3a3a3a;
                }
                QPushButton:disabled {
                    color: #777777;
                    background: #1a1a1a;
                    border-color: #292929;
                }
                QPushButton#stopButton {
                    background: #3d1919;
                    border: 1px solid #5c2727;
                    font-weight: 700;
                    font-size: 17px;
                }
                QPushButton#stopButton:hover {
                    background: #592121;
                }
                QProgressBar {
                    border: none;
                    background: #282828;
                    border-radius: 6px;
                    min-height: 12px;
                    max-height: 12px;
                    text-align: center;
                }
                QProgressBar::chunk {
                    background: #21a56b;
                    border-radius: 6px;
                }
                QPlainTextEdit {
                    background: #101010;
                    border: none;
                    color: #bcbcbc;
                    font-family: monospace;
                    font-size: 12px;
                }
                '''


def make_label(text, name=None, size=None):
    label = QLabel(text)

    if name:
        label.setObjectName(name)

    if size:
        label.setStyleSheet(f'font-size: {size}px;')

    return label


def make_panel(title):
    frame = QFrame()
    frame.setObjectName('panel')

    layout = QVBoxLayout(frame)
    layout.setContentsMargins(17, 16, 17, 16)
    layout.setSpacing(12)

    layout.addWidget(make_label(title, 'sectionTitle'))

    return frame, layout



# ROS 2 COMMUNICATION NODE


class RobotGuiNode(Node):

    def __init__(
        self,
        position_cb,
        status_cb,
        feedback_cb,
        event_cb
    ):
        super().__init__('robot_control_gui')

        self.position_cb = position_cb
        self.status_cb = status_cb
        self.feedback_cb = feedback_cb
        self.event_cb = event_cb

        self.action_client = ActionClient(
            self,
            ExecuteTrajectoryTarget,
            '/execute_trajectory_target'
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_active = False
        self.goal_handle = None
        self.goal_future = None
        self.result_future = None
        self.cancel_future = None
        self.cancel_requested = False
        self.current_destination = None
        self.started_at = None
        self.target_distance = None
        self.cancel_response_pending = False

    
    # Receive robot odometry
    

    def odom_callback(self, msg):

        q = msg.pose.pose.orientation

        sin_yaw = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cos_yaw = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        yaw = math.degrees(
            math.atan2(sin_yaw, cos_yaw)
        )

        self.position_cb(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        )

    
    # Read destination from poses.json
    

    def load_destination(self, destination):

        with POSES_FILE.open(
            'r',
            encoding='utf-8'
        ) as file:

            poses = json.load(file)['poses']

        pose = poses[DESTINATIONS[destination]]

        x = float(pose['x'])
        y = float(pose['y'])
        yaw = float(pose.get('yaw', 0.0))

        if not all(
            math.isfinite(v) for v in (x, y, yaw)
        ):
            raise ValueError(
                'Non-finite target coordinates'
            )

        return x, y, yaw, 'yaw' in pose

    
    # Send navigation goal
    

    def send_destination(self, destination):

        if self.goal_active:

            self.event_cb(
                'A GUI goal is already active'
            )

            return

        if not self.action_client.server_is_ready():

            self.status_cb(
                'ERROR',
                'Trajectory action server unavailable'
            )

            self.event_cb(
                'Action server unavailable'
            )

            return

        try:

            x, y, yaw, use_yaw = (
                self.load_destination(destination)
            )

        except (
            OSError,
            ValueError,
            KeyError,
            TypeError
        ) as exc:

            self.status_cb(
                'ERROR',
                f'Cannot load target: {exc}'
            )

            self.event_cb(
                f'Invalid target {destination}: {exc}'
            )

            return

        target = PoseStamped()

        target.header.frame_id = 'odom'

        target.header.stamp = (
            self.get_clock().now().to_msg()
        )

        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = 0.0

        target.pose.orientation.z = math.sin(
            yaw / 2.0
        )

        target.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        goal = ExecuteTrajectoryTarget.Goal()

        goal.target_pose = target
        goal.movement_type = 'omnidirectional'
        goal.use_target_yaw = use_yaw

        self.goal_active = True
        self.goal_handle = None
        self.cancel_requested = False
        self.cancel_response_pending = False
        self.current_destination = destination
        self.started_at = time.monotonic()
        self.target_distance = None

        self.status_cb(
            'SENDING',
            f'Sending goal to {destination}'
        )

        self.event_cb(
            f'Goal {destination} requested '
            f'({x:.2f}, {y:.2f})'
        )

        try:

            self.goal_future = (
                self.action_client.send_goal_async(
                    goal,
                    feedback_callback=self.on_feedback
                )
            )

            self.goal_future.add_done_callback(
                self.on_goal_response
            )

        except Exception as exc:

            self.finish_goal(
                'ERROR',
                f'Goal send failed: {exc}'
            )

    
    # Receive goal response
    

    def on_goal_response(self, future):

        try:

            handle = future.result()

            if not handle.accepted:

                self.finish_goal(
                    'REJECTED',
                    'Action server rejected the goal'
                )

                return

            self.goal_handle = handle

            self.event_cb(
                'Action server accepted goal'
            )

            self.result_future = (
                handle.get_result_async()
            )

            self.result_future.add_done_callback(
                self.on_goal_result
            )

            if self.cancel_requested:

                self.request_cancel()

            else:

                self.status_cb(
                    'MOVING',
                    f'Moving to {self.current_destination}'
                )

        except Exception as exc:

            self.finish_goal(
                'ERROR',
                f'Goal response failed: {exc}'
            )

    
    # Receive motion feedback
    

    def on_feedback(self, feedback_msg):

        if (
            not self.goal_active
            or self.cancel_requested
        ):
            return

        feedback = feedback_msg.feedback

        self.feedback_cb(
            self.current_destination,
            str(feedback.state),
            float(feedback.distance_remaining)
        )

    
    # Software stop
    

    def stop_motion(self):

        if not self.goal_active:

            self.event_cb(
                'No GUI goal to cancel; '
                'robot stop not confirmed'
            )

            self.status_cb(
                'NO GUI GOAL',
                'No GUI-controlled goal is active'
            )

            return

        if self.cancel_requested:

            self.event_cb(
                'Cancellation already requested'
            )

            return

        self.cancel_requested = True

        self.status_cb(
            'CANCEL PENDING',
            'Requesting action cancellation; '
            'stop not confirmed'
        )

        self.event_cb(
            'Software cancellation requested'
        )

        if self.goal_handle is not None:

            self.request_cancel()

    def request_cancel(self):

        if (
            self.goal_handle is None
            or self.cancel_response_pending
        ):
            return

        try:

            self.cancel_response_pending = True

            self.cancel_future = (
                self.goal_handle.cancel_goal_async()
            )

            self.cancel_future.add_done_callback(
                self.on_cancel_response
            )

        except Exception as exc:

            self.cancel_response_pending = False

            self.status_cb(
                'STOP ERROR',
                f'Cancellation request failed: {exc}'
            )

            self.event_cb(
                'Cancellation request failed; '
                'use independent stop'
            )

    def on_cancel_response(self, future):

        self.cancel_response_pending = False

        try:

            response = future.result()

            if response.goals_canceling:

                self.status_cb(
                    'CANCEL PENDING',
                    'Cancellation accepted; awaiting result'
                )

                self.event_cb(
                    'Action server accepted cancellation'
                )

            else:

                self.status_cb(
                    'STOP ERROR',
                    'Cancellation not accepted; '
                    'use independent stop'
                )

                self.event_cb(
                    'Cancellation not accepted'
                )

        except Exception as exc:

            self.status_cb(
                'STOP ERROR',
                f'Cancellation response failed: {exc}'
            )

            self.event_cb(
                'Cancellation response error'
            )

    
    # Receive final action result
    

    def on_goal_result(self, future):

        try:

            response = future.result()
            result = response.result

            if (
                response.status == GoalStatus.STATUS_SUCCEEDED
                and result.success
            ):

                self.finish_goal(
                    'REACHED',
                    result.message or 'Goal reached'
                )

            elif (
                response.status == GoalStatus.STATUS_CANCELED
            ):

                self.finish_goal(
                    'CANCELED',
                    (result.message or 'Action canceled')
                    + ' — verify robot is stationary'
                )

            else:

                self.finish_goal(
                    'ERROR',
                    result.message
                    or f'Action ended with status {response.status}'
                )

        except Exception as exc:

            self.finish_goal(
                'ERROR',
                f'Goal result failed: {exc}'
            )

    def finish_goal(self, status, message):

        elapsed = (
            time.monotonic() - self.started_at
            if self.started_at
            else None
        )

        self.goal_active = False
        self.goal_handle = None
        self.goal_future = None
        self.result_future = None
        self.cancel_future = None
        self.cancel_requested = False
        self.cancel_response_pending = False

        self.status_cb(
            status,
            message,
            elapsed
        )

        self.event_cb(
            f'{status}: {message}'
        )

        self.current_destination = None
        self.started_at = None
        self.target_distance = None



# GRAPHICAL USER INTERFACE


class RobotControlWindow(QMainWindow):

    def __init__(self):

        super().__init__()

        self.setWindowTitle(
            'CIIMA Mission Control System | InnoBoTics'
        )

        self.resize(1060, 900)
        self.setStyleSheet(STYLESHEET)

        self.last_odom_time = None
        self.last_connection_state = None
        self.current_goal = None
        self.last_distance = None
        self.last_elapsed = None
        self.last_status = 'IDLE'

        # Main scrollable window

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)

        content = QWidget()
        content.setObjectName('scrollContent')

        root = QVBoxLayout(content)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(16)

        
        # Header
        

        header = QHBoxLayout()
        heading = QVBoxLayout()

        heading.addWidget(
            make_label(
                'CIIMA Mission Control System',
                size=24
            )
        )

        heading.addWidget(
            make_label(
                'InnoBoTics',
                'muted'
            )
        )

        header.addLayout(heading)
        header.addStretch()

        self.connection_label = make_label(
            '● Waiting for /odom',
            size=14
        )

        header.addWidget(
            self.connection_label
        )

        root.addLayout(header)

        
        # Robot status
        

        stats = QGridLayout()
        stats.setHorizontalSpacing(20)

        for col, (name, value) in enumerate([
            ('Robot state', 'IDLE'),
            ('Active target', 'NONE'),
            ('Controller', 'CHECKING'),
        ]):

            cell = QVBoxLayout()

            cell.addWidget(
                make_label(name, 'muted')
            )

            label = make_label(
                value,
                'metric'
            )

            cell.addWidget(label)

            stats.addLayout(cell, 0, col)

            if col == 0:

                self.state_label = label

            elif col == 1:

                self.target_label = label

            else:

                self.controller_label = label

        root.addLayout(stats)

        
        # Navigation control: A, B, C, D, E, F, G
        

        nav_panel, nav = make_panel(
            'Navigation control'
        )

        grid = QGridLayout()
        grid.setSpacing(12)

        self.destination_buttons = {}

        for index, name in enumerate(
            ('A', 'B', 'C', 'D', 'E', 'F', 'G')
        ):

            button = QPushButton(
                f'Move to {name}'
            )

            button.setMinimumHeight(57)

            button.clicked.connect(
                lambda checked=False, destination=name:
                self.move_to_destination(destination)
            )

            grid.addWidget(
                button,
                index // 2,
                index % 2
            )

            self.destination_buttons[name] = button

        # Return to Home

        self.home_button = QPushButton(
            'Return to Home'
        )

        self.home_button.setMinimumHeight(56)

        self.home_button.clicked.connect(
            lambda: self.move_to_destination('Home')
        )

        grid.addWidget(
            self.home_button,
            3,
            1
        )

        nav.addLayout(grid)

        # Software Stop

        self.stop_button = QPushButton(
            'EMERGENCY STOP'
        )

        self.stop_button.setObjectName(
            'stopButton'
        )

        self.stop_button.setMinimumHeight(65)

        self.stop_button.clicked.connect(
            self.stop_robot
        )

        nav.addWidget(
            self.stop_button
        )

        

        

        root.addWidget(nav_panel)

        
        # Live odometry
        

        metrics = QGridLayout()
        metrics.setSpacing(12)

        odom_panel, odom = make_panel(
            'Live odometry'
        )

        self.x_label = self.add_metric(
            odom,
            'X position',
            '-- m'
        )

        self.y_label = self.add_metric(
            odom,
            'Y position',
            '-- m'
        )

        self.yaw_label = self.add_metric(
            odom,
            'Yaw',
            '--°'
        )

        odom.addStretch()

        metrics.addWidget(
            odom_panel,
            0,
            0
        )

        
        # Motion progress
        

        progress_panel, progress = make_panel(
            'Motion progress'
        )

        self.progress_target = make_label(
            'Target: None',
            'muted'
        )

        progress.addWidget(
            self.progress_target
        )

        self.progress_state = make_label(
            'Waiting for goal',
            'metric'
        )

        progress.addWidget(
            self.progress_state
        )

        self.progress_bar = QProgressBar()

        self.progress_bar.setTextVisible(False)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)

        progress.addWidget(
            self.progress_bar
        )

        self.remaining_label = make_label(
            'Remaining: -- m',
            'muted'
        )

        progress.addWidget(
            self.remaining_label
        )

        self.message_label = make_label(
            'Ready to receive a destination',
            'muted'
        )

        self.message_label.setWordWrap(True)

        progress.addWidget(
            self.message_label
        )

        line = QFrame()

        line.setFrameShape(
            QFrame.Shape.HLine
        )

        line.setStyleSheet(
            'background: #333333; max-height: 1px;'
        )

        progress.addWidget(line)

        progress.addWidget(
            make_label(
                'Execution time',
                'muted'
            )
        )

        self.elapsed_label = make_label(
            '-- s',
            'metric'
        )

        progress.addWidget(
            self.elapsed_label
        )

        progress.addStretch()

        metrics.addWidget(
            progress_panel,
            0,
            1
        )

        metrics.setColumnStretch(0, 1)
        metrics.setColumnStretch(1, 1)

        root.addLayout(metrics)

        
        # Event log
        

        log_panel, log_layout = make_panel(
            'Event log'
        )

        self.log_box = QPlainTextEdit()

        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(135)

        self.log_box.document().setMaximumBlockCount(
            250
        )

        log_layout.addWidget(
            self.log_box
        )

        root.addWidget(log_panel)
        root.addStretch()

        scroll.setWidget(content)

        self.setCentralWidget(scroll)

        
        # Initialize ROS 2 node
        

        self.ros_node = RobotGuiNode(
            self.update_position,
            self.update_status,
            self.update_feedback,
            self.add_event,
        )

        self.add_event(
            'Dashboard started'
        )

        # ROS 2 processing timer

        self.ros_timer = QTimer(self)

        self.ros_timer.timeout.connect(
            self.process_ros_messages
        )

        self.ros_timer.start(50)

        # Connection and execution timer

        self.health_timer = QTimer(self)

        self.health_timer.timeout.connect(
            self.update_health
        )

        self.health_timer.start(500)

        self.update_health()

    
    # Create metric labels
    

    def add_metric(self, layout, name, initial):

        layout.addWidget(
            make_label(name, 'muted')
        )

        label = make_label(
            initial,
            'metric'
        )

        layout.addWidget(label)

        return label

    
    # Event logging
   

    def add_event(self, message):

        timestamp = datetime.now().strftime(
            '%H:%M:%S'
        )

        self.log_box.appendPlainText(
            f'{timestamp}  {message}'
        )

    
    # Destination button callback
    

    def move_to_destination(self, destination):

        self.ros_node.send_destination(
            destination
        )

    
    # Software stop callback
    

    def stop_robot(self):

        self.ros_node.stop_motion()

    
    # Update live robot position
    

    def update_position(self, x, y, yaw):

        self.last_odom_time = time.monotonic()

        self.x_label.setText(
            f'{x:.3f} m'
        )

        self.y_label.setText(
            f'{y:.3f} m'
        )

        self.yaw_label.setText(
            f'{yaw:.1f}°'
        )

    
    # Update robot state
    

    def update_status(
        self,
        status,
        message,
        elapsed=None
    ):

        self.last_status = status

        self.state_label.setText(status)

        self.progress_state.setText(
            status.title()
        )

        self.message_label.setText(message)

        active = self.ros_node.goal_active

        if active:

            self.current_goal = (
                self.ros_node.current_destination
            )

            self.target_label.setText(
                self.current_goal or 'PENDING'
            )

            self.progress_target.setText(
                f'Target: {self.current_goal or "Pending"}'
            )

            self.progress_bar.setRange(0, 0)

            self.last_distance = None

        elif status == 'REACHED':

            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(100)

            self.remaining_label.setText(
                'Remaining: 0.00 m (goal reported reached)'
            )

            self.target_label.setText('NONE')

        else:

            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(0)

            self.target_label.setText('NONE')

        if elapsed is not None:

            self.last_elapsed = elapsed

            self.elapsed_label.setText(
                f'{elapsed:.1f} s'
            )

        for button in self.destination_buttons.values():

            button.setEnabled(not active)

        self.home_button.setEnabled(
            not active
        )

    
    # Update motion feedback
    

    def update_feedback(
        self,
        destination,
        state,
        distance
    ):

        self.progress_target.setText(
            f'Target: {destination}'
        )

        self.progress_state.setText(
            state or 'Moving'
        )

        self.state_label.setText('MOVING')

        if math.isfinite(distance):

            self.last_distance = distance

            self.remaining_label.setText(
                f'Remaining: {distance:.3f} m'
            )

    # --------------------------------------------------------
    # Process ROS 2 messages
    # --------------------------------------------------------

    def process_ros_messages(self):

        try:

            rclpy.spin_once(
                self.ros_node,
                timeout_sec=0.0
            )

        except Exception as exc:

            self.add_event(
                f'ROS 2 processing error: {exc}'
            )

            self.ros_timer.stop()

            self.state_label.setText('ERROR')

    
    # Update connection and controller status
    

    def update_health(self):

        odom_ok = (
            self.last_odom_time is not None
            and time.monotonic()
            - self.last_odom_time <= 2.0
        )

        connection = (
            'Receiving /odom'
            if odom_ok
            else 'Odometry unavailable'
        )

        if connection != self.last_connection_state:

            self.last_connection_state = connection

            self.add_event(connection)

        if odom_ok:

            self.connection_label.setText(
                '● Connected: /odom'
            )

            self.connection_label.setStyleSheet(
                'color: #25c66f; font-size: 14px;'
            )

        else:

            self.connection_label.setText(
                '● /odom unavailable'
            )

            self.connection_label.setStyleSheet(
                'color: #e6a04c; font-size: 14px;'
            )

        ready = (
            self.ros_node.action_client.server_is_ready()
        )

        self.controller_label.setText(
            'READY' if ready else 'OFFLINE'
        )

        if (
            self.ros_node.goal_active
            and self.ros_node.started_at is not None
        ):

            elapsed = (
                time.monotonic()
                - self.ros_node.started_at
            )

            self.elapsed_label.setText(
                f'{elapsed:.1f} s'
            )

    
    # Close GUI
    

    def closeEvent(self, event):

        if self.ros_node.goal_active:

            self.add_event(
                'Cannot close with active GUI goal; '
                'stop safely first'
            )

            event.ignore()

            return

        self.ros_timer.stop()
        self.health_timer.stop()

        self.ros_node.destroy_node()

        event.accept()





def main():

    rclpy.init(args=None)

    app = QApplication(sys.argv)

    window = RobotControlWindow()

    window.show()

    exit_code = app.exec()

    if rclpy.ok():

        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':

    main()
