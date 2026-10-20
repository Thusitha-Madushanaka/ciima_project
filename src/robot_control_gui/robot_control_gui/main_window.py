
import sys
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QProgressBar,
    QVBoxLayout,
    QWidget,
)


class OdometrySubscriber(Node):
    """ROS 2 node that receives the robot's odometry."""

    def __init__(self, update_position_callback):
        super().__init__("robot_control_gui")

        self.update_position_callback = update_position_callback

        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

    def odom_callback(self, msg):
        # Read the robot's position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Read orientation as a quaternion
        q = msg.pose.pose.orientation

        # Convert quaternion to yaw angle
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        yaw = math.atan2(sin_yaw, cos_yaw)
        yaw_degrees = math.degrees(yaw)

        # Send the values to the dashboard
        self.update_position_callback(x, y, yaw_degrees)


class RobotControlWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        # Initialize ROS 2
        if not rclpy.ok():
            rclpy.init(args=None)

        self.setWindowTitle("Robot Control Dashboard")
        self.resize(900, 700)

        # Main window layout
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)

        # Header
        title = QLabel("InnoBoTics | Robot Control Dashboard")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "font-size: 24px; font-weight: bold; padding: 8px;"
        )
        main_layout.addWidget(title)

        # Connection and robot status
        status_layout = QHBoxLayout()

        self.connection_label = QLabel("ROS 2: Checking...")
        self.robot_status_label = QLabel("Robot Status: Waiting for odometry")

        status_layout.addWidget(self.connection_label)
        status_layout.addStretch()
        status_layout.addWidget(self.robot_status_label)

        main_layout.addLayout(status_layout)

        # Robot navigation buttons
        destination_group = QGroupBox("Robot Navigation")
        destination_layout = QGridLayout(destination_group)

        self.destination_buttons = {}

        destinations = ["A", "B", "C", "D"]

        for index, destination in enumerate(destinations):
            button = QPushButton(f"Move to {destination}")
            button.setMinimumHeight(55)

            # Placeholder only: no robot command is sent
            button.clicked.connect(
                lambda checked=False, name=destination:
                self.show_message(
                    f"Destination {name} selected — no command sent"
                )
            )

            destination_layout.addWidget(
                button, index // 2, index % 2
            )

            self.destination_buttons[destination] = button

        self.home_button = QPushButton("Return to Home")
        self.home_button.setMinimumHeight(45)
        self.home_button.clicked.connect(
            lambda: self.show_message(
                "Home selected — no command sent"
            )
        )

        destination_layout.addWidget(self.home_button, 2, 0, 1, 2)
        main_layout.addWidget(destination_group)

        # Emergency stop placeholder
        self.stop_button = QPushButton("EMERGENCY STOP")
        self.stop_button.setMinimumHeight(55)
        self.stop_button.setStyleSheet(
            "background-color: #B42318;"
            "color: white;"
            "font-weight: bold;"
            "font-size: 16px;"
        )
        self.stop_button.clicked.connect(
            lambda: self.show_message(
                "GUI placeholder only — no stop command sent"
            )
        )
        main_layout.addWidget(self.stop_button)

        # Robot position from /odom
        odom_group = QGroupBox("Robot Position (/odom)")
        odom_layout = QGridLayout(odom_group)

        self.x_label = QLabel("X: Waiting for data...")
        self.y_label = QLabel("Y: Waiting for data...")
        self.yaw_label = QLabel("Yaw: Waiting for data...")

        odom_layout.addWidget(self.x_label, 0, 0)
        odom_layout.addWidget(self.y_label, 0, 1)
        odom_layout.addWidget(self.yaw_label, 0, 2)

        main_layout.addWidget(odom_group)

        # Trajectory progress
        progress_group = QGroupBox("Trajectory Progress")
        progress_layout = QVBoxLayout(progress_group)

        self.progress_label = QLabel("Waiting for trajectory")
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)

        progress_layout.addWidget(self.progress_label)
        progress_layout.addWidget(self.progress_bar)

        main_layout.addWidget(progress_group)

        main_layout.addStretch()
        self.setCentralWidget(central_widget)

        # Create the ROS 2 odometry subscriber
        self.odom_node = OdometrySubscriber(self.update_position)

        # Let Qt process ROS 2 messages regularly
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.process_ros_messages)
        self.ros_timer.start(50)

    def update_position(self, x, y, yaw_degrees):
        """Update the position labels with the latest odometry."""
        self.x_label.setText(f"X: {x:.3f} m")
        self.y_label.setText(f"Y: {y:.3f} m")
        self.yaw_label.setText(f"Yaw: {yaw_degrees:.2f}°")

        self.connection_label.setText("ROS 2: Receiving /odom")
        self.robot_status_label.setText("Robot Status: Odometry received")

    def process_ros_messages(self):
        """Process pending ROS 2 messages without blocking Qt."""
        rclpy.spin_once(self.odom_node, timeout_sec=0.0)

    def show_message(self, message):
        self.progress_label.setText(message)

    def closeEvent(self, event):
        """Clean up ROS 2 resources when the window closes."""
        self.ros_timer.stop()
        self.odom_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        event.accept()


def main():
    rclpy.init(args=None)

    app = QApplication(sys.argv)
    window = RobotControlWindow()
    window.show()

    exit_code = app.exec()

    # Ensure ROS 2 shuts down when the application exits
    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
