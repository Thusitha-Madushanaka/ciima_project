#!/usr/bin/env python3
# MIT License
# 
# Copyright (c) 2025 Innobotics s.r.l
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformBroadcaster
import math
from rclpy.duration import Duration

class NavigateToPoseClientTF(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client_tf')

        # --- Action client ---
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- TF broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parametri del goal
        self.goal_x = 2.236 #+ 4.0
        self.goal_y = -0.708
        self.goal_yaw_deg = 0.0  # rotazione in gradi rispetto a Z+
        self.ref_frame = 'constructa_map'

        # Pre-calcolo del quaternione
        yaw = math.radians(self.goal_yaw_deg)
        self.qz = math.sin(yaw / 2.0)
        self.qw = math.cos(yaw / 2.0)

        # Attendi la connessione al server action
        self.get_logger().info('Waiting for "navigate_to_pose" action server...')
        self._action_client.wait_for_server()

        # Invia la goal pose
        self.send_goal()

    def send_goal(self):
        """Invia una NavigateToPose action goal"""
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = self.ref_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = self.qz
        goal_msg.pose.pose.orientation.w = self.qw

        self.get_logger().info(
            f'Sending NavigateToPose goal to ({self.goal_x:.2f}, {self.goal_y:.2f}) yaw={self.goal_yaw_deg:.1f}°'
        )

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected 😕')
            return

        self.get_logger().info('NavigateToPose goal accepted ✅')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished with result code: {future.result().status}')
        self.get_logger().info('Result message: %s' % str(result))

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseClientTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
