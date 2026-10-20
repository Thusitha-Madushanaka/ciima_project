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
import math
import numpy as np
import cv2

from geometry_msgs.msg import PoseStamped,PointStamped, Point, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Float64
from message_filters import Subscriber, ApproximateTimeSynchronizer
from scipy.optimize import minimize
from rclpy.duration import Duration

from ibt_ros2_interfaces.msg import ComputedErrors
from mengozzi_interfaces.srv import GetSideLines, GetFrontPoints

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw

def yaw_to_quaternion(yaw):
    half_yaw = yaw / 2.0
    qx = 0.0
    qy = 0.0
    qz = np.sin(half_yaw)
    qw = np.cos(half_yaw)
    return (qx, qy, qz, qw)

def invert_transform(trans: TransformStamped) -> TransformStamped:
    inv = TransformStamped()

    # Scambia i frame
    inv.header.frame_id = trans.child_frame_id
    inv.child_frame_id = trans.header.frame_id

    # Mantieni il tempo
    inv.header.stamp = trans.header.stamp

    # --- Estrai traslazione e rotazione
    t = np.array([
        trans.transform.translation.x,
        trans.transform.translation.y,
        trans.transform.translation.z
    ])
    q = np.array([
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w
    ])

    # --- Inverti la rotazione (quaternion conjugate)
    q_inv = np.array([-q[0], -q[1], -q[2], q[3]])

    # --- Ruota e inverte la traslazione
    # calcola matrice di rotazione 3x3
    def quat_to_rot(q):
        x, y, z, w = q
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
    R = quat_to_rot(q)
    t_inv = -R.T @ t  # traslazione inversa

    # --- Scrivi il risultato
    inv.transform.translation.x = float(t_inv[0])
    inv.transform.translation.y = float(t_inv[1])
    inv.transform.translation.z = float(t_inv[2])

    inv.transform.rotation.x = float(q_inv[0])
    inv.transform.rotation.y = float(q_inv[1])
    inv.transform.rotation.z = float(q_inv[2])
    inv.transform.rotation.w = float(q_inv[3])

    return inv


class FrontDistance(Node):
    def __init__(self):
        super().__init__('front_distance')

        # Listener TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.points_pub = self.create_publisher(Marker, '/lidar_points', 10)
        self.front_dist_pub = self.create_publisher(Float64, '/front_min_distance', 10)

        self.ts = self.create_subscription(LaserScan, '/lidar_2/scan_filtered', self.laserscan_callback, 10)
        self.create_subscription(ComputedErrors, '/computed_errors', self.computed_errors_cb, 5)

        self.getfrontpoints_service = self.create_service(
            GetFrontPoints,
            'get_front_points',
            self.get_front_points_callback
        )

        self.frontleftpoints = None
        self.frontrightpoints = None

        self.homeposx = 2.050
        self.distswitchpolicy = 3.5

        self.actual_pose_timer = self.create_timer(0.5, self.updateRobotToMap)
        self.actual_robot_pose : TransformStamped = None

        self.computed_errors = None


    def filter_by_rectangle(self, points : list[PointStamped], x_range, y_range):
        filtered = []

        for p in points:
            if x_range[0] <= p.point.x <= x_range[1] and y_range[0] <= p.point.y <= y_range[1]:
                filtered.append(p)

        return filtered
        
    # filter points based on a range
    def filter_front_points(self, msg: LaserScan,  downsample_factor: float, x_range=(0.0, 10.0), y_rangeright=(-2.5, -0.8), y_rangeleft=(0.8, 2.5)):

        try:
            t = self.tf_buffer.lookup_transform(
                'tracked_robot_base_link',      # target frame
                msg.header.frame_id,      # source frame
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {msg.header.frame_id} to base_link: {ex}')
            return

        # angle_min = math.radians(angle_min)
        # angle_max = math.radians(angle_max)
        points_stampedleft : list[PointStamped] = []
        points_stampedright : list[PointStamped] = []

        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_max and distance > msg.range_min:

                # downsample the points ?
                if i % downsample_factor != 0:
                    continue
                
                # calculate the angle of the point
                angle = msg.angle_min + i * msg.angle_increment
                
                # transform the point to PointStamped
                p = PointStamped()
                p.header.stamp = msg.header.stamp
                p.header.frame_id = msg.header.frame_id
                p.point.x = distance * math.cos(angle)
                p.point.y = distance * math.sin(angle)
                p.point.z = 0

                # transform the point to tracked_base_link frame
                p_out = do_transform_point(p, t)

                if p_out.point.x > x_range[0] and p_out.point.x < x_range[1] and p_out.point.y > y_rangeleft[0] and p_out.point.y < y_rangeleft[1]:
                    points_stampedleft.append(p_out)

                if p_out.point.x > x_range[0] and p_out.point.x < x_range[1] and p_out.point.y > y_rangeright[0] and p_out.point.y < y_rangeright[1]:
                    points_stampedright.append(p_out)

        self.frontleftpoints = points_stampedleft
        self.frontrightpoints = points_stampedright

    # filter front points for distance from pieces based on the position on map
    def filter_points_for_dist(self, msg: LaserScan,  downsample_factor: float, x_range=(-0.8, 0.8), y_range=(-10, 10)):

        self.publish_base_link_aligned()

        try:
            t = self.tf_buffer.lookup_transform(
                'tracked_base_aligned',      # target frame
                msg.header.frame_id,      # source frame
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {msg.header.frame_id} to base_link: {ex}')
            return

        # angle_min = math.radians(angle_min)
        # angle_max = math.radians(angle_max)
        points_stamped : list[PointStamped] = []
        
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_max and distance > msg.range_min:

                # downsample the points ?
                if i % downsample_factor != 0:
                    continue
                
                # calculate the angle of the point
                angle = msg.angle_min + i * msg.angle_increment
                
                # transform the point to PointStamped
                p = PointStamped()
                p.header.stamp = msg.header.stamp
                p.header.frame_id = msg.header.frame_id
                p.point.x = distance * math.cos(angle)
                p.point.y = distance * math.sin(angle)
                p.point.z = 0

                # transform the point to tracked_base_link frame
                p_out = do_transform_point(p, t)
                points_stamped.append(p_out)

        # filter by range 
        points_filtered = self.filter_by_rectangle(points_stamped, x_range=x_range, y_range=y_range)

        # publish the points as a marker to visuailize in Rviz
        marker = Marker()
        marker.header.frame_id = 'tracked_base_aligned'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = msg.header.frame_id
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = [p.point for p in points_filtered]
        self.points_pub.publish(marker)

        return points_filtered
    
    # implement the min frontal distance calculation
    def calculate_min_frontal_distance(self, points_stamped_frontal: list[PointStamped], window_size=3):
        distances = [p.point.x for p in points_stamped_frontal]

        # check if there aren't enough points
        if len(distances) < window_size:
            return float(np.mean(distances))
        
        # mobile mean over `window_size` points
        averaged = [
            np.mean(distances[i:i + window_size])
            for i in range(0, len(distances), window_size)
        ]
        # return the minimun one
        return min(averaged)

    # calculate the corrections
    def laserscan_callback(self, msg: LaserScan):

        # filter points angle-range-based
        points_l2_front : list[PointStamped] = self.filter_points_for_dist(msg=msg, downsample_factor=1, x_range=(0.0, 10.0), y_range=(-1.0, 1.0))
        self.filter_front_points(msg=msg, downsample_factor=1, x_range=(0.0, 10.0), y_rangeright=(-1.5, -0.8), y_rangeleft=(0.8, 1.5))

        if not points_l2_front or len(points_l2_front) == 0:
            ld2_f_min = 20.0
        else:
            # frontal distance
            ld2_f_min = self.calculate_min_frontal_distance(points_l2_front)


        # publish distance forward
        msg1 = Float64()
        msg1.data = ld2_f_min
        self.front_dist_pub.publish(msg1)
   
    def get_front_points_callback(self, request, response):

        if self.frontleftpoints:
            response.frontleftpoints = self.frontleftpoints
        else:
            response.frontleftpoints = []

        if self.frontrightpoints:
            response.frontrightpoints = self.frontrightpoints
        else:
            response.frontrightpoints = []

        self.get_logger().info("Get front points requested")
        return response
    
    def publish_base_link_aligned(self):

        if self.actual_robot_pose is not None:

            if (self.actual_robot_pose.transform.translation.x - self.homeposx) < self.distswitchpolicy: #fuori dal camion
                
                inv_transf = invert_transform(self.actual_robot_pose)
                t1 = TransformStamped()
                t1.header.stamp = self.get_clock().now().to_msg()
                t1.header.frame_id = "tracked_base_link"
                t1.child_frame_id = "tracked_base_aligned"
                t1.transform.translation.x = 0.0
                t1.transform.translation.y = 0.0
                t1.transform.translation.z = 0.0
                t1.transform.rotation = inv_transf.transform.rotation
                self.tf_broadcaster.sendTransform(t1)

            else:  #dentro al camion

                if self.computed_errors:
                    t1 = TransformStamped()
                    t1.header.stamp = self.get_clock().now().to_msg()
                    t1.header.frame_id = "tracked_base_link"
                    t1.child_frame_id = "tracked_base_aligned"
                    t1.transform.translation.x = self.computed_errors.error_lateral_m * math.sin(math.radians(-self.computed_errors.error_angle_deg))
                    t1.transform.translation.y = self.computed_errors.error_lateral_m * math.cos(math.radians(-self.computed_errors.error_angle_deg))
                    t1.transform.translation.z = 0.0
                    quat = yaw_to_quaternion(math.radians(self.computed_errors.error_angle_deg))
                    t1.transform.rotation.x = float(quat[0])
                    t1.transform.rotation.y = float(quat[1])
                    t1.transform.rotation.z = float(quat[2])
                    t1.transform.rotation.w = float(quat[3])
                    self.tf_broadcaster.sendTransform(t1) 
                else:
                    t1 = TransformStamped()
                    t1.header.stamp = self.get_clock().now().to_msg()
                    t1.header.frame_id = "tracked_base_link"
                    t1.child_frame_id = "tracked_base_aligned"
                    t1.transform.translation.x = 0.0
                    t1.transform.translation.y = 0.0
                    t1.transform.translation.z = 0.0
                    t1.transform.rotation.x = 0.0
                    t1.transform.rotation.y = 0.0
                    t1.transform.rotation.z = 0.0
                    t1.transform.rotation.w = 1.0
       
        else:

            t1 = TransformStamped()
            t1.header.stamp = self.get_clock().now().to_msg()
            t1.header.frame_id = "tracked_base_link"
            t1.child_frame_id = "tracked_base_aligned"
            t1.transform.translation.x = 0.0
            t1.transform.translation.y = 0.0
            t1.transform.translation.z = 0.0
            t1.transform.rotation.x = 0.0
            t1.transform.rotation.y = 0.0
            t1.transform.rotation.z = 0.0
            t1.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t1)

    def updateRobotToMap(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'constructa_map',    
                'tracked_base_link',    
                rclpy.time.Time())
            self.actual_robot_pose = t
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform constructa_map to tracked_base_link: {ex}')
            self.actual_robot_pose = None

    def computed_errors_cb(self, msg: ComputedErrors):
        self.computed_errors = msg

def main(args=None):
    rclpy.init(args=args)
    node = FrontDistance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
