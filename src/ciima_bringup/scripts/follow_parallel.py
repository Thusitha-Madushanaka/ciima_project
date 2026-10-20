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


class FollowParallel(Node):
    def __init__(self):
        super().__init__('follow_parallel')

        # Listener TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.points_pub = self.create_publisher(Marker, '/lidar_points', 10)
        self.error_pub = self.create_publisher(ComputedErrors, '/computed_errors', 10)
        #self.front_dist = self.create_publisher(Float64, '/front_min_distance', 10)
        self.sub_lidar1 = Subscriber(self, LaserScan, '/lidar_1/scan_filtered')
        self.sub_lidar2 = Subscriber(self, LaserScan, '/lidar_2/scan_filtered')

        # synch the two lidars 50ms tolerance
        self.ts = ApproximateTimeSynchronizer([self.sub_lidar1, self.sub_lidar2], queue_size=5, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

        self.getsidelines_service = self.create_service(
            GetSideLines,
            'get_side_lines',
            self.get_side_lines_callback
        )

        #self.getfrontpoints_service = self.create_service(
            #GetFrontPoints,
            #'get_front_points',
            #self.get_front_points_callback
        #)

        self.v_ortho1_x = 0.0
        self.v_ortho1_y = 0.0
        self.v_ortho2_x = 0.0
        self.v_ortho2_y = 0.0
        self.ld1 = 0.0
        self.ld2 = 0.0

        self.error_angle_deg = 0.0
        self.error_lateral_m = 0.0

        #self.frontleftpoints = None
        #self.frontrightpoints = None

        #self.homeposx = 2.050
        #self.distswitchpolicy = 3.5

        #self.actual_pose_timer = self.create_timer(0.5, self.updateRobotToMap)
        #self.actual_robot_pose : TransformStamped = None



    def filter_by_rectangle(self, points : list[PointStamped], x_range, y_range):
        filtered = []

        for p in points:
            if x_range[0] <= p.point.x <= x_range[1] and y_range[0] <= p.point.y <= y_range[1]:
                filtered.append(p)

        return filtered
        

    # filter points based on a range
    def filter_lidar_points(self, msg: LaserScan,  downsample_factor: float, x_range=(-0.8, 0.8), y_range=(-10, 10)):
        try:
            t = self.tf_buffer.lookup_transform(
                'tracked_base_link',      # target frame
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
        marker.header.frame_id = 'tracked_base_link'
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
    
    # def filter_front_points(self, msg: LaserScan,  downsample_factor: float, x_range=(0.0, 10.0), y_rangeright=(-2.5, -0.8), y_rangeleft=(0.8, 2.5)):

    #     try:
    #         t = self.tf_buffer.lookup_transform(
    #             'tracked_robot_base_link',      # target frame
    #             msg.header.frame_id,      # source frame
    #             rclpy.time.Time())
    #     except TransformException as ex:
    #         self.get_logger().info(
    #             f'Could not transform {msg.header.frame_id} to base_link: {ex}')
    #         return

    #     # angle_min = math.radians(angle_min)
    #     # angle_max = math.radians(angle_max)
    #     points_stampedleft : list[PointStamped] = []
    #     points_stampedright : list[PointStamped] = []

    #     for i, distance in enumerate(msg.ranges):
    #         if distance < msg.range_max and distance > msg.range_min:

    #             # downsample the points ?
    #             if i % downsample_factor != 0:
    #                 continue
                
    #             # calculate the angle of the point
    #             angle = msg.angle_min + i * msg.angle_increment
                
    #             # transform the point to PointStamped
    #             p = PointStamped()
    #             p.header.stamp = msg.header.stamp
    #             p.header.frame_id = msg.header.frame_id
    #             p.point.x = distance * math.cos(angle)
    #             p.point.y = distance * math.sin(angle)
    #             p.point.z = 0

    #             # transform the point to tracked_base_link frame
    #             p_out = do_transform_point(p, t)

    #             if p_out.point.x > x_range[0] and p_out.point.x < x_range[1] and p_out.point.y > y_rangeleft[0] and p_out.point.y < y_rangeleft[1]:
    #                 points_stampedleft.append(p_out)

    #             if p_out.point.x > x_range[0] and p_out.point.x < x_range[1] and p_out.point.y > y_rangeright[0] and p_out.point.y < y_rangeright[1]:
    #                 points_stampedright.append(p_out)

    #     self.frontleftpoints = points_stampedleft
    #     self.frontrightpoints = points_stampedright

    # filter front points for distance from pieces based on the position on map
    # def filter_points_for_dist(self, msg: LaserScan,  downsample_factor: float, x_range=(-0.8, 0.8), y_range=(-10, 10)):

    #     self.publish_base_link_aligned()

    #     try:
    #         t = self.tf_buffer.lookup_transform(
    #             'tracked_base_aligned',      # target frame
    #             msg.header.frame_id,      # source frame
    #             rclpy.time.Time())
    #     except TransformException as ex:
    #         self.get_logger().info(
    #             f'Could not transform {msg.header.frame_id} to base_link: {ex}')
    #         return

    #     # angle_min = math.radians(angle_min)
    #     # angle_max = math.radians(angle_max)
    #     points_stamped : list[PointStamped] = []
        
    #     for i, distance in enumerate(msg.ranges):
    #         if distance < msg.range_max and distance > msg.range_min:

    #             # downsample the points ?
    #             if i % downsample_factor != 0:
    #                 continue
                
    #             # calculate the angle of the point
    #             angle = msg.angle_min + i * msg.angle_increment
                
    #             # transform the point to PointStamped
    #             p = PointStamped()
    #             p.header.stamp = msg.header.stamp
    #             p.header.frame_id = msg.header.frame_id
    #             p.point.x = distance * math.cos(angle)
    #             p.point.y = distance * math.sin(angle)
    #             p.point.z = 0

    #             # transform the point to tracked_base_link frame
    #             p_out = do_transform_point(p, t)
    #             points_stamped.append(p_out)

    #     # filter by range 
    #     points_filtered = self.filter_by_rectangle(points_stamped, x_range=x_range, y_range=y_range)

    #     # publish the points as a marker to visuailize in Rviz
    #     marker = Marker()
    #     marker.header.frame_id = 'tracked_base_aligned'
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = msg.header.frame_id
    #     marker.id = 0
    #     marker.type = Marker.POINTS
    #     marker.action = Marker.ADD
    #     marker.scale.x = 0.05
    #     marker.scale.y = 0.05
    #     marker.color.a = 1.0
    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0
    #     marker.points = [p.point for p in points_filtered]
    #     self.points_pub.publish(marker)

    #     return points_filtered

    def fit_parallel_lines_cv(self, points_list1: list[PointStamped], points_list2: list[PointStamped]):
        # convert to np.array
        points1 = np.array([[p.point.x, p.point.y] for p in points_list1])
        points2 = np.array([[p.point.x, p.point.y] for p in points_list2])

        # fit lines
        [vxa, vya, x0a, y0a] = cv2.fitLine(points1, cv2.DIST_L2, 0.0, 0.01, 0.01)
        [vxb, vyb, x0b, y0b] = cv2.fitLine(points2, cv2.DIST_L2, 0.0, 0.01, 0.01)
        import copy
        tha = np.arctan2(copy.deepcopy(vya), copy.deepcopy(vxa))
        thb = np.arctan2(copy.deepcopy(vyb), copy.deepcopy(vxb))

        return (vxa, vya, x0a, y0a, tha), (vxb, vyb, x0b, y0b, thb)

    # ortho_distance
    def ortho_distance(self, vx, vy, x0, y0):
        # compute the orthogonal versor to the line
        v_ortho = np.array([vy, -vx])

        # compute the orthogonal distance
        ld = np.dot(np.array([x0, y0]).T, v_ortho)
        return ld
    
    # implement the min frontal distance calculation
    # def calculate_min_frontal_distance(self, points_stamped_frontal: list[PointStamped], window_size=3):
    #     distances = [p.point.x for p in points_stamped_frontal]

    #     # check if there aren't enough points
    #     if len(distances) < window_size:
    #         return float(np.mean(distances))
        
    #     # mobile mean over `window_size` points
    #     averaged = [
    #         np.mean(distances[i:i + window_size])
    #         for i in range(0, len(distances), window_size)
    #     ]
    #     # return the minimun one
    #     return min(averaged)

    # calculate the corrections
    def sync_callback(self, msg1: LaserScan, msg2: LaserScan):

        # filter points angle-range-based
        points_l1 : list[PointStamped] = self.filter_lidar_points(msg=msg1, downsample_factor=1, x_range=(-0.6, 0.6), y_range=(-10, 10))
        points_l2 : list[PointStamped] = self.filter_lidar_points(msg=msg2, downsample_factor=1, x_range=(-0.6, 0.6), y_range=(-10, 10))
        #points_l2_front : list[PointStamped] = self.filter_lidar_points(msg=msg2, downsample_factor=1, x_range=(0.0, 10.0), y_range=(-1.0, 1.0))
        #points_l2_front : list[PointStamped] = self.filter_points_for_dist(msg=msg2, downsample_factor=1, x_range=(0.0, 10.0), y_range=(-1.0, 1.0))

        #self.filter_front_points(msg=msg2, downsample_factor=1, x_range=(0.0, 10.0), y_rangeright=(-1.5, -0.8), y_rangeleft=(0.8, 1.5))

        if not points_l1 or not points_l2 or len(points_l1) == 0 or len(points_l2) == 0:
            ld1_l = 10.0
            ld2_l = 10.0
            vy1 = 0.0
            vx1 = 0.0
            vx2 = 0.0
            vy2 = 0.0

            correction_angle_l = 0.0
            correction_dist_lateral = 0.0
            tha = 0.0
            thb = 0.0
        else:
            # fit the two lines with // constraint
            (vx1, vy1, x01, y01, tha), (vx2, vy2, x02, y02, thb) = self.fit_parallel_lines_cv(points_l1, points_l2)

            # angle
            theta = (tha + thb )/2

            # distances
            ld1_l = self.ortho_distance(vx1, vy1, x01, y01)
            ld2_l = self.ortho_distance(vx2, vy2, x02, y02)

            # angle correction
            correction_angle_l = math.degrees(theta)
            # lateral correction
            mean = (ld1_l - ld2_l) / 2.0
            correction_dist_lateral = mean - ld1_l 

        # if not points_l2_front or len(points_l2_front) == 0:
        #     ld2_f_min = 20.0
        # else:
        #     # frontal distance
        #     ld2_f_min = self.calculate_min_frontal_distance(points_l2_front)


        #salva valori per la service call se arriva
        self.v_ortho1_x = vy1
        self.v_ortho1_y = -vx1
        self.v_ortho2_x = vy2
        self.v_ortho2_y = -vx2
        self.ld1 = ld1_l
        self.ld2 = ld2_l


        # publish computed errors
        msg = ComputedErrors()
        msg.frame_id1 = msg1.header.frame_id
        msg.lateral_m1 = round(float(ld1_l), 3)
        msg.frontal_m1 = 0.0
        msg.angle_deg1 = round(float(math.degrees(tha)), 3)

        msg.frame_id2 = msg2.header.frame_id
        msg.lateral_m2 = round(float(ld2_l), 3)
        #msg.frontal_m2 = round(float(ld2_f_min), 3)
        msg.frontal_m2 = 0.0
        msg.angle_deg2 = round(float(math.degrees(thb)), 3)

        msg.error_angle_deg = round(float(correction_angle_l), 3)
        msg.error_lateral_m = round(float(correction_dist_lateral), 3)

        self.error_angle_deg = round(float(correction_angle_l), 3)
        self.error_lateral_m = round(float(correction_dist_lateral), 3)

        self.error_pub.publish(msg)

    #service callback
    def get_side_lines_callback(self, request, response):

        response.v_ortho1_x = float(self.v_ortho1_x)
        response.v_ortho1_y = float(self.v_ortho1_y)
        response.v_ortho2_x = float(self.v_ortho2_x)
        response.v_ortho2_y = float(self.v_ortho2_y)
        response.ld1 = float(self.ld1)
        response.ld2 = float(self.ld2)

        self.get_logger().info("Get side lines requested")
        return response
    
    # def get_front_points_callback(self, request, response):

    #     if self.frontleftpoints:
    #         response.frontleftpoints = self.frontleftpoints
    #     else:
    #         response.frontleftpoints = []

    #     if self.frontrightpoints:
    #         response.frontrightpoints = self.frontrightpoints
    #     else:
    #         response.frontrightpoints = []

    #     self.get_logger().info("Get front points requested")
    #     return response
    
    # def publish_base_link_aligned(self):

    #     if self.actual_robot_pose is not None:

    #         if (self.actual_robot_pose.transform.translation.x - self.homeposx) < self.distswitchpolicy: #fuori dal camion
                
    #             inv_transf = invert_transform(self.actual_robot_pose)
    #             t1 = TransformStamped()
    #             t1.header.stamp = self.get_clock().now().to_msg()
    #             t1.header.frame_id = "tracked_base_link"
    #             t1.child_frame_id = "tracked_base_aligned"
    #             t1.transform.translation.x = 0.0
    #             t1.transform.translation.y = 0.0
    #             t1.transform.translation.z = 0.0
    #             t1.transform.rotation = inv_transf.transform.rotation
    #             self.tf_broadcaster.sendTransform(t1)

    #         else:  #dentro al camion

    #             t1 = TransformStamped()
    #             t1.header.stamp = self.get_clock().now().to_msg()
    #             t1.header.frame_id = "tracked_base_link"
    #             t1.child_frame_id = "tracked_base_aligned"
    #             t1.transform.translation.x = self.error_lateral_m * math.sin(math.radians(-self.error_angle_deg))
    #             t1.transform.translation.y = self.error_lateral_m * math.cos(math.radians(-self.error_angle_deg))
    #             t1.transform.translation.z = 0.0
    #             quat = yaw_to_quaternion(math.radians(self.error_angle_deg))
    #             t1.transform.rotation.x = float(quat[0])
    #             t1.transform.rotation.y = float(quat[1])
    #             t1.transform.rotation.z = float(quat[2])
    #             t1.transform.rotation.w = float(quat[3])
    #             self.tf_broadcaster.sendTransform(t1)             
    #     else:

    #         t1 = TransformStamped()
    #         t1.header.stamp = self.get_clock().now().to_msg()
    #         t1.header.frame_id = "tracked_base_link"
    #         t1.child_frame_id = "tracked_base_aligned"
    #         t1.transform.translation.x = 0.0
    #         t1.transform.translation.y = 0.0
    #         t1.transform.translation.z = 0.0
    #         t1.transform.rotation.x = 0.0
    #         t1.transform.rotation.y = 0.0
    #         t1.transform.rotation.z = 0.0
    #         t1.transform.rotation.w = 1.0

    #         self.tf_broadcaster.sendTransform(t1)

    # def updateRobotToMap(self):
    #     try:
    #         t = self.tf_buffer.lookup_transform(
    #             'constructa_map',    
    #             'tracked_base_link',    
    #             rclpy.time.Time())
    #         self.actual_robot_pose = t
    #     except TransformException as ex:
    #         self.get_logger().error(
    #             f'Could not transform constructa_map to tracked_base_link: {ex}')
    #         self.actual_robot_pose = None

def main(args=None):
    rclpy.init(args=args)
    node = FollowParallel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
