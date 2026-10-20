#!/usr/bin/env python3
# MIT License
# 
# Copyright (c) 2025 Innobotics s.r.l & Constructa s.r.l
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

import halcon as ha
from pypylon import pylon

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose, Quaternion, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener, TransformException
from rclpy.time import Time
import math
import numpy as np

def q_multiply(q1, q2):
    # q = q1 * q2 (w,x,y,z) convenzione tf_transformations -> (x,y,z,w) in ROS msg
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return [x, y, z, w]

def q_normalize(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    return (q / n).tolist()

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

class CalltabTFPub(Node):
    def __init__(self):
        super().__init__('calltab_tf_pub')
        
        # Broadcaster static & dynamic
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Listener TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer_callback1 = self.create_timer(0.5, self.callback1) 

        self.init_camera()

        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_RGB8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        self.CameraParameters = ha.read_cam_par('/home/constructa/src/Image_Calibration/CamBasler/Internal/camparbasler')

        self.TmpCtrl_PlateDescription = '/home/constructa/src/caltab/caltabtestA1.descr'
        self.TmpCtrl_FindCalObjParNames = ['gap_tolerance','alpha','skip_find_caltab']
        self.TmpCtrl_FindCalObjParValues = [1.18,0.34,'false']

        self.CalibHandle = ha.create_calib_data('calibration_object', 1, 1)
        ha.set_calib_data_cam_param(self.CalibHandle, 0, [], self.CameraParameters)
        ha.set_calib_data_calib_object(self.CalibHandle, 0, self.TmpCtrl_PlateDescription)

        self.last_trans: TransformStamped = None

        self.get_logger().info("Node started.")


    def init_camera(self):

        try:
            CAMERA_SERIAL = "40213036"  # <-- il serial della tua camera

            tl_factory = pylon.TlFactory.GetInstance()
            devices = tl_factory.EnumerateDevices()

            # Cerca il device con quel serial
            for dev in devices:
                if dev.GetSerialNumber() == CAMERA_SERIAL:
                    self.camera = pylon.InstantCamera(tl_factory.CreateDevice(dev))
                    break
            else:
                raise ValueError("Camera con serial {} non trovata".format(CAMERA_SERIAL))

            # init alcon e camera
            #self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            self.camera.Open()
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

            #self.camera.ExposureTime.Value = 20000.0
            #self.camera.ExposureTime.Value = 10000.0
            self.camera.ExposureTime.Value = 5000.0
           # self.camera.ExposureTime.Value = 15000.0
            #self.camera.Gain.Value = 30.0
            self.camera.Gain.Value = 25.0

        except Exception as e:
            self.get_logger().error(f"Failed init camera: {e}")

    def destroy_node(self):
        self.get_logger().info("Destroy camera node.")

        self.camera.StopGrabbing()
        self.camera.Close()        
        
        super().destroy_node()


    def callback1(self):
        if self.camera is None:
            self.get_logger().error("Camera is null")
            self.init_camera()
            return
        try:
            if self.camera.IsGrabbing():    
                grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    image = self.converter.Convert(grabResult)
                else:
                    grabResult.Release()
                    self.get_logger().error("Failed acquiring caltab image")
                    return
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            self.camera.Close()
            self.camera = None
            return

        try:

            img_array = image.GetArray()

            img_h = ha.himage_from_numpy_array(img_array)

            ha.find_calib_object(img_h, self.CalibHandle, 0, 0, 0, self.TmpCtrl_FindCalObjParNames, self.TmpCtrl_FindCalObjParValues)
            _, _, _, CameraPose = ha.get_calib_data_observ_points(self.CalibHandle, 0, 0, 0)

            posequat = ha.pose_to_quat(CameraPose)
            # create and publish tf
            t1 = TransformStamped()
            t1.header.stamp = self.get_clock().now().to_msg()
            t1.header.frame_id = "tracked_camera_base_link"
            t1.child_frame_id = "caltab"
            t1.transform.translation.x = CameraPose[0]
            t1.transform.translation.y = CameraPose[1]
            t1.transform.translation.z = CameraPose[2]
            t1.transform.rotation.x = posequat[1]
            t1.transform.rotation.y = posequat[2]
            t1.transform.rotation.z = posequat[3]
            t1.transform.rotation.w = posequat[0]

            self.tf_broadcaster.sendTransform(t1)

            self.read_dynamic_tf()
                # release image
            grabResult.Release()
            return
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            grabResult.Release()

        try:
            if self.last_trans is not None:
                self.last_trans.header.stamp = self.get_clock().now().to_msg()
                self.tf_broadcaster.sendTransform(self.last_trans)
                self.get_logger().warn(f"Using old transform")
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")


    def read_dynamic_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'caltab', rclpy.time.Time()
            )

            # self.get_logger().info(
            #     f"✅ Transform at ({trans.transform.translation.x:.2f}, {trans.transform.translation.y:.2f})"
            # )

            trans.child_frame_id = 'caltab_static'
            trans.header.stamp = (self.get_clock().now() + Duration(seconds=1.0)).to_msg()
            #trans.header.stamp = (self.get_clock().now()).to_msg()
            q = trans.transform.rotation
            quat_orig = [q.x, q.y, q.z, q.w]

            q_delta = [-0.5, 0.5, 0.5, 0.5]
            q_new = q_multiply(quat_orig, q_delta)
            q_new = q_normalize(q_new)

            # --- Aggiorna la trasformazione
            trans.transform.rotation.x = q_new[0]
            trans.transform.rotation.y = q_new[1]
            trans.transform.rotation.z = q_new[2]
            trans.transform.rotation.w = q_new[3]

            inve_transf = invert_transform(trans)
            inve_transf.header.frame_id = 'constructa_map'

            self.last_trans = inve_transf

            #self.static_broadcaster.sendTransform(inve_transf)
            self.tf_broadcaster.sendTransform(inve_transf)
        except TransformException as e:
            self.get_logger().warn(f"Transform not available: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CalltabTFPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
