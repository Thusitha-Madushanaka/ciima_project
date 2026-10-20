#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converte roll, pitch, yaw (rad) in quaternion (x, y, z, w)
    Convenzione standard ROS (RPY intrinseci ZYX).
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


class SimulateTFPub(Node):
    def __init__(self):
        super().__init__('simulate_tf_pub')

        # --- Parametri
        self.declare_parameter('parent_frame', 'constructa_map')
        self.declare_parameter('child_frame', 'odom')

        self.declare_parameter('x', 2.0)
        self.declare_parameter('y', -0.8)
        self.declare_parameter('z', 0.0)

        # Roll, pitch, yaw in radianti
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

        parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        z = self.get_parameter('z').get_parameter_value().double_value

        roll = self.get_parameter('roll').get_parameter_value().double_value
        pitch = self.get_parameter('pitch').get_parameter_value().double_value
        yaw = self.get_parameter('yaw').get_parameter_value().double_value

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        self.static_broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        # Pubblica una sola volta (static)
        self.static_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"Static TF pubblicata: {parent_frame} -> {child_frame} "
            f"t=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimulateTFPub()
    try:
        # Per i TF statici basta che il nodo rimanga vivo
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()