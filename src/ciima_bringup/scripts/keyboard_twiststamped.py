#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class KeyboardTwistStamped(Node):
    def __init__(self):
        super().__init__('keyboard_twiststamped')

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.speed = 0.5
        self.turn = 1.0

        self.get_logger().info("Teleop TwistStamped attivo: W A S D, X stop, Q esci")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        twist = TwistStamped()
        twist.header.frame_id = "base_link"

        while rclpy.ok():
            key = self.get_key()

            if key == 'w':
                twist.twist.linear.x = self.speed
            elif key == 's':
                twist.twist.linear.x = -self.speed
            elif key == 'a':
                twist.twist.angular.z = self.turn
            elif key == 'd':
                twist.twist.angular.z = -self.turn
            elif key == 'x':
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = 0.0
            elif key == 'q':
                break
            else:
                # ritorno graduale a zero
                twist.twist.linear.x *= 0.9
                twist.twist.angular.z *= 0.9

            twist.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(twist)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = KeyboardTwistStamped()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()