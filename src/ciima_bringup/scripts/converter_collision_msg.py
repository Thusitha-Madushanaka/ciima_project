#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool


class CollisionToMarker(Node):
    def __init__(self):
        super().__init__('collision_to_marker')
        self.sub = self.create_subscription(
            CollisionObject,
            '/collision_object',
            self.collision_cb,
            200
        )
        self.sub2 = self.create_subscription(
            Bool,
            '/clear_three_dee_marker',
            self.clear_cb,
            1
        )

        self.pub = self.create_publisher(MarkerArray, '/collision_objects_marker', 5)

        self.counter = 0
        self.erase_marker = False
        self.markers_stored = MarkerArray()

        self.timer = self.create_timer(1.0, self.timer_cb)

    def clear_cb(self, msg: Bool):
        msg_marker = Marker()
        self.erase_marker = msg.data

        if self.erase_marker:
            msg_marker.action = Marker.DELETEALL
            array = MarkerArray()
            array.markers.append(msg_marker)
            self.pub.publish(array)

            self.get_logger().info(f"Object removed: {self.counter}")
            self.counter = 0
            self.erase_marker = False
            self.markers_stored.markers.clear()


    def timer_cb(self):
        self.pub.publish(self.markers_stored)

    def collision_cb(self, msg: CollisionObject):

        if self.erase_marker:
            return

        mark = Marker()
        mark.header = msg.header
        mark.ns = 'collision_objects_marker'

        
        if msg.operation == CollisionObject.REMOVE:
            mark.action = Marker.DELETE

        elif msg.operation == CollisionObject.ADD:
            mark.id = self.counter
            self.counter += 1

            mark.action = int(Marker.ADD)
            mark.type = int(Marker.CUBE)
            mark.pose = msg.pose

            # dimensioni
            if len(msg.primitives) > 0:
                mark.scale.x = msg.primitives[0].dimensions[0]
                mark.scale.y = msg.primitives[0].dimensions[1]
                mark.scale.z = msg.primitives[0].dimensions[2]

            # color
            mark.color.r = 0.2
            mark.color.g = 0.8
            mark.color.b = 0.2
            mark.color.a = 0.7    

        self.markers_stored.markers.append( mark )  



def main(args=None):
    rclpy.init(args=args)
    node = CollisionToMarker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()