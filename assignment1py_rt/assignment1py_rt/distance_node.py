#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        self.t1_pose = None
        self.t2_pose = None

        self.sub_t1 = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_callback, 10)

        self.sub_t2 = self.create_subscription(Pose, '/turtle2/pose', self.turtle2_callback, 10)

        self.distance_pub = self.create_publisher(Float32, '/turtles_distance', 10)

        self.get_logger().info("Distance node started")

    def turtle1_callback(self, msg):
        self.t1_pose = msg
        self.compute_and_publish_distance()

    def turtle2_callback(self, msg):
        self.t2_pose = msg
        self.compute_and_publish_distance()

    def compute_and_publish_distance(self):
        
        if self.t1_pose is None or self.t2_pose is None:
            return

        dx = self.t1_pose.x - self.t2_pose.x
        dy = self.t1_pose.y - self.t2_pose.y

        distance = math.sqrt(dx*dx + dy*dy)

        msg = Float32()
        msg.data = distance
        self.distance_pub.publish(msg)

        # Log
        #self.get_logger().info(f"Distance: {distance:.3f}")

    
def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()