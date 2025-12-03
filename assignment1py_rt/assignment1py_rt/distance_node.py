#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        # sub for poses
        self.t1_pose = None
        self.t2_pose = None
        self.sub_t1 = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_callback, 10)
        self.sub_t2 = self.create_subscription(Pose, '/turtle2/pose', self.turtle2_callback, 10)

        # distance pub
        self.distance_pub = self.create_publisher(Float32, '/turtles_distance', 10)

        # stop pub
        self.stop_t1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.stop_t2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # threshold distance
        self.min_distance = 1

        self.get_logger().info("Distance node started")

    def turtle1_callback(self, msg):
        self.t1_pose = msg
        self.process_distances()

    def turtle2_callback(self, msg):
        self.t2_pose = msg
        self.process_distances()

    def process_distances(self):
        
        if self.t1_pose is None or self.t2_pose is None:
            return

        # which turtle is moving?
        moving_t1 = (self.t1_pose.linear_velocity != 0)
        moving_t2 = (self.t2_pose.linear_velocity != 0)


        dx = self.t1_pose.x - self.t2_pose.x
        dy = self.t1_pose.y - self.t2_pose.y

        distance = math.sqrt(dx*dx + dy*dy)

        msg = Float32()
        msg.data = distance
        self.distance_pub.publish(msg)

        # check distance between turtles
        if distance < self.min_distance:
            self.get_logger().warn(f"TOO CLOSE: {distance:.2f} → stopping turtles")

            stop = Twist()

            if moving_t1:
                self.stop_t1.publish(stop)
                self.get_logger().warn(f"Turtle1 stop")
            if moving_t2:
                self.stop_t2.publish(stop)
                self.get_logger().warn(f"Turtle2 stop")

        # check distance from walls
        if moving_t1 and self.is_near_wall(self.t1_pose):

            self.get_logger().warn("Turtle1 too close to a wall")
            self.stop_t1.publish(Twist())

        if moving_t2 and self.is_near_wall(self.t2_pose):

            self.get_logger().warn("Turtle2 too close to a wall")
            self.stop_t2.publish(Twist())

    def is_near_wall(self, pose):

        return (pose.x < 1 or pose.x > 10 or pose.y < 1 or pose.y > 10)
    

def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()