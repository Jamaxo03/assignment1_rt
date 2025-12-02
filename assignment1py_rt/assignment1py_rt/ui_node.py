#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        self.wait_for_two_turtles()

        # publishers
        self.pub_t1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_t2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.selected_pub = self.choose_turtle()

        self.user_loop()  

    def wait_for_two_turtles(self):
        
        self.get_logger().info("Checking turtle1 and turtle2")

        while True:
            topic_list = self.get_topic_names_and_types()
            topics = [t[0] for t in topic_list]

            t1 = "/turtle1/cmd_vel" in topics
            t2 = "/turtle2/cmd_vel" in topics

            if t1 and t2:
                self.get_logger().info("Both turtle1 and turtle2 detected!")
                break
            else:
                self.get_logger().info("Waiting for both turtles to be available...")
                time.sleep(2)

    def choose_turtle(self):
        while True:
            print("\nChoose turtle1 or turtle2 [1,2]: ")
            choice = input()

            if choice == "1":
                print("TURTLE 1!")
                return self.pub_t1

            elif choice == "2":
                print("TURTLE 2!")
                return self.pub_t2

            else:
                print("Invalid input")
 

    def user_loop(self):
        
        while rclpy.ok():
            print("\n Select velocity:")

            try:
                lin = float(input("Linear velocity (x): "))
                ang = float(input("Angolar Velocity (z): "))
            except:
                print("Invalid input")
                continue

            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang

            print("Start")
            self.selected_pub.publish(msg)

            time.sleep(1)

            stop_msg = Twist()
            print("Stop")
            self.selected_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()