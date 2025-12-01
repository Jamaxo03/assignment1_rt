#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        # publisher
        self.pub_t1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.get_logger().info("UI node started")
        self.user_loop()   

    def user_loop(self):
        
        while rclpy.ok():
            print("\n select velocity:")

            try:
                lin = float(input("Linear velocity (x): "))
                ang = float(input("Angolar Velocity (z): "))
            except:
                print("error")
                continue

            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang

            print("Start")

            start = time.time()
            while time.time() - start < 1.0:
                self.pub_t1.publish(msg)
                time.sleep(0.1)

            #stop 
            stop_msg = Twist()
            self.pub_t1.publish(stop_msg)

            print("stop")

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()