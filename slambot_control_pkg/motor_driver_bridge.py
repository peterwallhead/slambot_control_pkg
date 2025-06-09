#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class MotorDriverBridge(Node):
    def __init__(self):
        super().__init__("motor_driver_bridge")

        self.geometry_subscriber = self.create_subscription(Twist, "cmd_vel", self.callback_command_motors, 10)

    def callback_command_motors(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        self.get_logger().info(f"linear.x: {linear_x}, angular.z: {angular_z}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()