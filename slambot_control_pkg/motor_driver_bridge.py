#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import serial

class MotorDriverBridge(Node):
    def __init__(self, motor_driver_port):
        super().__init__("motor_driver_bridge")

        self.ser = serial.Serial(motor_driver_port, 115200, timeout=10)

        self.geometry_subscriber = self.create_subscription(Twist, "cmd_vel", self.callback_command_motors, 10)

    def callback_command_motors(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        self.get_logger().info(f"linear.x: {linear_x}, angular.z: {angular_z}")

        command = f"V{linear_x:.2f},{angular_z:.2f}\n"
        self.ser.write(command.encode('utf-8'))

def main(args=None):
    parser = argparse.ArgumentParser(description='MotorDriverBridge')
    parser.add_argument('--motor_driver_port', type=str, default='/dev/ttyUSB0', help='Set motor driver (Arduino Mega) port')

    user_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MotorDriverBridge(motor_driver_port=user_args.motor_driver_port)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()