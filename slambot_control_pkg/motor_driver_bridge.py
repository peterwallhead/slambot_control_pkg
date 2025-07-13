#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from slambot_interfaces.msg import EncoderTicks

import serial

class MotorDriverBridge(Node):
    def __init__(self, motor_driver_port):
        super().__init__("motor_driver_bridge")

        self.initial_left_encoder_ticks = None
        self.initial_right_encoder_ticks = None

        self.maximum_motor_pwm = 255
        self.left_motor_pwm = self.maximum_motor_pwm
        self.right_motor_pwm = self.maximum_motor_pwm

        self.ser = serial.Serial(motor_driver_port, 115200, timeout=10)
        self.encoder_ticks_subscriber_ = self.create_subscription(EncoderTicks, "encoder_ticks", self.callback_calculate_encoder_ticks_delta, 10)
        self.geometry_subscriber = self.create_subscription(Twist, "cmd_vel", self.callback_command_motors, 10)

    def callback_calculate_encoder_ticks_delta(self, msg: EncoderTicks):
        left_encoder_ticks = msg.left_encoder
        right_encoder_ticks = msg.right_encoder

        if self.initial_left_encoder_ticks is None:
            self.initial_left_encoder_ticks = left_encoder_ticks

        if self.initial_right_encoder_ticks is None:
            self.initial_right_encoder_ticks = right_encoder_ticks


        left_encoder_ticks_delta = left_encoder_ticks - self.initial_left_encoder_ticks
        right_encoder_ticks_delta = right_encoder_ticks - self.initial_right_encoder_ticks

        encoder_ticks_delta = left_encoder_ticks_delta - right_encoder_ticks_delta

        self.calculate_motor_pwm_correction(encoder_ticks_delta)

    def calculate_motor_pwm_correction(self, encoder_ticks_delta):
        # If the robot is veering, try to increase the motor speed on the side it's veering to
        # If the motor speed is already maxed out, reduce the motor speed on the opposing side
        if encoder_ticks_delta > 0: # Veering right 
            if self.right_motor_pwm < self.maximum_motor_pwm - 1:
                self.right_motor_pwm += 1
            else:
                if self.left_motor_pwm > 1:
                    self.left_motor_pwm -= 1
        elif encoder_ticks_delta < 0: # Veering left
            if self.left_motor_pwm < self.maximum_motor_pwm - 1:
                self.left_motor_pwm += 1
            else:
                if self.right_motor_pwm > 1:
                    self.right_motor_pwm -= 1
        else:
            self.left_motor_pwm = self.maximum_motor_pwm
            self.right_motor_pwm = self.maximum_motor_pwm

        self.get_logger().info(f"Left motor PWM: {self.left_motor_pwm}, right motor PWM: {self.right_motor_pwm}")
    
    def callback_command_motors(self, msg: Twist):
        self.maximum_motor_pwm = int(msg.linear.x * 255)

        if msg.linear.x > 0:
            left_motor = self.left_motor_pwm
            right_motor = self.right_motor_pwm
        else:
            left_motor = 0
            right_motor = 0

        # V255:1,255:1

        #self.get_logger().info(f"linear.x: {linear_x}, angular.z: {angular_z}")

        command = f"V{left_motor}:1,{right_motor}:1\n"
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