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
        self.encoder_ticks_delta = 0

        self.maximum_motor_pwm = 255
        self.left_motor_pwm = self.maximum_motor_pwm
        self.right_motor_pwm = self.maximum_motor_pwm

        self.ser = serial.Serial(motor_driver_port, 115200, timeout=10)
        self.encoder_ticks_subscriber_ = self.create_subscription(EncoderTicks, "encoder_ticks", self.callback_calculate_encoder_ticks_delta, 10)
        self.geometry_subscriber = self.create_subscription(Twist, "cmd_vel", self.callback_command_motors, 10)

        self.calculate_correction_timer_ = self.create_timer(0.05, self.calculate_motor_pwm_correction)

    def callback_calculate_encoder_ticks_delta(self, msg: EncoderTicks):
        left_encoder_ticks = msg.left_encoder
        right_encoder_ticks = msg.right_encoder

        self.get_logger().info(f"left encoder ticks: {left_encoder_ticks}, right encoder ticks: {right_encoder_ticks}")

        if self.initial_left_encoder_ticks is None:
            self.initial_left_encoder_ticks = left_encoder_ticks

        if self.initial_right_encoder_ticks is None:
            self.initial_right_encoder_ticks = right_encoder_ticks


        left_encoder_ticks_delta = left_encoder_ticks - self.initial_left_encoder_ticks
        right_encoder_ticks_delta = right_encoder_ticks - self.initial_right_encoder_ticks

        self.encoder_ticks_delta = left_encoder_ticks_delta - right_encoder_ticks_delta

    def calculate_motor_pwm_correction(self):
        p = 3.1
        pwm_correction = abs(int(p * self.encoder_ticks_delta))

        if self.encoder_ticks_delta > 0: # Veering right 
            self.left_motor_pwm = max(0, min(255, self.maximum_motor_pwm - pwm_correction))
            self.right_motor_pwm = self.maximum_motor_pwm

        elif self.encoder_ticks_delta < 0: # Veering left
            self.left_motor_pwm = self.maximum_motor_pwm
            self.right_motor_pwm = max(0, min(255, self.maximum_motor_pwm - pwm_correction))

        else:
            self.left_motor_pwm = self.maximum_motor_pwm
            self.right_motor_pwm = self.maximum_motor_pwm

        # self.get_logger().info(f"encoder ticks delta: {self.encoder_ticks_delta}, p: {p}, pwm correction: {pwm_correction}, left motor PWM: {self.left_motor_pwm}, right motor PWM: {self.right_motor_pwm}")
    
    def callback_command_motors(self, msg: Twist):
        self.maximum_motor_pwm = int(msg.linear.x * 255)

        if msg.linear.x > 0:
            left_motor = self.left_motor_pwm
            right_motor = self.right_motor_pwm
        else:
            left_motor = 0
            right_motor = 0

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