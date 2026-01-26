#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class MotorController(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # Motor setpoint publishers
        self.motor1_pub = self.create_publisher(Float32, "/motor/1/setpoint", 10)
        self.motor2_pub = self.create_publisher(Float32, "/motor/2/setpoint", 10)
        self.motor3_pub = self.create_publisher(Float32, "/motor/3/setpoint", 10)
        self.motor4_pub = self.create_publisher(Float32, "/motor/4/setpoint", 10)
        
        self.motor_pubs = [self.motor1_pub, self.motor2_pub, self.motor3_pub, self.motor4_pub]
        
        # Joy subscriber
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
    def joy_callback(self, msg):
        # --- JOYSTICK AXES ---
        L_horizontal = msg.axes[0]  # Turning
        L_vertical = msg.axes[1]    # Forward/Backward
        
        # Speed scaling
        boost_btn = msg.buttons[4]      # L1
        precision_btn = msg.buttons[5]  # R1
        
        scale = 1.0
        if boost_btn: 
            scale = 2.0
        elif precision_btn: 
            scale = 0.5
        
        # Motor control logic
        # L_vertical: forward/backward
        # L_horizontal: turning
        forward_speed = L_vertical * scale * 4000.0  # Scale to 0-4000 RPM
        turn_speed = L_horizontal * scale * 4000.0
        
        # Motors 1,2 on right, 3,4 on left
        right_speed = forward_speed + turn_speed
        left_speed = forward_speed - turn_speed
        
        # Clamp speeds to valid range
        right_speed = max(-4000.0, min(4000.0, right_speed))
        left_speed = max(-4000.0, min(4000.0, left_speed))
        
        # Publish motor setpoints via ROS2
        self.motor1_pub.publish(Float32(data=right_speed))
        self.motor2_pub.publish(Float32(data=right_speed))
        self.motor3_pub.publish(Float32(data=left_speed))
        self.motor4_pub.publish(Float32(data=left_speed))

def main(args=None):
    rclpy.init(args=args)
    node = MotorController("motor_controller_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
