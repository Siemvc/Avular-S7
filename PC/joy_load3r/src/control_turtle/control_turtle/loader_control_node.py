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
        self.motor1_pub = self.create_publisher(Float32, "/motor/m1/setpoint", 10)
        self.motor2_pub = self.create_publisher(Float32, "/motor/m2/setpoint", 10)
        self.motor3_pub = self.create_publisher(Float32, "/motor/m3/setpoint", 10)
        self.motor4_pub = self.create_publisher(Float32, "/motor/m4/setpoint", 10)

        
        self.motor_pubs = [self.motor1_pub, self.motor2_pub, self.motor3_pub, self.motor4_pub]
        
        # Joy subscriber
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
    def joy_callback(self, msg):
    # Protect against index out of range if controller is disconnected or different
        if len(msg.axes) < 2 or len(msg.buttons) < 6:
            self.get_logger().warn("Joystick message too short! Check controller connection.")
            return

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

        # Forward/turn logic
        forward_speed = L_vertical * scale * 4000.0
        turn_speed = L_horizontal * scale * 4000.0

        # Differential drive calculation
        right_speed = float(max(-4000.0, min(4000.0, forward_speed + turn_speed)))
        left_speed = float(max(-4000.0, min(4000.0, forward_speed - turn_speed)))

        # Publish
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
