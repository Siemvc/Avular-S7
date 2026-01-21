#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

class Control(Node):
    def __init__(self, name):
        super().__init__(name)
        self.actuator_pub = self.create_publisher(Twist, "/actuator_pub", 10)
        self.drive_mode_pub = self.create_publisher(String, "/drive_mode", 10)
        self.lights_pub = self.create_publisher(Bool, "/lights_toggle", 10)
        self.standby_pub = self.create_publisher(Bool, "/standby", 10)     
        self.subscription = self.create_subscription(
            Joy, 
            '/joy', 
            self.listener_callback, 
            10
        )  
        

        self.prev_d_pad_x = 0.0
        self.lights_on = False
        self.prev_standby = 0.0
        self.standby = True
        self.prev_square = 0
        self.prev_triangle = 0
        self.current_mode = "Normal"
        self.standby_pub.publish(Bool(data = self.standby))
    def listener_callback(self, msg):
        

        L_horizontal = msg.axes[0]  #joy_left_x
        L_vertical = msg.axes[1]    #joy_left_y

        R_horizontal = msg.axes[3]  #joy_right_x
        R_vertical = msg.axes[4]    #joy_left_y

        Boost_btn = msg.buttons[4]  #L1
        precision_btn = msg.buttons[5] #R1

        d_pad_x = msg.axes[6] #right/left button
        
        square = msg.buttons[3] #square button
        triangle = msg.buttons[2] #triangle button

        standby_btn = msg.buttons[10] #ps-button
        scale = 1.0
        if Boost_btn:
            scale = 2.0
        elif precision_btn:
            scale = 0.5
        
        L_horizontal *= scale
        L_vertical *= scale
        R_horizontal *= scale
        R_vertical *= scale

        t = Twist()
        t.linear.y = L_vertical  #forward movement
        t.angular.z = L_horizontal #turning
        t.angular.x = R_vertical #arms
        t.angular.y = R_horizontal #bucket

        self.actuator_pub.publish(t)

        if d_pad_x == -1 and self.prev_d_pad_x == 0.0:
            self.lights_on = not self.lights_on
            self.lights_pub.publish(Bool(data = self.lights_on))

        if standby_btn == 1 and self.prev_standby == 0.0:
            self.standby = not self.standby
            self.standby_pub.publish(Bool(data = self.standby))

        if square == 1 and self.prev_square == 0:
            if self.current_mode == "drive_position":
                self.current_mode = "normal"
            else : 
                self.current_mode = "drive_position"
            self.drive_mode_pub.publish(String(data = self.current_mode))
        
        if triangle == 1 and self.prev_triangle == 0:
            if self.current_mode == "scrape_position":
                self.current_mode = "normal"
            else:
                self.current_mode = "scrape_position"
            
            self.drive_mode_pub.publish(String(data = self.current_mode))

        self.prev_standby = standby_btn
        self.prev_d_pad_x = d_pad_x
        self.prev_square = square
        self.prev_triangle = triangle
        
        self.get_logger().info("Sending msg")
def main(args=None):
    rclpy.init(args=args)
    node = Control("control_Node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
