#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
class Control(Node):
    def __init__(self, name):
        super().__init__(name)
        self.actuator_pub = self.create_publisher(Twist, "/actuator_pub", 10)  
        self.lights_pub = self.create_publisher(Bool, "/lights_toggle", 10)
        self.standby_pub = self.create_publisher(Bool, "/standby", 10)     
        self.buttons_pub = self.create_publisher(Int32, "/actuator_buttons", 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.listener_callback, 10)

        # State tracking voor edge-detection
        self.prev_d_pad_x = 0.0
        self.lights_on = False

        self.prev_standby = 0.0
        self.standby = True

        self.prev_square = 0
        self.prev_cross = 0
        self.prev_circle = 0
        self.prev_triangle = 0
        #Send initial states
        self.standby_pub.publish(Bool(data = self.standby))
        
    def listener_callback(self, msg):
        #Joystick axes and buttons
        #Driving
        L_horizontal = msg.axes[0]  #joy_left_x / Forawrd and backward
        L_vertical = msg.axes[1]    #joy_left_y / Left and right
        #Actuators
        R_horizontal = msg.axes[3]  #joy_right_x / Lift up and down
        R_vertical = msg.axes[4]    #joy_left_y / Bucket tilt
        #Speed scaling
        Boost_btn = msg.buttons[4]  #L1
        precision_btn = msg.buttons[5] #R1
        scale = 1.0
        if Boost_btn: scale = 2.0
        elif precision_btn: scale = 0.5

        #Making twist message
        t = Twist()
        t.linear.y = L_vertical * scale  #forward movement * scale
        t.angular.z = L_horizontal * scale #turning * scale
        t.angular.x = R_vertical #arms
        t.angular.y = R_horizontal #bucket

        self.actuator_pub.publish(t) #Publish actuator each callback

        #Buttons
        cross = msg.buttons[0]
        circle = msg.buttons[1]
        triangle = msg.buttons[2]
        #Logic: Send message only on button press (edge detection)
        if cross == 1 and self.prev_cross == 0:
            self.get_logger().info("Preset: LOW (Scrape)")
            self.buttons_pub.publish(Int32(data=0))
            
        if circle == 1 and self.prev_circle == 0:
            self.get_logger().info("Preset: DRIVE")
            self.buttons_pub.publish(Int32(data=1))
            
        if triangle == 1 and self.prev_triangle == 0:
            self.get_logger().info("Preset: DUMP")
            self.buttons_pub.publish(Int32(data=2))
        standby_btn = msg.buttons[10] #ps-button
        
        #Other buttons
        d_pad_x = msg.axes[6]
        if d_pad_x == -1 and self.prev_d_pad_x == 0.0:
            self.lights_on = not self.lights_on
            self.lights_pub.publish(Bool(data=self.lights_on))

        standby_btn = msg.buttons[10] # PS Button
        if standby_btn == 1 and self.prev_standby == 0:
            self.standby_state = not self.standby_state
            self.standby_pub.publish(Bool(data=self.standby_state))

        #Update previous states
        self.prev_cross = cross
        self.prev_circle = circle
        self.prev_triangle = triangle
        self.prev_d_pad_x = d_pad_x
        self.prev_standby = standby_btn

def main(args=None):
    rclpy.init(args=args)
    node = Control("control_Node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
