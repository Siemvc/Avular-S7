#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
import os


class Control(Node):
    def __init__(self, name):
        super().__init__(name)
        # Publishers
        self.actuator_pub = self.create_publisher(Twist, "/actuator_pub", 10)  
        self.lights_pub = self.create_publisher(Bool, "/lights_toggle", 10)
        self.standby_pub = self.create_publisher(Bool, "/standby", 10)     
        self.buttons_pub = self.create_publisher(Int32, "/actuator_buttons", 10)
        self.system_pub = self.create_publisher(Int32, '/system_command', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.listener_callback, 10)

        # State tracking voor edge-detection
        self.prev_d_pad_x = 0.0
        self.lights_on = False

        self.prev_standby = 0
        self.standby_active = True 
       
        self.prev_cross = 0
        self.prev_circle = 0
        self.prev_triangle = 0

        self.optionButton_prev = 0
        
        # Send initial states
        self.standby_pub.publish(Bool(data=self.standby_active))

    def listener_callback(self, msg):
    #JOYSTICK AXES AND BUTTONS MAPPING
        # Driving (Left Stick)
        L_horizontal = msg.axes[0]  #Turning
        L_vertical = msg.axes[1]    #Forward/Backward
        
        # Actuators (Right Stick)
        R_horizontal = msg.axes[3]  #Tilt
        R_vertical = msg.axes[4]    #Lift

        # Speed scaling driving
        Boost_btn = msg.buttons[5]      # L1
        precision_btn = msg.buttons[6]  # R1
        
        scale = 1.0
        if Boost_btn: scale = 2.0
        elif precision_btn: scale = 0.5

        # Making twist message
        t = Twist()
        t.linear.y = L_vertical * scale    # Forward/Backward
        t.angular.z = L_horizontal * scale # Turning
        t.angular.x = R_vertical           # Lift Manual 
        t.angular.y = R_horizontal         # Tilt Manual 

        self.actuator_pub.publish(t) 

        #BUTTONS FOR PRESETS
        cross = msg.buttons[0]
        circle = msg.buttons[1]
        square = msg.buttons[3]
        triangle = msg.buttons[2]

        # Logic: Send message only on button press (Rising Edge)
        if cross == 1 and self.prev_cross == 0:
            self.get_logger().info("Preset: LOW (Scrape)")
            self.buttons_pub.publish(Int32(data=0))
            
        if circle == 1 and self.prev_circle == 0:
            self.get_logger().info("Preset: DRIVE")
            self.buttons_pub.publish(Int32(data=1))
            
        if triangle == 1 and self.prev_triangle == 0:
            self.get_logger().info("Preset: DUMP")
            self.buttons_pub.publish(Int32(data=2))
        
        #OTHER BUTTONS
        if len(msg.axes) > 6:
            d_pad_down = msg.axes[6] # D-Pad Down/Up
        else:
            d_pad_down = 0.0 # Default waarde als de as niet bestaat
            
        if d_pad_down == -1 and self.prev_d_pad_x == 0.0:
            self.lights_on = not self.lights_on
            self.lights_pub.publish(Bool(data=self.lights_on))

        standbyButton = msg.buttons[10] # PS Button
        
        #Standby toggle logic
        if standbyButton == 1 and self.prev_standby == 0:
            self.standby_active = not self.standby_active
            self.standby_pub.publish(Bool(data=self.standby_active))
            self.get_logger().info(f"Standby: {self.standby_active}")

        #Shutdown logic
        shareButton = msg.buttons[9]  # Share button logic for shutdown
        optionButton = msg.buttons[8] 
        if shareButton == 1 and optionButton == 1:
            self.get_logger().warn("SHUTDOWN SEQUENCE INITIATED...")
        
            #Send shutdown signal to teensy
            sys_msg = Int32()
            sys_msg.data = 99
            self.system_pub.publish(sys_msg)
            #Wait a moment to ensure message is sent
            time.sleep(2.0)
            #Execute shutdown command
            os.system("shutdown now -h")

        #Update previous states
        self.prev_cross = cross
        self.prev_circle = circle
        self.prev_triangle = triangle
        self.prev_d_pad_x = d_pad_down
        self.prev_standby = standbyButton
        self.optionButton_prev = optionButton
        
def main(args=None):
    rclpy.init(args=args)
    node = Control("control_Node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()