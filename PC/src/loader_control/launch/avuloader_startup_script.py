import os
import sys
import serial # Zorg dat pyserial is geïnstalleerd (sudo apt install python3-serial)
import time

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# --- FUNCTIE: TEENSY RESETTEN ---
def reset_teensy_via_usb(port="/dev/ttyACM0"):
    print(f"[LAUNCH] Poging tot resetten van Teensy op {port}...")
    try:
        # DTR Toggle truc om Teensy te herstarten
        ser = serial.Serial(port, 57600)
        ser.dtr = False
        time.sleep(0.1)
        ser.dtr = True
        ser.close()
        print("[LAUNCH] Reset signaal verstuurd! Wachten op herstart...")
    except Exception as e:
        print(f"[LAUNCH WAARSCHUWING] Kon Teensy niet resetten: {e}")
        print("[LAUNCH] We gaan door, misschien was hij al klaar.")

def generate_launch_description():
    # 1. VOER DE RESET UIT (Voordat ROS start)
    # Dit gebeurt tijdens het "bouwen" van de launch beschrijving
    reset_teensy_via_usb()
    
    # 2. WACHT OP USB ENUMERATIE
    # Geef Linux even de tijd om de USB-poort weer te vinden na de reset
    time.sleep(3.0) 
    print("[LAUNCH] Wachttijd voorbij, systemen starten nu!")

    return LaunchDescription([
        # --- NODE 1: MICRO-ROS AGENT ---
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),
        
        # --- NODE 2: PS4 DRIVER ---
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # --- NODE 3: LOADER CONTROL ---
        Node(
            package='loader_control',
            executable='loader_node',
            name='loader_controller',
            output='screen'
        )
    ])