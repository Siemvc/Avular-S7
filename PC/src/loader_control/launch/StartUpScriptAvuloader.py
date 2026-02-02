from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Start de Micro-ROS Agent
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),
        
        # 2. Start de PS4 Joy Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # 3. Start jouw Control Node
        Node(
            package='loader_control',
            executable='loader_node',
            name='loader_controller',
            output='screen'
        ),
    ])