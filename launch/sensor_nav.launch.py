# launch/sensor_nav.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
       Node(
           package='mjrbot',
           executable='arduino_serial_node',
           name='arduino_serial',
           remappings=[],
           parameters=[],
           namespace='',
           condition=None,
           output='screen',
       ),
       Node(
           package='mjrbot',
           executable='sensor_navigation_node',
           name='sensor_navigation',
           remappings=[],
           parameters=[],
           namespace='',
           condition=None,
           output='screen',
       )
   ])