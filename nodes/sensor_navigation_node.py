# nodes/sensor_navigation_node.py
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from std_msgs.msg import String

class SensorNavigationNode(Node):
    def __init__(self):
        super().__init__('sensor_navigation')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.serial_sub = self.create_subscription(String, 'sensor_data', self.sensor_callback, 10)
        
        # Define waypoints (replace with your actual coordinates)
        self.waypoints = {
            'A': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
            'B': {'position': {'x': 1.0, 'y': 1.0, 'z': 0.0}}
        }

    def create_pose_stamped(self, position):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position['x']
        pose.pose.position.y = position['y']
        pose.pose.position.z = position['z']
        return pose

    def navigate_to(self, waypoint):
        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose_stamped(self.waypoints[waypoint]['position'])
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal)

    def sensor_callback(self, msg):
        try:
            command, value = msg.data.split()
            value = int(value)
            
            if command == 'l':  # LDR
                if value > 400:
                    self.navigate_to('B')
                else:
                    self.navigate_to('A')
            elif command == 'w' and value < 900:  # Rain
                self.navigate_to('A')
                
        except ValueError:
            self.get_logger().warn('Invalid sensor data format')

def main():
    rclpy.init()
    node = SensorNavigationNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()