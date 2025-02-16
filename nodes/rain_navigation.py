#!/usr/bin/env python3
"""
rain_navigation_node.py

This node subscribes to the /sensor/rain topic and monitors the rain sensor values.
If a value below 900 is detected (indicating it's raining), it overrides the
light navigation and sends a navigation goal to Point A. Once triggered,
it stops checking further and logs that the override is active.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

class Config:
    # Define the pose for Point A (x, y, z-orientation, w-orientation)
    POINT_A = (6.252, -0.326, 0.815, 0.580)
    # Rain threshold: if sensor value is below 900, it is considered raining.
    RAIN_THRESHOLD = 800

class RainNavigationNode(Node):
    def __init__(self):
        super().__init__('rain_navigation')
        
        # Flag to ensure we only override once.
        self.override_triggered = False
        
        # Create a subscription to the rain sensor topic.
        self.create_subscription(Int32, 'sensor/rain', self.rain_callback, 10)
        
        # Create an action client for navigation.
        self.callback_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Rain Navigation Node initialized.')

    def rain_callback(self, msg: Int32):
        if self.override_triggered:
            # Already triggered; ignore further messages.
            return

        rain_value = msg.data
        self.get_logger().info(f"Received rain sensor value: {rain_value}")

        if rain_value < Config.RAIN_THRESHOLD:
            self.get_logger().info("Rain detected (value below 900)! Overriding control to navigate to Point A.")
            self.override_triggered = True  # Prevent further triggers.
            self.navigate_to_point_a()

    def create_pose_msg(self, point):
        """Helper function to create a PoseStamped message from a tuple."""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = point[0]
        pose_msg.pose.position.y = point[1]
        pose_msg.pose.orientation.z = point[2]
        pose_msg.pose.orientation.w = point[3]
        return pose_msg

    def navigate_to_point_a(self):
        """Send a navigation goal to Point A."""
        # Wait for the navigation action server to be available.
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigation action server...')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_msg(Config.POINT_A)
        
        self.get_logger().info('Sending navigation goal to Point A due to rain override...')
        future_goal = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future_goal)
        result_goal = future_goal.result()

        if not result_goal.accepted:
            self.get_logger().error('Navigation goal was rejected by the server.')
            return

        self.get_logger().info('Navigation goal accepted; waiting for result...')
        future_result = result_goal.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        result = future_result.result().result
        self.get_logger().info('Navigation override complete. Robot is at Point A.')
        self.get_logger().warn("Rain override active. To regain control, please restart both the rain and light navigation nodes.")

def main(args=None):
    rclpy.init(args=args)
    node = RainNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
