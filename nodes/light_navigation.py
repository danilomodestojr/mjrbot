#!/usr/bin/env python3

# MARKER: IMPORTS
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup

# MARKER: CONFIGURATION
class Config:
    # Light thresholds
    LIGHT_HIGH = 500
    LIGHT_LOW = 100
        
    # Confirmation counts (set to 2 for testing)
    CONFIRM_COUNT = 2
    
    # Navigation points (x, y, z-orientation, w-orientation)
    POINT_A = (6.252, -0.326, 0.815, 0.580)
    POINT_B = (4.369, 7.039, -0.621, 0.784)

# MARKER: MAIN_CLASS
class LightNavigationNode(Node):
    def __init__(self):
        super().__init__('light_navigation')
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize navigation client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Subscribe to light sensor topic
        self.light_sub = self.create_subscription(
            Int32,
            'sensor/light',
            self.light_callback,
            10
        )
        
        # Initialize state variables
        self.current_light = 0
        self.high_count = 0
        self.low_count = 0
        self.is_navigating = False
        self.at_point_a = True  # Start assuming we're at Point A
        
        self.get_logger().info('Light Navigation Node initialized')
        self.get_logger().info(f'High threshold: {Config.LIGHT_HIGH}, Low threshold: {Config.LIGHT_LOW}')
        self.get_logger().info(f'Confirmation count: {Config.CONFIRM_COUNT}')

    def light_callback(self, msg):
        """Handle incoming light sensor data with additional debugging"""
        if self.is_navigating:
            self.get_logger().debug("Currently navigating. Skipping sensor update.")
            return  # Skip sensor processing while navigating
            
        self.current_light = msg.data
        self.get_logger().info(f"Received light value: {self.current_light} | at_point_a: {self.at_point_a}")
        
        # Update counters based on thresholds
        if self.current_light > Config.LIGHT_HIGH:
            self.high_count += 1
            self.low_count = 0
            self.get_logger().debug(f"High count incremented: {self.high_count}")
        elif self.current_light < Config.LIGHT_LOW:
            self.low_count += 1
            self.high_count = 0
            self.get_logger().debug(f"Low count incremented: {self.low_count}")
        else:
            self.high_count = 0
            self.low_count = 0
            self.get_logger().debug("Light value between thresholds. Counters reset.")
        
        self.get_logger().debug(f"Current counters -> High: {self.high_count}, Low: {self.low_count}")
        
        # Check if conditions are met to trigger navigation
        if self.high_count >= Config.CONFIRM_COUNT and self.at_point_a:
            self.get_logger().info('Light consistently high, navigating to Point B')
            # Reset counters before navigation to avoid immediate re-triggering
            self.high_count = 0
            self.low_count = 0
            self.navigate_to_point_b()
            
        if self.low_count >= Config.CONFIRM_COUNT and not self.at_point_a:
            self.get_logger().info('Light consistently low, navigating to Point A')
            # Reset counters before navigation to avoid immediate re-triggering
            self.high_count = 0
            self.low_count = 0
            self.navigate_to_point_a()

    def create_pose_msg(self, point):
        """Create a PoseStamped message from a point tuple with debugging info"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = point[0]
        pose_msg.pose.position.y = point[1]
        pose_msg.pose.orientation.z = point[2]
        pose_msg.pose.orientation.w = point[3]
        self.get_logger().debug(f"Created PoseStamped: {pose_msg}")
        return pose_msg

    def navigate(self, point):
        """Send navigation goal synchronously and wait for result with additional debugging"""
        self.get_logger().info("Starting navigation...")
        # Wait for navigation action server to be available
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for navigation action server...")
        self.is_navigating = True
        
        # Create and send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_msg(point)
        self.get_logger().info("Sending navigation goal...")
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        if result.accepted:
            self.get_logger().info("Goal accepted, waiting for result...")
            result_future = result.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            final_result = result_future.result()
            self.get_logger().info(f"Navigation completed with result: {final_result.result}")
            self.is_navigating = False
            return final_result.result
        else:
            self.get_logger().error("Goal rejected by navigation server!")
            self.is_navigating = False
            return None

    def navigate_to_point_a(self):
        """Navigate to Point A with debugging"""
        self.get_logger().info('Navigating to Point A...')
        self.navigate(Config.POINT_A)
        self.at_point_a = True
        self.get_logger().info("Arrived at Point A.")

    def navigate_to_point_b(self):
        """Navigate to Point B with debugging"""
        self.get_logger().info('Navigating to Point B...')
        self.navigate(Config.POINT_B)
        self.at_point_a = False
        self.get_logger().info("Arrived at Point B.")

def main(args=None):
    rclpy.init(args=args)
    node = LightNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down node.')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
