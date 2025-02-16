#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseCapture(Node):
    def __init__(self):
        super().__init__('pose_capture')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('Pose capture node started. Click 2D Nav Goal in RViz2 to capture coordinates.')

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        self.get_logger().info(f'Captured pose: Position(x: {x:.3f}, y: {y:.3f}), Orientation(z: {z:.3f}, w: {w:.3f})')

def main():
    rclpy.init()
    node = PoseCapture()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()