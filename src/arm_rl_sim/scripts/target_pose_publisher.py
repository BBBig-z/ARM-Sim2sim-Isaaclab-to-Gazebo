#!/usr/bin/env python3
"""
Target Pose Publisher for testing the arm RL sim
Publishes random reachable target poses for the robot arm
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random
import math

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # Publish at 0.1 Hz (every 10 seconds) to give robot time to reach target
        self.timer = self.create_timer(10.0, self.publish_target_pose)
        
        self.get_logger().info('Target Pose Publisher started')
        
        # Publish initial target immediately
        self.publish_target_pose()
    
    def publish_target_pose(self):
        """Generate and publish a random reachable target pose"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Generate random position within reachable workspace
        # For a 6-DOF arm, typical workspace is a sphere around the base
        radius = random.uniform(0.2, 0.5)  # Distance from base
        theta = random.uniform(0, 2 * math.pi)  # Azimuth
        phi = random.uniform(math.pi/6, math.pi/2)  # Elevation (avoid too high/low)
        
        msg.pose.position.x = radius * math.sin(phi) * math.cos(theta)
        msg.pose.position.y = radius * math.sin(phi) * math.sin(theta)
        msg.pose.position.z = radius * math.cos(phi) + 0.2  # Offset from ground
        
        # Random orientation (simplified - identity or slight rotation)
        # For reach task, orientation might be less important
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published target pose: '
            f'x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}, '
            f'z={msg.pose.position.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TargetPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down target pose publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

