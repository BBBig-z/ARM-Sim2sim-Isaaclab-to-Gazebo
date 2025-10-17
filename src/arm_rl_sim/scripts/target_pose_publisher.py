#!/usr/bin/env python3
"""
Target Pose Publisher for testing the arm RL sim
Publishes random reachable target poses for the robot arm from database
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random
import pickle
import os


class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')

        self.publisher = self.create_publisher(
            PoseStamped, '/target_pose', 10)

        # Load reachable poses database
        script_dir = os.path.dirname(os.path.abspath(__file__))
        db_path = os.path.join(script_dir, 'reachable_poses_database.pkl')

        try:
            with open(db_path, 'rb') as f:
                poses_data = pickle.load(f)
            
            # 解析Isaac Lab格式的数据库
            if isinstance(poses_data, dict) and 'positions' in poses_data:
                positions = poses_data['positions']  # Shape: (N, 3)
                orientations = poses_data['orientations_quat']  # Shape: (N, 4), [w, x, y, z]
                
                # 组合位置和姿态为pose列表
                self.poses_database = []
                for pos, quat in zip(positions, orientations):
                    # Isaac Lab格式: quat = [w, x, y, z]
                    # ROS格式: quat = [x, y, z, w]
                    pose = {
                        'position': pos.tolist() if hasattr(pos, 'tolist') else list(pos),
                        'orientation': [quat[1], quat[2], quat[3], quat[0]]  # 转换为ROS格式
                    }
                    self.poses_database.append(pose)
                
                self.get_logger().info(
                    f'从Isaac Lab数据库加载了 {len(self.poses_database)} 个可达位姿')
            elif isinstance(poses_data, (list, tuple)):
                self.poses_database = list(poses_data)
                self.get_logger().info(
                    f'Loaded {len(self.poses_database)} poses from database (list)')
            else:
                self.get_logger().error(
                    f'Unknown database format: {type(poses_data)}, keys: {poses_data.keys() if isinstance(poses_data, dict) else "N/A"}')
                self.poses_database = []
        except Exception as e:
            self.get_logger().error(f'Failed to load poses database: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.poses_database = []

        # Publish at 0.1 Hz (every 10 seconds) for robot to reach target
        self.timer = self.create_timer(10.0, self.publish_target_pose)

        self.get_logger().info('Target Pose Publisher started')

        # Publish initial target immediately
        self.publish_target_pose()

    def publish_target_pose(self):
        """Select and publish a random reachable target pose from database"""
        if not self.poses_database:
            self.get_logger().warn('No poses in database, skipping publish')
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Randomly select a pose from database
        pose_data = random.choice(self.poses_database)

        # Handle different possible data formats
        # pose_data could be dict, tuple, or list with position/orientation
        if isinstance(pose_data, dict):
            # If it's a dictionary with 'position' and 'orientation' keys
            if 'position' in pose_data:
                msg.pose.position.x = pose_data['position'][0]
                msg.pose.position.y = pose_data['position'][1]
                msg.pose.position.z = pose_data['position'][2]
            if 'orientation' in pose_data:
                msg.pose.orientation.x = pose_data['orientation'][0]
                msg.pose.orientation.y = pose_data['orientation'][1]
                msg.pose.orientation.z = pose_data['orientation'][2]
                msg.pose.orientation.w = pose_data['orientation'][3]
            else:
                # Default orientation
                msg.pose.orientation.w = 1.0
        elif isinstance(pose_data, (list, tuple)):
            # If list/tuple [x, y, z] or [x, y, z, qx, qy, qz, qw]
            msg.pose.position.x = pose_data[0]
            msg.pose.position.y = pose_data[1]
            msg.pose.position.z = pose_data[2]
            if len(pose_data) >= 7:
                msg.pose.orientation.x = pose_data[3]
                msg.pose.orientation.y = pose_data[4]
                msg.pose.orientation.z = pose_data[5]
                msg.pose.orientation.w = pose_data[6]
            else:
                msg.pose.orientation.w = 1.0

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
