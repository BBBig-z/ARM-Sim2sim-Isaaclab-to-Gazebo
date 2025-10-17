#!/usr/bin/env python3
"""
ARM RL Sim - Python implementation
Sim2Sim transfer from Isaac Lab to Gazebo for 6-DOF arm reach-ik task
"""

import os
import torch
import threading
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import PoseStamped, TransformStamped
from robot_msgs.msg import RobotCommand as RobotCommandMsg
from robot_msgs.msg import RobotState as RobotStateMsg
from robot_msgs.msg import MotorCommand as MotorCommandMsg
from robot_msgs.msg import MotorState as MotorStateMsg
from robot_msgs.msg import ArmDebugStatus
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformListener, Buffer
from scipy.spatial.transform import Rotation

class ARM_RL_Sim(Node):
    def __init__(self):
        super().__init__("arm_rl_sim_node")
        
        # Member variables
        self.target_pose = PoseStamped()
        self.robot_state_msg = RobotStateMsg()
        self.robot_command_msg = RobotCommandMsg()
        
        self.ros_namespace = self.get_namespace()
        
        # Get params from param_node
        self.param_client = self.create_client(GetParameters, '/param_node/get_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for param_node service to be available...")
        
        request = GetParameters.Request()
        request.names = ['robot_name', 'gazebo_model_name']
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if len(future.result().values) < 2:
            self.get_logger().warn("Failed to get all parameters from param_node")
            self.robot_name = "arm_t_isaaclab"
        else:
            self.robot_name = future.result().values[0].string_value
            self.gazebo_model_name = future.result().values[1].string_value
            self.get_logger().info(f"Get param robot_name: {self.robot_name}")
            self.get_logger().info(f"Get param gazebo_model_name: {self.gazebo_model_name}")
        
        # Read configuration from YAML
        self.load_config()
        
        # Initialize RL components
        torch.set_grad_enabled(False)
        self.robot_command_msg.motor_command = [MotorCommandMsg() for _ in range(self.num_of_dofs)]
        self.robot_state_msg.motor_state = [MotorStateMsg() for _ in range(self.num_of_dofs)]
        
        # Initialize observations
        self.obs_dof_pos = torch.zeros(1, self.num_of_dofs)
        self.obs_dof_vel = torch.zeros(1, self.num_of_dofs)
        self.obs_target_pose = torch.zeros(1, 7)  # xyz + quaternion (wxyz)
        self.obs_actions = torch.zeros(1, self.num_of_dofs)
        
        # Initialize outputs
        self.output_torques = torch.zeros(1, self.num_of_dofs)
        self.output_dof_pos = torch.zeros(1, self.num_of_dofs)
        
        # Load model
        model_path = os.path.join(
            get_package_share_directory('arm_rl_sim'),
            'models',
            self.robot_name,
            self.model_name
        )
        self.get_logger().info(f"Loading model from: {model_path}")
        self.model = torch.jit.load(model_path)
        self.model.eval()
        
        # Initialize target pose (default)
        self.target_pose.pose.position.x = 0.3
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.3
        self.target_pose.pose.orientation.w = 1.0
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = 0.0
        
        # TF listener for end-effector pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ee_frame = "link6"  # End-effector frame
        self.base_frame = "base_link"
        
        # Publisher (use group controller)
        self.robot_command_publisher = self.create_publisher(
            RobotCommandMsg,
            self.ros_namespace + "robot_joint_controller_group/command",
            qos_profile_system_default
        )
        
        # Debug status publisher
        self.debug_status_publisher = self.create_publisher(
            ArmDebugStatus,
            "/arm_debug_status",
            qos_profile_system_default
        )
        
        # Subscribers
        self.target_pose_subscriber = self.create_subscription(
            PoseStamped,
            "/target_pose",
            self.target_pose_callback,
            qos_profile_system_default
        )
        self.robot_state_subscriber = self.create_subscription(
            RobotStateMsg,
            self.ros_namespace + "robot_joint_controller_group/state",
            self.robot_state_callback,
            qos_profile_system_default
        )
        
        # Control flags
        self.simulation_running = True
        self.running_state = True
        
        # Control threads
        self.thread_control = threading.Thread(target=self.control_loop)
        self.thread_rl = threading.Thread(target=self.rl_loop)
        self.thread_control.daemon = True
        self.thread_rl.daemon = True
        self.thread_control.start()
        self.thread_rl.start()
        
        self.get_logger().info("ARM_RL_Sim started")
    
    def load_config(self):
        """Load configuration from YAML file"""
        config_path = os.path.join(
            get_package_share_directory('arm_rl_sim'),
            'models',
            self.robot_name,
            'config.yaml'
        )
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Extract parameters
        params = config[self.robot_name]
        self.model_name = params['model_name']
        self.framework = params['framework']
        self.dt = params['dt']
        self.decimation = params['decimation']
        self.num_observations = params['num_observations']
        self.observations = params['observations']
        self.clip_obs = params['clip_obs']
        self.clip_actions_lower = torch.tensor(params['clip_actions_lower'])
        self.clip_actions_upper = torch.tensor(params['clip_actions_upper'])
        self.num_of_dofs = params['num_of_dofs']
        self.action_scale = params['action_scale']
        self.dof_pos_scale = params['dof_pos_scale']
        self.dof_vel_scale = params['dof_vel_scale']
        self.torque_limits = torch.tensor(params['torque_limits'])
        self.default_dof_pos = torch.tensor(params['default_dof_pos'])
        self.rl_kp = torch.tensor(params['rl_kp'])
        self.rl_kd = torch.tensor(params['rl_kd'])
        
        self.get_logger().info(f"Loaded config for {self.robot_name}")
        self.get_logger().info(f"  DOFs: {self.num_of_dofs}")
        self.get_logger().info(f"  Observations: {self.num_observations}")
        self.get_logger().info(f"  Action scale: {self.action_scale}")
    
    def target_pose_callback(self, msg):
        """Callback for target pose updates"""
        self.target_pose = msg
    
    def robot_state_callback(self, msg):
        """Callback for robot state updates"""
        self.robot_state_msg = msg
    
    def get_observation(self):
        """Construct observation tensor from current state"""
        obs_list = []
        
        for obs_name in self.observations:
            if obs_name == "joint_pos":
                obs_list.append(self.obs_dof_pos * self.dof_pos_scale)
            elif obs_name == "joint_vel":
                obs_list.append(self.obs_dof_vel * self.dof_vel_scale)
            elif obs_name == "pose_command":
                obs_list.append(self.obs_target_pose)
            elif obs_name == "actions":
                obs_list.append(self.obs_actions)
        
        obs = torch.cat(obs_list, dim=1)
        obs = torch.clamp(obs, -self.clip_obs, self.clip_obs)
        return obs
    
    def forward(self):
        """Run model inference"""
        obs = self.get_observation()
        actions = self.model(obs)
        actions = torch.clamp(actions, self.clip_actions_lower, self.clip_actions_upper)
        return actions
    
    def compute_torques(self, actions):
        """Compute joint torques from actions using PD control"""
        # Target positions: default + scaled actions
        target_pos = self.default_dof_pos + actions[0] * self.action_scale
        
        # PD control
        pos_error = target_pos - self.obs_dof_pos[0]
        vel_error = -self.obs_dof_vel[0]  # Target velocity is 0
        
        torques = self.rl_kp * pos_error + self.rl_kd * vel_error
        return torques.unsqueeze(0)
    
    def compute_position(self, actions):
        """Compute target joint positions from actions"""
        target_pos = self.default_dof_pos + actions[0] * self.action_scale
        return target_pos.unsqueeze(0)
    
    def quaternion_to_euler(self, quat):
        """Convert quaternion (x,y,z,w) to Euler angles (roll, pitch, yaw) in radians"""
        r = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
        return r.as_euler('xyz', degrees=False)
    
    def get_ee_pose(self):
        """Get current end-effector pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            pos = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            quat = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            rpy = self.quaternion_to_euler(quat)
            
            return pos, rpy
        except Exception as e:
            return None, None
    
    def log_debug_status(self):
        """Log detailed debug information and publish to /arm_debug_status"""
        # Get target pose
        target_pos = [
            self.target_pose.pose.position.x,
            self.target_pose.pose.position.y,
            self.target_pose.pose.position.z
        ]
        target_quat = [
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w
        ]
        target_rpy = self.quaternion_to_euler(target_quat)
        
        # Get actual end-effector pose
        actual_pos, actual_rpy = self.get_ee_pose()
        
        # Joint torques
        torques = [float(self.output_torques[0, i]) for i in range(self.num_of_dofs)]
        
        # Joint positions
        joint_pos = [float(self.obs_dof_pos[0, i]) for i in range(self.num_of_dofs)]
        joint_vel = [float(self.obs_dof_vel[0, i]) for i in range(self.num_of_dofs)]
        target_joint_pos = [float(self.output_dof_pos[0, i]) for i in range(self.num_of_dofs)]
        
        # Publish debug status message
        debug_msg = ArmDebugStatus()
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = self.base_frame
        
        # Target and actual poses
        debug_msg.target_pose = self.target_pose.pose
        if actual_pos and actual_rpy is not None:
            debug_msg.actual_pose.position.x = actual_pos[0]
            debug_msg.actual_pose.position.y = actual_pos[1]
            debug_msg.actual_pose.position.z = actual_pos[2]
            # Convert RPY back to quaternion
            r = Rotation.from_euler('xyz', actual_rpy)
            quat = r.as_quat()
            debug_msg.actual_pose.orientation.x = quat[0]
            debug_msg.actual_pose.orientation.y = quat[1]
            debug_msg.actual_pose.orientation.z = quat[2]
            debug_msg.actual_pose.orientation.w = quat[3]
            
            # Position errors
            debug_msg.error_x = target_pos[0] - actual_pos[0]
            debug_msg.error_y = target_pos[1] - actual_pos[1]
            debug_msg.error_z = target_pos[2] - actual_pos[2]
            debug_msg.error_norm = np.sqrt(debug_msg.error_x**2 + debug_msg.error_y**2 + debug_msg.error_z**2)
            
            # Orientation errors (in degrees)
            debug_msg.error_roll = np.rad2deg(target_rpy[0] - actual_rpy[0])
            debug_msg.error_pitch = np.rad2deg(target_rpy[1] - actual_rpy[1])
            debug_msg.error_yaw = np.rad2deg(target_rpy[2] - actual_rpy[2])
        
        # Joint data
        debug_msg.joint_positions = joint_pos
        debug_msg.joint_velocities = joint_vel
        debug_msg.joint_torques = torques
        debug_msg.target_joint_positions = target_joint_pos
        debug_msg.joint_position_errors = [target_joint_pos[i] - joint_pos[i] for i in range(self.num_of_dofs)]
        
        self.debug_status_publisher.publish(debug_msg)
    
    def rl_loop(self):
        """Main RL loop - runs at decimated rate"""
        thread_period = self.dt * self.decimation
        self.get_logger().info(f"RL loop started with period {thread_period*1000:.1f}ms")
        
        while rclpy.ok():
            if self.running_state and self.simulation_running:
                # Update observations from robot state
                for i in range(self.num_of_dofs):
                    if i < len(self.robot_state_msg.motor_state):
                        self.obs_dof_pos[0, i] = self.robot_state_msg.motor_state[i].q
                        self.obs_dof_vel[0, i] = self.robot_state_msg.motor_state[i].dq
                
                # Update target pose observation
                self.obs_target_pose[0, 0] = self.target_pose.pose.position.x
                self.obs_target_pose[0, 1] = self.target_pose.pose.position.y
                self.obs_target_pose[0, 2] = self.target_pose.pose.position.z
                self.obs_target_pose[0, 3] = self.target_pose.pose.orientation.w
                self.obs_target_pose[0, 4] = self.target_pose.pose.orientation.x
                self.obs_target_pose[0, 5] = self.target_pose.pose.orientation.y
                self.obs_target_pose[0, 6] = self.target_pose.pose.orientation.z
                
                # Run model
                actions = self.forward()
                self.obs_actions = actions
                
                # Compute torques and positions
                self.output_torques = self.compute_torques(actions)
                self.output_torques = torch.clamp(
                    self.output_torques,
                    -self.torque_limits,
                    self.torque_limits
                )
                self.output_dof_pos = self.compute_position(actions)
            
            time.sleep(thread_period)
    
    def control_loop(self):
        """Control loop - sends commands every 15 seconds"""
        command_publish_period = 15.0  # Publish command every 15 seconds
        debug_period = 0.1  # Debug data publish every 0.1 seconds (10Hz)
        
        self.get_logger().info(f"Control loop started with command publish period {command_publish_period}s")
        self.get_logger().info(f"Loaded rl_kp: {self.rl_kp}")
        self.get_logger().info(f"Loaded rl_kd: {self.rl_kd}")
        
        last_command_time = time.time()
        last_debug_time = time.time()
        
        while rclpy.ok():
            current_time = time.time()
            
            if self.simulation_running:
                # Publish commands every 15 seconds
                if current_time - last_command_time >= command_publish_period:
                    # Prepare command message
                    for i in range(self.num_of_dofs):
                        if i < len(self.robot_command_msg.motor_command):
                            self.robot_command_msg.motor_command[i].q = float(self.output_dof_pos[0, i])
                            self.robot_command_msg.motor_command[i].dq = 0.0
                            self.robot_command_msg.motor_command[i].kp = 0.0
                            self.robot_command_msg.motor_command[i].kd = 0.0
                            self.robot_command_msg.motor_command[i].tau = float(self.output_torques[0, i])
                    
                    self.robot_command_publisher.publish(self.robot_command_msg)
                    last_command_time = current_time
                
                # Debug log every 15 seconds
                if current_time - last_debug_time >= debug_period:
                    self.log_debug_status()
                    last_debug_time = current_time
            
            time.sleep(0.1)  # Sleep 100ms to avoid busy waiting

def main(args=None):
    rclpy.init(args=args)
    arm_rl_sim = ARM_RL_Sim()
    
    try:
        rclpy.spin(arm_rl_sim)
    except KeyboardInterrupt:
        arm_rl_sim.get_logger().info("Shutdown signal received")
    finally:
        arm_rl_sim.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

