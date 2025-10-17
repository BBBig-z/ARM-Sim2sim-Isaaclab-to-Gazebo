#!/usr/bin/env python3
"""
Gripper Mimic Controller
使gripper_2_joint镜像跟随gripper_1_joint
"""

import rclpy
from rclpy.node import Node
from robot_msgs.msg import RobotCommand, MotorCommand, RobotState
from rclpy.qos import qos_profile_system_default


class GripperMimicController(Node):
    def __init__(self):
        super().__init__('gripper_mimic_controller')

        # 订阅gripper_1的状态
        self.gripper_1_state_sub = self.create_subscription(
            RobotState,
            '/gripper_1_controller/state',
            self.gripper_1_state_callback,
            qos_profile_system_default
        )

        # 发布gripper_2的命令（镜像）
        self.gripper_2_cmd_pub = self.create_publisher(
            RobotCommand,
            '/gripper_2_controller/command',
            qos_profile_system_default
        )

        self.gripper_1_pos = 0.0
        self.gripper_1_vel = 0.0
        self.gripper_1_effort = 0.0

        # 定时发布gripper_2命令
        self.timer = self.create_timer(
            0.01, self.publish_gripper_2_command)  # 100Hz

        self.get_logger().info("Gripper mimic controller started")

    def gripper_1_state_callback(self, msg):
        """接收gripper_1状态"""
        if len(msg.motor_state) > 0:
            self.gripper_1_pos = msg.motor_state[0].q
            self.gripper_1_vel = msg.motor_state[0].dq
            self.gripper_1_effort = msg.motor_state[0].tau

    def publish_gripper_2_command(self):
        """发布gripper_2命令（镜像gripper_1）"""
        cmd = RobotCommand()
        cmd.motor_command = [MotorCommand()]

        # 镜像位置和速度（取反）
        cmd.motor_command[0].q = -self.gripper_1_pos
        cmd.motor_command[0].dq = -self.gripper_1_vel

        # 使用更高刚度PD控制保持位置（防止重力影响）
        cmd.motor_command[0].kp = 500.0  # 更高刚度
        cmd.motor_command[0].kd = 100.0  # 更高阻尼

        # 增加重力补偿力矩
        # gripper_2质量约0.043kg，需要足够的力矩抵抗重力
        gravity_compensation = 0.2  # 增加重力补偿（N·m）
        cmd.motor_command[0].tau = gravity_compensation

        self.gripper_2_cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GripperMimicController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
