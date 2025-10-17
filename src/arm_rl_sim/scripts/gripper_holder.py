#!/usr/bin/env python3
"""
Gripper Holder - 保持gripper_1在零位置
"""

import rclpy
from rclpy.node import Node
from robot_msgs.msg import RobotCommand, MotorCommand
from rclpy.qos import qos_profile_system_default


class GripperHolder(Node):
    def __init__(self):
        super().__init__('gripper_holder')

        # 发布gripper_1命令（保持零位置）
        self.gripper_1_cmd_pub = self.create_publisher(
            RobotCommand,
            '/gripper_1_controller/command',
            qos_profile_system_default
        )

        # 定时发布命令 (100Hz)
        self.timer = self.create_timer(0.01, self.publish_gripper_1_command)

        self.get_logger().info(
            "Gripper holder started - holding gripper_1 at zero position")

    def publish_gripper_1_command(self):
        """发布gripper_1命令（保持零位置）"""
        cmd = RobotCommand()
        cmd.motor_command = [MotorCommand()]

        # 保持零位置
        cmd.motor_command[0].q = 0.0
        cmd.motor_command[0].dq = 0.0

        # 使用更高刚度PD控制 + 重力补偿
        cmd.motor_command[0].kp = 500.0  # 增加刚度
        cmd.motor_command[0].kd = 100.0  # 增加阻尼

        # 重力补偿：gripper_1质量0.043kg，沿z轴可能受重力
        # 当link6倾斜时需要补偿力矩
        cmd.motor_command[0].tau = 0.2  # 增加基础力矩

        self.gripper_1_cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GripperHolder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
