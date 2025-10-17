# ARM RL Sim - 六轴机械臂 Sim2Sim 项目

## 项目简介

本项目实现了从 Isaac Lab 到 Gazebo 的六轴机械臂 reach-ik 任务的 sim2sim 迁移。基于 `example/rl_sar` 的架构，针对机械臂任务进行了适配。

## 主要特点

- **任务类型**: 6-DOF 机械臂末端执行器位姿跟踪（reach-ik）
- **观察空间**: 关节位置、关节速度、目标位姿、上一步动作（共19维）
- **动作空间**: 6 维关节位置增量
- **控制方式**: PD 控制（位置+力矩混合）
- **实现语言**: Python（避免 Torch C++ API 兼容性问题）

## 项目结构

```
src/arm_rl_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── scripts/
│   ├── arm_rl_sim.py           # 主 RL Sim 节点（Python实现）
│   └── target_pose_publisher.py # 目标位姿发布器（测试用）
├── launch/
│   └── gazebo.launch.py         # Gazebo 仿真启动文件
├── models/
│   └── arm_t_isaaclab/
│       ├── config.yaml           # 模型配置参数
│       └── policy.pt             # 训练好的策略网络
└── config/
    └── robot_control.yaml        # ros2_control 控制器配置
```

## 依赖项

### ROS 2 包
- rclpy
- geometry_msgs
- robot_msgs (来自 example)
- robot_joint_controller (来自 example)
- robot_state_publisher
- gazebo_ros
- controller_manager

### Python 依赖
- torch
- numpy
- pyyaml

### 机器人模型
- arm_description (基于 panther_description，已适配 ros2_control)

## 编译与安装

1. **确保依赖包已编译**:
```bash
cd /home/y/works/ARM-Sim2sim-Isaaclab-to-Gazebo
colcon build --packages-select robot_msgs robot_joint_controller arm_description
```

2. **编译 arm_rl_sim**:
```bash
colcon build --packages-select arm_rl_sim
```

3. **source 环境**:
```bash
source install/setup.bash
```

## 使用方法

### 1. 启动 Gazebo 仿真

```bash
ros2 launch arm_rl_sim gazebo.launch.py
```

这将自动启动:
- Gazebo 仿真环境
- 机械臂模型
- ros2_control 控制器（每个关节一个控制器）
- joint_state_broadcaster
- robot_joint_controller_group
- ARM RL Sim 节点（8秒后）
- 目标位姿发布器（10秒后，用于测试）

### 2. 手动发布目标位姿

如果想手动指定目标位姿:

```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'base_link'
pose:
  position: {x: 0.3, y: 0.1, z: 0.3}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

### 3. 监控机器人状态

```bash
# 查看关节状态
ros2 topic echo /robot_joint_controller/state

# 查看目标位姿
ros2 topic echo /target_pose

# 查看发送的命令
ros2 topic echo /robot_joint_controller/command
```

## 配置说明

### config.yaml 参数

位于 `models/arm_t_isaaclab/config.yaml`:

- **model_name**: 策略模型文件名
- **framework**: "isaaclab"
- **dt**: 控制周期 (60Hz = 0.0167s)
- **decimation**: RL 策略更新的降采样率
- **num_observations**: 观察值维度 (19)
- **observations**: 观察值类型列表
- **action_scale**: 动作缩放因子 (0.3)
- **rl_kp/rl_kd**: PD 控制增益
- **torque_limits**: 关节力矩限制
- **default_dof_pos**: 默认关节位置

## 关键差异：四足 vs 机械臂

| 特性 | 四足机器人 (example/rl_sar) | 机械臂 (arm_rl_sim) |
|------|---------------------------|-------------------|
| DOF | 12 | 6 |
| IMU数据 | 需要（base_quat, ang_vel） | 不需要（固定base） |
| 命令输入 | 运动指令 (x, y, yaw) | 目标位姿 (xyz + quat) |
| 观察维度 | 45 | 19 |
| 基座 | 移动 | 固定 |

## 技术实现细节

### 观察值构建

```python
observations = [
    joint_pos (6),      # 关节位置
    joint_vel (6),      # 关节速度  
    pose_command (7),   # 目标位姿 (xyz + wxyz)
    actions (6)         # 上一步动作
]
# 总计: 6 + 6 + 7 = 19 维
```

### PD 控制

```python
target_pos = default_dof_pos + actions * action_scale
pos_error = target_pos - current_pos
vel_error = -current_vel

torques = kp * pos_error + kd * vel_error
```

### 线程架构

- **RL Loop**: 以 `dt * decimation` 周期运行模型推理
- **Control Loop**: 以 `dt` 周期发送控制命令
- 两个线程异步运行，确保高频控制

## 故障排除

### 1. 模型加载失败
- 检查 `models/arm_t_isaaclab/policy.pt` 是否存在
- 确认 PyTorch 版本兼容性

### 2. 控制器启动失败
- 检查 robot_joint_controller 是否正确安装
- 查看 ros2_control 配置是否正确

### 3. 机器人不动
- 检查是否收到目标位姿 (`/target_pose`)
- 检查是否收到机器人状态 (`/robot_joint_controller/state`)
- 查看日志输出

### 4. Gazebo 崩溃
- 增加 spawn 延迟时间
- 检查 URDF 文件是否正确

## 待优化项

1. **C++ 实现**: 解决 Torch C++ API 兼容性问题后可迁移到 C++ 版本
2. **观察值缓冲**: 添加历史观察值支持（如需要）
3. **域随机化**: 添加参数扰动以提高鲁棒性
4. **末端执行器反馈**: 添加实际末端位姿计算和反馈
5. **Gripper 控制**: 集成夹爪控制
6. **性能优化**: 优化推理速度和控制频率

## 参考

- Isaac Lab: https://github.com/isaac-sim/IsaacLab
- Example RL SAR: `example/rl_sar`
- ROS 2 Control: https://control.ros.org

## 许可证

MIT License

