# ARM Sim2Sim - Isaac Lab 到 Gazebo

六轴机械臂 Reach-IK 任务的 Sim2Sim 项目（Isaac Lab → Gazebo）

## 📁 项目结构

```
.
├── src/
│   ├── arm_description/          # 机械臂 URDF 模型 (Panther 6-DOF)
│   ├── robot_msgs/                # 自定义消息类型
│   ├── robot_joint_controller/    # 自定义 PD+力矩控制器
│   └── arm_rl_sim/                # RL Sim2Sim 主包
│       ├── models/arm_t_isaaclab/ # 训练好的策略模型
│       ├── config/                # 控制器配置
│       ├── scripts/               # Python 节点
│       └── launch/                # 启动文件
├── launch_sim.sh                  # 一键启动脚本
└── README.md
```

## 🚀 快速开始

### 1. 构建项目（在系统 Python 环境下）

```bash
# 确保退出 conda 环境
conda deactivate

# 构建
cd ~/works/ARM-Sim2sim-Isaaclab-to-Gazebo
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 2. 启动仿真（自动激活 conda 环境）

```bash
./launch_sim.sh
```

脚本会自动：
1. 清理之前的 Gazebo 进程
2. Source ROS 2 环境
3. 激活 `isaaclab_env` conda 环境（PyTorch）
4. 验证 PyTorch 可用
5. 启动 Gazebo 仿真

## 📦 关键组件

### 1. **机械臂模型** (`arm_description`)
- 6-DOF 机械臂 + 夹爪
- 使用 `ros2_control` 力矩控制接口
- Gazebo 物理仿真

### 2. **自定义控制器** (`robot_joint_controller`)
- `RobotJointController`: 单关节 PD + 前馈力矩控制
- `RobotJointControllerGroup`: 多关节组控制器
- 实时状态发布

### 3. **RL Sim2Sim 节点** (`arm_rl_sim.py`)
- 加载 Isaac Lab 训练的策略（PyTorch）
- 观测：关节位置/速度 + 目标末端位姿
- 动作：末端位姿增量（IK 模式）
- 控制频率：30Hz (RL) / 60Hz (控制)

### 4. **目标位姿发布器** (`target_pose_publisher.py`)
- 发布随机目标末端位姿
- 用于测试 Reach-IK 任务

## ⚙️ 配置文件

### 控制器配置 (`config/robot_control.yaml`)
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    robot_joint_controller_group:
      type: robot_joint_controller/RobotJointControllerGroup
```

### RL 参数 (`models/arm_t_isaaclab/config.yaml`)
```yaml
arm_t_isaaclab:
  model_name: "policy.pt"
  dt: 0.005
  decimation: 2
  num_observations: 19
  observations: ["joint_pos", "joint_vel", "pose_command", "actions"]
  rl_kp: [100, 100, 100, 100, 100, 100]
  rl_kd: [5, 5, 5, 5, 5, 5]
  action_scale: 0.3
```

## 🔍 验证系统运行

### 检查 ROS 2 节点
```bash
source install/setup.bash
ros2 node list
```

应该看到：
- `/arm_rl_sim_node`
- `/controller_manager`
- `/robot_joint_controller_group`
- `/gazebo_ros2_control`
- `/target_pose_publisher`

### 检查控制器状态
```bash
ros2 control list_controllers
```

应该看到：
- `joint_state_broadcaster` [active]
- `robot_joint_controller_group` [active]

### 查看关节状态
```bash
ros2 topic echo /joint_states
```

### 发布目标位姿
```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{
  pose: {
    position: {x: 0.3, y: 0.1, z: 0.4},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

## 🐛 常见问题

### 1. **CMake 找不到 catkin_pkg**
**原因**：在 conda 环境中构建  
**解决**：退出 conda 环境后再构建
```bash
conda deactivate
colcon build --symlink-install
```

### 2. **controller_manager 服务不可用**
**原因**：`gazebo_ros2_control` 插件未启动  
**解决**：检查 URDF 配置，确保 `panther_gazebo_control.xacro` 正确

### 3. **机器人塌下去**
**原因**：控制器未激活或资源冲突  
**解决**：确保只有 `robot_joint_controller_group` 激活，没有单独的关节控制器冲突

### 4. **PyTorch 找不到**
**原因**：未激活 conda 环境  
**解决**：使用 `./launch_sim.sh` 启动（自动处理）

## 📚 技术细节

### 观测空间 (19维)
- `joint_pos` (6): 关节位置
- `joint_vel` (6): 关节速度
- `pose_command` (7): 目标末端位姿 (pos_xyz + quat_xyzw)

### 动作空间 (6维)
- 末端位姿增量 (Isaac Lab IK 模式)
- 范围：[-1, 1]，缩放因子：0.3

### 控制流程
1. `arm_rl_sim.py` 接收目标位姿和关节状态
2. RL 策略推理输出动作（末端位姿增量）
3. 计算目标关节位置（PD 控制）
4. 发送力矩命令到 Gazebo
5. 关节执行力矩，更新状态

## 📝 开发说明

### 构建说明
- **构建时**：使用系统 Python（`/usr/bin/python3`）
- **运行时**：使用 conda 环境（`isaaclab_env`，包含 PyTorch）

### 文件结构
- **URDF 分离式结构**（参考可工作版本）：
  - `panther_with_ros2_control.urdf.xacro`: 主入口
  - `panther_gazebo.urdf.xacro`: 机器人描述 + world link
  - `panther_gazebo_control.xacro`: ros2_control 配置

## 📄 许可证

TODO: 添加许可证信息

## 👥 贡献者

- Isaac Lab 策略训练
- Gazebo Sim2Sim 实现

