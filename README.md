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
│       │   ├── policy.pt          # PyTorch JIT 模型
│       │   ├── config.yaml        # Gazebo适配配置
│       │   └── params/            # Isaac Lab训练参数
│       │       ├── env.yaml       # 环境配置（观测/动作/奖励）
│       │       └── agent.yaml     # PPO算法配置
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
- 6-DOF 机械臂 + 固定夹爪（`fixed` 关节）
- 使用 `ros2_control` 力矩控制接口（仅6个主关节）
- 初始姿态：`[0.0, -0.5, 0.5, 0.0, 0.0, 0.0]` rad
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
- 从 `reachable_poses_database.pkl` 加载可达位姿
- 随机发布目标末端位姿（每1000秒更新）
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

### Gazebo适配配置 (`models/arm_t_isaaclab/config.yaml`)
```yaml
arm_t_isaaclab:
  model_name: "policy.pt"
  dt: 0.036666666666666666  # 60Hz / decimation
  decimation: 2             # RL推理频率 = 30Hz
  num_observations: 25      # 6+6+7+6
  observations: ["joint_pos", "joint_vel", "pose_command", "actions"]
  
  # Gazebo PD控制参数（低增益，适配差分IK）
  rl_kp: [1.8, 1.8, 1.8, 1.8, 0.8, 0.4]
  rl_kd: [0.3, 0.3, 0.3, 0.3, 0.3, 0.6]
  
  action_scale: 0.3         # 与Isaac Lab训练一致
  torque_limits: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
```

### Isaac Lab训练配置 (`models/arm_t_isaaclab/params/`)

**环境配置 (`env.yaml`)**:
```yaml
sim:
  dt: 0.016666666666666666  # 60Hz
  decimation: 2

robot:
  init_state:
    joint_pos:
      joint1: 0.0
      joint2: -0.5
      joint3: 0.5
      joint4-6: 0.0
  
  actuators:
    arm:
      # Isaac Lab HIGH_PD_CFG (ImplicitActuator)
      stiffness: [500, 450, 400, 350, 300, 250]
      damping: [150, 140, 130, 110, 90, 80]
      effort_limit_sim: 3.0

observations:
  policy:
    - joint_pos (6维): 相对关节位置，噪声 ±0.01
    - joint_vel (6维): 相对关节速度，噪声 ±0.01  
    - pose_command (7维): 目标末端位姿 (xyz + quat_wxyz)
    - actions (6维): 上一步动作（历史）

actions:
  arm_action:
    type: DifferentialInverseKinematicsAction
    scale: 0.3
    ik_method: dls  # Damped Least Squares
    ik_params:
      lambda_val: 0.01
```

**算法配置 (`agent.yaml`)**:
```yaml
policy:
  class_name: ActorCritic
  actor_hidden_dims: [256, 128]
  critic_hidden_dims: [256, 128]
  activation: relu

algorithm:
  class_name: PPO
  learning_rate: 0.001
  gamma: 0.9
  lam: 0.95
  clip_param: 0.12
```

## 📚 技术细节

### 观测空间 (25维)
| 项目 | 维度 | 说明 |
|------|------|------|
| `joint_pos` | 6 | 关节位置 (rad)，相对于默认位置 |
| `joint_vel` | 6 | 关节速度 (rad/s) |
| `pose_command` | 7 | 目标末端位姿 (xyz + quat_wxyz) |
| `actions` | 6 | 上一步动作（历史，防止震荡） |
| **总计** | **25** | |

### 动作空间 (6维)
| 项目 | 说明 |
|------|------|
| 类型 | 末端位姿增量 (Differential IK) |
| 格式 | `[dx, dy, dz, droll, dpitch, dyaw]` |
| 范围 | `[-1, 1]` (归一化) |
| 缩放 | `0.3` (实际移动范围) |
| IK方法 | 伪雅可比矩阵 (简化DLS) |

### 控制流程
```
1. target_pose_publisher.py
   ↓ 发布: /target_pose (每1000秒)
   
2. arm_rl_sim.py (RL线程 30Hz)
   ├─ 构造观测: [joint_pos, joint_vel, pose_cmd, last_actions]
   ├─ PyTorch推理: policy.pt (ActorCritic 256x128)
   ├─ 输出动作: [dx, dy, dz, droll, dpitch, dyaw]
   ├─ 差分IK: J_pinv @ pose_delta → joint_delta
   ├─ 目标位置: current_joint_pos + joint_delta
   └─ PD控制: tau = Kp*pos_error + Kd*vel_error
   
3. arm_rl_sim.py (控制线程 30Hz)
   ↓ 发布: /robot_joint_controller_group/command
   
4. robot_joint_controller_group (100Hz)
   ├─ 接收命令 (q_target, tau_ff)
   ├─ 应用力矩到Gazebo
   └─ 发布状态: /robot_joint_controller_group/state
   
5. Gazebo物理引擎 (实时)
   └─ 模拟机械臂运动
```

### 差分逆运动学
```python
# 伪雅可比矩阵 (6x6): 关节增量 = J_pinv @ 位姿增量
J_pinv = [
    [ 0.0,  5.0,  0.0,  0.0,  0.0,  3.0],  # joint1
    [ 0.0,  0.0,  3.5, -0.5,  0.0,  0.0],  # joint2
    [ 4.0,  0.0,  1.5,  1.0,  0.0,  0.0],  # joint3
    [ 0.0,  0.0,  0.0,  0.0,  5.0,  0.0],  # joint4
    [ 0.0,  0.0,  0.0,  5.0,  0.0,  0.0],  # joint5
    [ 0.0,  0.0,  0.0,  0.0,  0.0,  2.5],  # joint6
]
```
> 基于机械臂在典型姿态 `[0, -0.5, 1.57, 0, 0, 0]` 附近的雅可比估计

### Isaac Lab vs Gazebo 对比
| 项目 | Isaac Lab (训练) | Gazebo (部署) |
|------|------------------|---------------|
| 物理引擎 | PhysX (GPU加速) | ODE/Bullet |
| PD增益 | Kp=[500,450,400,350,300,250]<br>Kd=[150,140,130,110,90,80] | Kp=[1.8,1.8,1.8,1.8,0.8,0.4]<br>Kd=[0.3,0.3,0.3,0.3,0.3,0.6] |
| 控制器 | ImplicitActuator | 自定义PD+力矩 |
| IK方法 | DLS (Damped LS) | 伪雅可比 |
| 环境数 | 4096 (并行) | 1 (单个) |
| 频率 | 60Hz / 30Hz | 100Hz / 30Hz |
