#!/bin/bash

# ARM Sim2Sim Gazebo 启动脚本
# 注意：构建用系统Python，运行用conda环境（PyTorch）

echo "======================================"
echo "ARM Sim2Sim Gazebo Launch Script"
echo "======================================"

# 进入项目目录
cd "$(dirname "$0")"

# 1. 清理之前的 Gazebo 进程和缓存
echo "[1/5] 清理之前的 Gazebo 进程和缓存..."
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 ros2 2>/dev/null
sleep 2
# 清理 Gazebo 缓存（解决 Entity already exists 问题）
rm -rf ~/.gazebo/log/* 2>/dev/null
echo "  ✓ Gazebo 进程和缓存已清理"

# 2. Source ROS 2 环境（使用系统 Python）
echo "[2/5] Source ROS 2 环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. 检测并激活 conda 环境
echo "[3/5] 激活 isaaclab_env conda 环境..."

# 检测 conda 安装路径
CONDA_SCRIPT=""
for conda_path in ~/miniforge3 ~/miniconda3 ~/anaconda3; do
    if [ -f "$conda_path/etc/profile.d/conda.sh" ]; then
        CONDA_SCRIPT="$conda_path/etc/profile.d/conda.sh"
        break
    fi
done

if [ -z "$CONDA_SCRIPT" ]; then
    echo "错误: 找不到 conda 安装"
    echo "请确保已安装 conda (miniforge3/miniconda3/anaconda3)"
    exit 1
fi

# 激活 conda
source "$CONDA_SCRIPT"
conda activate isaaclab_env

if [ $? -ne 0 ]; then
    echo "错误: 无法激活 isaaclab_env 环境"
    echo "请先创建环境: conda create -n isaaclab_env python=3.10"
    exit 1
fi

# 4. 验证 PyTorch
echo "[4/5] 验证 PyTorch..."
python3 -c "import torch; print(f'✓ PyTorch {torch.__version__} 可用')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "错误: PyTorch 未安装或不可用"
    echo "请在 isaaclab_env 环境中安装: conda install pytorch -c pytorch"
    exit 1
fi

# 5. 启动 Gazebo 仿真
echo "[5/5] 启动 Gazebo 仿真..."
echo "======================================"
echo ""
ros2 launch arm_rl_sim gazebo.launch.py

# 清理
echo ""
echo "======================================"
echo "仿真已停止"
echo "======================================"

