#!/bin/bash
# ============================================================
# multiModeQuad_ROS 本地复现环境一键配置脚本
# 适用于 WSL2 (Ubuntu 20.04) / Ubuntu 20.04
# 用法: chmod +x setup_reproduction.sh && ./setup_reproduction.sh
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="/e/multiModeQuad_ROS"
WORKSPACE_DIR="$HOME/catkin_ws_multiquad"

echo "========================================"
echo " multiModeQuad_ROS 复现环境配置"
echo "========================================"
echo "项目路径: $PROJECT_DIR"
echo "工作空间: $WORKSPACE_DIR"
echo ""

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "[1/4] 检查 ROS 环境..."
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
        echo "  ✅ ROS Noetic 已安装"
    else
        echo "  ⚠️  ROS Noetic 未安装，请先安装:"
        echo "     sudo apt install ros-noetic-desktop-full"
        echo "  ⚠️  或参考: http://wiki.ros.org/noetic/Installation/Ubuntu"
        exit 1
    fi
else
    echo "[1/4] ROS 环境已就绪: $ROS_DISTRO"
fi

# 创建 catkin 工作空间
echo "[2/4] 创建 catkin 工作空间..."
mkdir -p "$WORKSPACE_DIR/src"
if [ ! -L "$WORKSPACE_DIR/src/multiModeQuad_ROS" ]; then
    ln -sf "$PROJECT_DIR" "$WORKSPACE_DIR/src/multiModeQuad_ROS"
    echo "  ✅ 软链接已创建: $WORKSPACE_DIR/src/multiModeQuad_ROS"
else
    echo "  ✅ 软链接已存在"
fi

# 编译
echo "[3/4] 编译 ROS 包..."
cd "$WORKSPACE_DIR"
catkin_make 2>&1 | tail -5
echo "  ✅ 编译完成"

# 设置环境
echo "[4/4] 配置环境..."
SHELL_RC="$HOME/.bashrc"
SETUP_CMD="source $WORKSPACE_DIR/devel/setup.bash"
if ! grep -q "catkin_ws_multiquad" "$SHELL_RC"; then
    echo "" >> "$SHELL_RC"
    echo "# multiModeQuad_ROS 工作空间" >> "$SHELL_RC"
    echo "alias cw_multiquad='cd $WORKSPACE_DIR'" >> "$SHELL_RC"
    echo "alias multiquad_launch='roslaunch multimodequad_ros multi_quad_demo.launch'" >> "$SHELL_RC"
    echo "  ✅ 别名已添加到 $SHELL_RC"
else
    echo "  ✅ 别名已存在"
fi

echo ""
echo "========================================"
echo " 配置完成！"
echo "========================================"
echo ""
echo "快速启动:"
echo "  1. 新终端 1: source $SETUP_CMD && roscore"
echo "  2. 新终端 2: source $SETUP_CMD && multiquad_launch"
echo "  3. 新终端 3: source $SETUP_CMD && rostopic echo /quad_1/pose"
echo "  4. 发送指令: source $SETUP_CMD"
echo "     rostopic pub /quad_1/flight_mode std_msgs/Int16 'data: 0' --once"
echo "     rostopic pub /quad_1/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist"
echo "              'linear: {x: 1.0, y: 0.0, z: 0.0}' --rate 20"
echo ""
echo "控制脚本: python3 $SCRIPT_DIR/multi_quad_demo.py"
echo "文档参考: $SCRIPT_DIR/setup_multimodequad_ros.md"
echo ""
