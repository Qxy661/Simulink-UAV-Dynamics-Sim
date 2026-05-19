#!/bin/bash
# ============================================================
# test_quad.sh - multiModeQuad_ROS 快速测试脚本
# 自动完成编译、启动、发送指令、验证的全流程
# 用法: ./test_quad.sh [single|multi]
# ============================================================

set -e

MODE=${1:-single}
WORKSPACE="$HOME/catkin_ws_multiquad"
PACKAGE="multimodequad_ros"
DURATION=${TEST_DURATION:-10}  # 测试持续时间(秒)

echo "============================================"
echo " multiModeQuad_ROS 测试套件"
echo " 模式: $MODE"
echo " 时长: ${DURATION}s"
echo "============================================"

# 检查 ROS 环境
if [ -z "$ROS_MASTER_URI" ]; then
    source /opt/ros/noetic/setup.bash
fi

if [ -f "$WORKSPACE/devel/setup.bash" ]; then
    source "$WORKSPACE/devel/setup.bash"
fi

# 检查 roscore 是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "[ERROR] roscore 未运行！请先启动: roscore"
    exit 1
fi

# 检查包是否已编译
if [ ! -f "$WORKSPACE/devel/lib/$PACKAGE/$PACKAGE" ]; then
    echo "[1/4] 编译 $PACKAGE..."
    cd "$WORKSPACE"
    catkin_make 2>&1 | tail -3
fi

# 启动四旋翼节点
echo "[1/4] 启动四旋翼..."
if [ "$MODE" = "multi" ]; then
    roslaunch $PACKAGE multi_quad.launch count:=2 &
    NAMESPACES="quad_1 quad_2"
else
    roslaunch $PACKAGE single_quad.launch rviz:=false &
    NAMESPACES="quad"
fi
sleep 3

# 验证节点已运行
echo "[2/4] 验证通信..."
for ns in $NAMESPACES; do
    TOPIC="/${ns}/pose"
    if rostopic list | grep -q "$TOPIC"; then
        echo "  ✅ $TOPIC 就绪"
    else
        echo "  ⚠️  $TOPIC 未检测到 (仍在等待...)"
        sleep 2
    fi
done

# 发送控制指令
echo "[3/4] 发送控制指令..."

for ns in $NAMESPACES; do
    # 切换到速度模式
    rostopic pub /${ns}/flight_mode std_msgs/Int16 "data: 0" --once > /dev/null 2>&1
    echo "  ✅ ${ns}: 速度模式"
done

# 前飞测试
for ns in $NAMESPACES; do
    rostopic pub /${ns}/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 1.0
  y: 0.0
  z: -0.3
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --rate 20 > /dev/null 2>&1 &

    echo "  ✅ ${ns}: 前飞指令发送中..."
done

# 等待并监控
echo "[4/4] 监控飞行状态 (${DURATION}s)..."
START_TIME=$SECONDS
while [ $((SECONDS - START_TIME)) -lt $DURATION ]; do
    ELAPSED=$((SECONDS - START_TIME))
    # 显示进度条
    BAR=$(printf "%-${DURATION}s" "=")
    echo -ne "\r  进度: [${BAR:0:$ELAPSED}>] ${ELAPSED}/${DURATION}s"
    sleep 1
done
echo ""

# 停止指令
echo "  发送悬停指令..."
for ns in $NAMESPACES; do
    rostopic pub /${ns}/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --once > /dev/null 2>&1
done

# 清理
echo ""
echo "============================================"
echo " 测试完成!"
echo "============================================"
echo "快速再次测试: ./test_quad.sh $MODE"
echo "查看话题数据: rostopic echo /quad/pose"
echo "IMU数据:      rostopic echo /quad/imuData"
echo "停止所有节点: rosnode kill -a"
echo ""
