# multiModeQuad_ROS 复现指南

> 本指南提供从零搭建 multiModeQuad_ROS 仿真环境的完整步骤，含 ROS Noetic 安装、项目编译、单机/多机运行、控制指令发送。

---

## 1. 环境要求

### 1.1 操作系统

| 系统 | 支持 | 备注 |
|------|------|------|
| Ubuntu 20.04 LTS | ✅ 原生支持 | 推荐，ROS Noetic 官方支持 |
| Ubuntu 22.04 | ⚠️ 需自行编译 ROS Noetic | 不推荐 |
| Windows (WSL2) | ⚠️ 实验性支持 | 需 Ubuntu 20.04 WSL |
| macOS | ❌ | ROS 生态不支持 |

### 1.2 软件依赖

| 软件 | 版本 | 安装方式 |
|------|------|---------|
| ROS | Noetic Ninjemys | `apt install ros-noetic-desktop` |
| catkin | ROS 自带 | — |
| CMake | ≥ 3.9 | `apt install cmake` |
| GCC/G++ | ≥ 7.5 | `apt install build-essential` |

---

## 2. 安装 ROS Noetic

```bash
# 1. 设置 sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. 添加密钥
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 3. 更新并安装
sudo apt update
sudo apt install ros-noetic-desktop-full

# 4. 初始化 rosdep
sudo rosdep init
rosdep update

# 5. 设置环境变量
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 6. 安装依赖工具
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

---

## 3. 编译 multiModeQuad_ROS

```bash
# 1. 创建 catkin 工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 2. 克隆项目
cd ~/catkin_ws/src
git clone https://github.com/JJJJJJJack/multiModeQuad_ROS.git

# 3. 安装依赖
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 编译
catkin_make

# 5. 确认生成可执行文件
ls -la devel/lib/multimodequad_ros/multimodequad_ros
# 应输出：-rwxr-xr-x ... multimodequad_ros
```

> **注意**：如果遇到编译错误，检查 `CMakeLists.txt` 中的 `-std=c++0x` 是否与系统编译器兼容。Ubuntu 20.04 默认 GCC 9 无此问题。

---

## 4. 运行与测试

### 4.1 启动 ROS Master

```bash
# 新终端
roscore
```

### 4.2 启动四旋翼节点

```bash
# 新终端
cd ~/catkin_ws
source devel/setup.bash
roslaunch multimodequad_ros multi_quad_demo.launch
```

启动日志应显示：
```
[INFO] ** Starting the model "multiModeQuad_ROS" **
```

### 4.3 查看话题列表

```bash
# 新终端
source /opt/ros/noetic/setup.bash
rostopic list
```

预期输出（多机模式）：
```
/quad_1/flight_mode
/quad_1/setpoint_velocity/cmd_vel_unstamped
/quad_1/setpoint_attitude/attitude
/quad_1/setpoint_attitude/cmd_vel
/quad_1/thrust
/quad_1/pose
/quad_1/imuData
/quad_2/flight_mode
/quad_2/setpoint_velocity/cmd_vel_unstamped
...
/rosout
/rosout_agg
```

### 4.4 查看位姿输出

```bash
rostopic echo /quad_1/pose
```

输出示例：
```
header:
  seq: 42
  stamp:
    secs: 1234567890
    nsecs: 123456789
  frame_id: "map"
pose:
  position:
    x: 1.0        # NED坐标系下位置
    y: 1.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
---
```

---

## 5. 控制指令速查

### 5.1 速度控制模式（mode=0）

```bash
# 切换到速度模式
rostopic pub /quad_1/flight_mode std_msgs/Int16 "data: 0" --once

# 前飞 1 m/s
rostopic pub /quad_1/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --rate 20

# 向左侧飞
rostopic pub /quad_1/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --rate 20

# 上升
rostopic pub /quad_1/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 0.0
  z: -1.0    # NED坐标系下，上升为负
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --rate 20
```

### 5.2 姿态控制模式（mode=2）

```bash
# 切换到姿态模式
rostopic pub /quad_1/flight_mode std_msgs/Int16 "data: 2" --once

# 发送悬停姿态（单位四元数）
rostopic pub /quad_1/setpoint_attitude/attitude geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --rate 20
```

### 5.3 角速率控制模式（mode=1）

```bash
# 切换到角速率模式
rostopic pub /quad_1/flight_mode std_msgs/Int16 "data: 1" --once

# 发送偏航角速度
rostopic pub /quad_1/setpoint_attitude/cmd_vel geometry_msgs/TwistStamped "
header:
  frame_id: 'map'
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5" --rate 20
```

### 5.4 推力控制模式

```bash
# 直接发送油门指令（需配合其他模式）
rostopic pub /quad_1/thrust std_msgs/Float32 "data: 3.5" --rate 20
```

---

## 6. 多机编队测试

### 6.1 双机独立控制

```bash
# 终端 1: quad_1 前飞
rostopic pub /quad_1/flight_mode std_msgs/Int16 "data: 0" --once
rostopic pub /quad_1/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --rate 20 &

# 终端 2: quad_2 侧飞
rostopic pub /quad_2/flight_mode std_msgs/Int16 "data: 0" --once
rostopic pub /quad_2/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --rate 20
```

### 6.2 使用 Python 脚本控制

创建 `multi_quad_control.py`：

```python
#!/usr/bin/env python3
"""multiModeQuad_ROS 多机控制示例脚本"""

import rospy
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped


class MultiQuadController:
    """多四旋翼控制器"""

    def __init__(self, namespace="quad_1"):
        self.ns = namespace
        self.pub_flight_mode = rospy.Publisher(
            f"/{namespace}/flight_mode", Int16, queue_size=1)
        self.pub_velocity = rospy.Publisher(
            f"/{namespace}/setpoint_velocity/cmd_vel_unstamped",
            Twist, queue_size=1)
        self.pub_attitude = rospy.Publisher(
            f"/{namespace}/setpoint_attitude/attitude",
            PoseStamped, queue_size=1)
        self.pub_rate = rospy.Publisher(
            f"/{namespace}/setpoint_attitude/cmd_vel",
            TwistStamped, queue_size=1)
        self.pub_thrust = rospy.Publisher(
            f"/{namespace}/thrust", Float32, queue_size=1)

    def set_mode(self, mode: int):
        """设置飞行模式"""
        self.pub_flight_mode.publish(Int16(data=mode))

    def cmd_velocity(self, vx=0.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
        """发送速度指令（体轴系）"""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        twist.angular.x = wx
        twist.angular.y = wy
        twist.angular.z = wz
        self.pub_velocity.publish(twist)

    def cmd_thrust(self, thrust: float):
        """发送推力指令"""
        self.pub_thrust.publish(Float32(data=thrust))


if __name__ == "__main__":
    rospy.init_node("multi_quad_control_demo", anonymous=True)
    rate = rospy.Rate(20)  # 20 Hz

    # 控制 quad_1 前飞
    q1 = MultiQuadController("quad_1")
    q1.set_mode(0)
    rospy.loginfo("quad_1: velocity mode, moving forward")

    while not rospy.is_shutdown():
        q1.cmd_velocity(vx=1.0, vz=-0.5)  # 前飞+缓慢上升
        rate.sleep()
```

运行：
```bash
chmod +x multi_quad_control.py
python3 multi_quad_control.py
```

---

## 7. 修改模型并重新生成

如需修改动力学模型或控制器参数：

### 7.1 修改 PID 参数

无需重新生成代码！直接在 `multiModeQuad_ROS_data.cpp` 中修改参数文件中对应常量即可：

| 参数 | 文件内搜索关键字 | 默认值 |
|------|-----------------|--------|
| 速度环 P | `PIDVelocityx_P` | 10.0 |
| 速度环 I | `PIDVelocityx_I` | 0.02 |
| 速度环 D | `PIDVelocityx_D` | 0.01 |
| 角速率环 P | `PIDangularroll_P` | 0.3 |
| 质量 | `Mass` | 2.0 |
| 惯量 | `inertia` | [0.015, 0, 0; 0, 0.015, 0; 0, 0, 0.03] |

### 7.2 修改 SLX 模型并重新生成

```matlab
% 1. 打开模型
open_system('multiModeQuad_ROS.slx');

% 2. 修改模型（在 Simulink 编辑器中拖拽修改）

% 3. 生成 ROS 节点
slbuild('multiModeQuad_ROS');

% 4. 手动修改生成代码（添加多机支持）
%    在 slros_initialize.cpp 中：
%    SLROSNodePtr = new ros::NodeHandle("~");  % 添加 "~"
%
%    在 slros_busmsg_conversion.cpp 中：
%    msgPtr->header.frame_id = "map";  % 设置 frame_id

% 5. 重新编译 ROS 包
%    cd ~/catkin_ws && catkin_make
```

---

## 8. 常见问题

### Q1: `catkin_make` 报错 `std::c++0x` 不支持

```bash
# 编辑 CMakeLists.txt，将
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fpermissive")
# 改为
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")
```

### Q2: 无法启动，报 `[FATAL] [ROS requires python3]`

确保 Python3 环境和 ROS 包都正确安装：
```bash
sudo apt install python3-roslaunch python3-rostopic
```

### Q3: 话题中没有 `~` 前缀

如果使用默认 `NodeHandle()` 而不是 `NodeHandle("~")`，所有话题将发布在全局命名空间：
```
/multiModeQuad_ROS/flight_mode  （全局模式）
```
而不是：
```
/quad_1/flight_mode              （私有命名空间模式）
```

### Q4: 模型不受控制，一直悬停

检查飞行模式是否正确设置（`flight_mode` 话题），以及发布频率是否足够（建议 ≥ 20 Hz）。

### Q5: ROS Noetic 在 Ubuntu 22.04 上安装

Ubuntu 22.04 不支持 ROS Noetic 官方包。建议使用 Docker：
```bash
docker run -it --rm --net=host osrf/ros:noetic-desktop-full
```

---

## 9. 测试验证清单

| # | 测试项 | 预期结果 | 状态 |
|---|--------|---------|------|
| 1 | `catkin_make` 编译 | 成功，生成 `multimodequad_ros` 可执行文件 | |
| 2 | `roslaunch` 启动 | 显示 "Starting the model" 日志 | |
| 3 | `rostopic list` | 看到 `quad_1/` 和 `quad_2/` 命名空间话题 | |
| 4 | `rostopic echo /quad_1/pose` | 持续输出位姿数据，frame_id 为 "map" | |
| 5 | 发布 `flight_mode = 0` + 速度指令 | 四旋翼位置随时间变化 | |
| 6 | 切换 `flight_mode = 2` | 四旋翼跟踪姿态指令 | |
| 7 | 双机独立控制 | quad_1 和 quad_2 运动轨迹不同 | |

---

> **文档版本**：v1.0 | **对应项目**：[multiModeQuad_ROS](https://github.com/JJJJJJJack/multiModeQuad_ROS)
