# multiModeQuad_ROS 本地复现项目

## 位置

| 项目 | 路径 |
|------|------|
| 原始仓库 | `github.com/JJJJJJJack/multiModeQuad_ROS` |
| 本地克隆 | `E:\multiModeQuad_ROS` (WSL: `/e/multiModeQuad_ROS`) |
| 分析文档 | `docs/09-开源项目精读/06-multiModeQuad_ROS深度解析.md` |
| 复现指南 | `references/setup_multimodequad_ros.md` |
| 控制脚本 | `references/multi_quad_demo.py` |
| 配置脚本 | `references/setup_reproduction.sh` |

## 快速启动

```bash
# 1. 一键测试 (自动编译+启动+控制+验证)
cd /e/multiModeQuad_ROS/scripts && bash test_quad.sh single

# 或分步操作:
# 1. roscore (新终端)
# 2. roslaunch multimodequad_ros multi_quad.launch count:=2 (新终端)
# 3. rostopic pub /quad_1/setpoint_velocity/cmd_vel_unstamped ...
```

## 与本项目的关联

multiModeQuad_ROS 展示了 **Simulink → ROS 代码生成 → 多机部署** 的完整链路，与本项目的第 11 章（PX4飞控对接）紧密相关。其核心价值：

1. **模型驱动开发（MBD）范例**：SLX → C++ → ROS Package
2. **级联 PID 工程参数**：可直接参考的速度/角速率 PID 参数
3. **多机扩展架构**：NodeHandle("~") 命名空间隔离方案
4. **多控制模式切换**：速度/姿态/角速率/推力，通过 Stateflow 实现

## 文件结构

```
E:\multiModeQuad_ROS\
├── model/
│   └── multiModeQuad_ROS.slx  ← Simulink 源模型 (R2024a, ODE4@200Hz)
│
├── scripts/                     ← ★ 新增: MATLAB 脚本工具集
│   ├── quad_params.m           ← 参数初始化 (物理/控制/仿真)
│   ├── quad_trajectory.m       ← 5种参考轨迹 (hover/circle/figure8/step/helix)
│   ├── plot_quad_results.m     ← 6面板可视化 (3D/位置/姿态/控制/能量/电机)
│   ├── pid_tuning_guide.m      ← PID 整定指南 + ZN自动调参
│   ├── analyze_bag.m           ← ROS bag 数据分析器
│   └── test_quad.sh            ← 一键测试脚本 (编译→启动→控制→验证)
│
├── launch/                      ← ★ 改进: 增强启动文件
│   ├── multi_quad_demo.launch  ← 原双机启动
│   ├── single_quad.launch      ← 新: 单机启动 (可选 RVIZ)
│   ├── multi_quad.launch       ← 新: 多机启动 (可配置数量)
│   └── record_quad.launch      ← 新: 数据记录启动
│
├── config/                      ← ★ 新增: RVIZ 可视化配置
│   ├── quad_rviz.rviz          ← 单机 RVIZ
│   └── multi_quad_rviz.rviz    ← 多机 RVIZ
│
├── docs/                        ← ★ 新增: 技术文档
│   └── model_map.md            ← 完整模型地图 (SLX↔C++ 映射)
│
├── src/                         ← Simulink Coder 自动生成
├── include/                     ← 自动生成头文件
├── CMakeLists.txt               ← ROS catkin 编译
└── package.xml                  ← ROS 包描述
```
