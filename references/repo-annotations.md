# GitHub 仓库注解

> 本文档对 Simulink 无人机动力学仿真项目相关的 GitHub 仓库进行系统化注解，按主题分组，标注关键特性和学习价值。
> 难度等级：入门 / 进阶 / 高级 / 专家

---

## 一、多旋翼动力学

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 1 | [ethz-asl/ethzasl_msf](https://github.com/ethz-asl/ethzasl_msf) | 600+ | 多传感器融合框架 | EKF/UKF, ROS 集成, IMU+GPS+视觉融合 | 学习多传感器融合架构 | 进阶 |
| 2 | [hku-mars/MARSIM](https://github.com/hku-mars/MARSIM) | 500+ | 轻量级无人机仿真器 | LiDAR仿真, 点云渲染, 真实感强 | 理解传感器仿真原理 | 进阶 |
| 3 | [ethz-asl/euroc_dataset_tools](https://github.com/ethz-asl/euroc_dataset_tools) | 300+ | EuRoC 数据集工具 | MAV 数据集, 真值对比, 评估工具 | 评估状态估计算法 | 入门 |
| 4 | [uzh-rpg/rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control) | 1.2k+ | UZH 四旋翼控制 | SE(3)控制, 最小snap轨迹, ROS | 学习几何控制器 | 高级 |
| 5 | [ethz-asl/ethzasl_quadrotor_sim](https://github.com/ethz-asl/ethzasl_quadrotor_sim) | 200+ | ETH 四旋翼仿真 | 完整动力学模型, 积分方法对比 | 理解数值积分对仿真的影响 | 入门 |
| 6 | [JJJJJJJack/multiModeQuad_ROS](https://github.com/JJJJJJJack/multiModeQuad_ROS) | 1 | Simulink 生成的多模式四旋翼 ROS 节点 | 多控制模式切换, 多机扩展, Simulink→ROS 自动生成 | 学习 MBD 开发流程与多机架构 | 进阶 |

---

## 二、固定翼无人机

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 6 | [OpenVSP/OpenVSP](https://github.com/OpenVSP/OpenVSP) | 800+ | 飞行器参数化建模 | 气动外形设计, CFD 接口, NASA 开源 | 学习飞行器几何建模 | 进阶 |
| 7 | [Tomvonne/Flight-Dynamics-and-Control](https://github.com/Tomvonne/Flight-Dynamics-and-Control) | 100+ | 飞行动力学与控制 | 固定翼模型, 稳定性分析, MATLAB/Simulink | 理解固定翼动力学 | 进阶 |
| 8 | [SFSU-EE/flight_sim](https://github.com/SFSU-EE/flight_sim) | 50+ | 飞行仿真 MATLAB | 6-DOF 固定翼, 气动数据库, Simulink 模型 | 学习气动数据库集成 | 入门 |

---

## 三、综合无人机库

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 9 | [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | 7.5k+ | PX4 飞控固件 | 完整飞控栈, EKF2, MAVLink, SITL/HIL | 飞控系统架构参考 | 高级 |
| 10 | [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) | 10k+ | ArduPilot 飞控 | 多平台支持, 完善文档, 社区活跃 | 飞控工程实践参考 | 高级 |
| 11 | [ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator) | 700+ | ETH 旋翼仿真器 | Gazebo 插件, 电机模型, ROS 集成 | 理解仿真器架构 | 进阶 |
| 12 | [open-airlab/airsim_ros](https://github.com/open-airlab/airsim_ros) | 300+ | AirSim ROS 封装 | ROS 接口, 传感器数据, 控制指令 | 学习仿真器与 ROS 集成 | 进阶 |
| 13 | [microsoft/AirSim](https://github.com/microsoft/AirSim) | 16k+ | 微软 AirSim | 高保真渲染, 物理引擎, 多平台 | 学习高保真仿真 | 进阶 |
| 14 | [RobotWebTools/mavros](https://github.com/mavlink/mavros) | 900+ | MAVLink ROS 接口 | MAVLink 协议, 话题/服务, PX4/ArduPilot | 学习飞控通信协议 | 进阶 |

---

## 四、控制算法

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 15 | [uzh-rpg/rpg_mpc](https://github.com/uzh-rpg/rpg_mpc) | 400+ | 四旋翼 MPC 控制 | 非线性 MPC, acados 求解器, C++ 实现 | 学习 MPC 工程实现 | 高级 |
| 16 | [scipy/scipy](https://github.com/scipy/scipy) | 13k+ | SciPy 科学计算 | 最优化, 信号处理, 科学计算 | Python 科学计算基础 | 入门 |
| 17 | [OxfordControl/forcespro-python](https://github.com/OxfordControl/forcespro-python) | 100+ | FORCES Pro 求解器接口 | 嵌入式 MPC, 实时求解, Python 接口 | 学习嵌入式 MPC 求解 | 高级 |
| 18 | [casadi/casadi](https://github.com/casadi/casadi) | 1.5k+ | CasADi 优化框架 | 非线性优化, 自动微分, C++/Python | 学习数值优化方法 | 高级 |
| 19 | [chrismailer/mpt3](https://github.com/chrismailer/mpt3) | 50+ | MPT3 多参数工具箱 | 显式 MPC, 不变集计算, MATLAB | 学习显式 MPC 理论 | 专家 |

---

## 五、PX4 集成与仿真

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 20 | [PX4/PX4-SITL_gazebo](https://github.com/PX4/PX4-SITL_gazebo) | 300+ | PX4 Gazebo SITL | 软件在环仿真, 多机型, 环境模型 | 学习 SITL 仿真流程 | 进阶 |
| 21 | [PX4/PX4-Avoidance](https://github.com/PX4/PX4-Avoidance) | 800+ | PX4 避障模块 | 深度相机, 路径规划, 避障算法 | 学习感知与规划集成 | 高级 |
| 22 | [ethz-asl/pylon_camera](https://github.com/ethz-asl/pyylon_camera) | 100+ | Basler 相机驱动 | ROS 驱动, 实时采集, 工业相机 | 理解视觉传感器集成 | 入门 |
| 23 | [LorenzMeier/Firmware](https://github.com/PX4/PX4-Autopilot) | -- | PX4 开发分支 | 最新功能, 实验性特性 | 跟踪 PX4 最新进展 | 专家 |

---

## 六、强化学习飞行控制

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 24 | [utiasDSL/gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) | 1.5k+ | PyBullet 无人机 RL 环境 | Gym 接口, 多机仿真, 多种 RL 算法 | 学习 RL 环境搭建 | 进阶 |
| 25 | [FAL-Lab/PolyCAD](https://github.com/FAL-Lab/PolyCAD) | 100+ | 无人机路径规划 | 3D 路径规划, 障碍物避让, CAD 集成 | 学习路径规划算法 | 进阶 |
| 26 | [deeplearning-math/deeprl-quadrotor](https://github.com/deeplearning-math/deeprl-quadrotor) | 50+ | 深度 RL 四旋翼 | DDPG/PPO, 端到端控制, 仿真 | 学习 RL 应用到控制 | 进阶 |
| 27 | [openai/gym](https://github.com/openai/gym) | 34k+ | OpenAI Gym | 标准 RL 环境接口, 多种环境 | 理解 RL 环境设计标准 | 入门 |
| 28 | [DLR-RM/stable-baselines3](https://github.com/DLR-RM/stable-baselines3) | 9k+ | 稳定基线算法库 | PPO/SAC/TD3, PyTorch, 文档完善 | 学习 RL 算法实现 | 进阶 |

---

## 七、NASA 开源项目

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 29 | [nasa/T-MATS](https://github.com/nasa/T-MATS) | 300+ | NASA 涡轮建模工具 | 发动机热力学建模, Simulink 集成 | 学习推进系统建模 | 高级 |
| 30 | [nasa/astrobee](https://github.com/nasa/astrobee) | 800+ | NASA 自由飞行机器人 | 零重力仿真, ROS, 空间站操作 | 学习空间机器人控制 | 高级 |
| 31 | [nasa/fprime](https://github.com/nasa/fprime) | 10k+ | NASA 嵌入式框架 | 组件化架构, 实时系统, 飞行软件 | 学习飞行软件工程 | 专家 |

---

## 八、容错控制

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 32 | [ethz-asl/ethzasl_fw_px4](https://github.com/ethz-asl/ethzasl_fw_px4) | 100+ | 固定翼 PX4 集成 | 固定翼控制, PX4 接口, 容错策略 | 学习固定翼飞控集成 | 进阶 |
| 33 | [Tomvonne/FTC-Quadrotor](https://github.com/Tomvonne/FTC-Quadrotor) | 50+ | 容错控制四旋翼 | 故障检测, 控制重构, Simulink | 学习容错控制基础 | 进阶 |
| 34 | [mavlink/mavlink](https://github.com/mavlink/mavlink) | 2k+ | MAVLink 通信协议 | 消息定义, 多语言库, 版本管理 | 理解飞控通信协议 | 入门 |

---

## 九、传感器融合与状态估计

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 35 | [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr) | 2.5k+ | 相机-IMU 标定工具 | 联合标定, 多相机支持, 工业级精度 | 学习传感器标定方法 | 高级 |
| 36 | [hku-mars/VINS-Fusion](https://github.com/hku-mars/VINS-Fusion) | 3k+ | 视觉惯性导航系统 | VIO, GPS 融合, 回环检测 | 学习 VIO 状态估计 | 高级 |
| 37 | [rpng/open_vins](https://github.com/rpng/open_vins) | 1.5k+ | 开源 VINS | MSCKF, 视觉惯性, 模块化 | 学习 MSCKF 算法 | 专家 |
| 38 | [Cra2yPierr0t/MSCKF_VIO](https://github.com/Cra2yPierr0t/MSCKF_VIO) | 100+ | MSCKF VIO 实现 | MSCKF 算法, 代码注释详细 | 学习 MSCKF 代码实现 | 高级 |

---

## 十、轨迹规划与路径规划

| # | 仓库 | Stars | 简述 | 关键特性 | 学习价值 | 难度 |
|---|------|-------|------|----------|----------|------|
| 39 | [HKUST-Aerial-Robotics/Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) | 2.5k+ | 快速路径规划 | 动力学可行路径, 避障, 实时 | 学习无人机路径规划 | 高级 |
| 40 | [ethz-asl/mav_voxblox_planning](https://github.com/ethz-asl/mav_voxblox_planning) | 400+ | 基于体素的规划 | 3D 占用地图, 路径搜索, 增量更新 | 学习基于地图的规划 | 高级 |
| 41 | [OMPL/ompl](https://github.com/ompl/ompl) | 1k+ | 开源运动规划库 | RRT, PRM, 多种规划器 | 理解运动规划理论 | 进阶 |

---

## 快速索引

| 主题 | 仓库数 | 推荐入门仓库 |
|------|--------|-------------|
| 多旋翼动力学 | 5 | #4 rpg_quadrotor_control |
| 固定翼 | 3 | #7 Flight-Dynamics-and-Control |
| 综合库 | 6 | #11 rotors_simulator |
| 控制算法 | 5 | #18 casadi |
| PX4 集成 | 4 | #20 PX4-SITL_gazebo |
| RL 飞行控制 | 5 | #24 gym-pybullet-drones |
| NASA | 3 | #29 T-MATS |
| 容错控制 | 3 | #34 mavlink |
| 传感器融合 | 4 | #35 kalibr |
| 轨迹规划 | 3 | #39 Fast-Planner |
| **合计** | **41** | -- |
