# multiModeQuad_ROS 模型地图

> SLX 模型与生成代码的完整对应关系，帮助理解 Simulink 模块如何映射到 C++ 源码。

---

## 1. 模型层次总览

```
multiModeQuad_ROS.slx
├── Root
│   ├── Flight mode          →  slros_initialize.cpp: Sub_multiModeQuad_ROS_472
│   ├── Sub setpoint velocity → slros_initialize.cpp: Sub_multiModeQuad_ROS_426
│   ├── Sub setpoint attitude → slros_initialize.cpp: Sub_multiModeQuad_ROS_496
│   ├── Sub setpoint rate     → slros_initialize.cpp: Sub_multiModeQuad_ROS_497
│   ├── Sub thrust            → slros_initialize.cpp: Sub_multiModeQuad_ROS_500
│   ├── world / world1        → chart_104: "world" MATLAB Function
│   ├── time to sec & nsec    → chart_30/59: timetosecnsec()
│   │
│   ├── Attitude control (SubSystem)  ← system_526.xml (133KB, 80blocks)
│   │   ├── Velocity PID x    → multiModeQuad_ROS_data.cpp: PIDVelocityx_P/I/D/N
│   │   ├── Velocity PID y    → multiModeQuad_ROS_data.cpp: PIDVelocityy_P/I/D/N
│   │   ├── Velocity PID z    → multiModeQuad_ROS_data.cpp: PIDVelocityz_P/I/D/N
│   │   ├── Angular PID roll  → multiModeQuad_ROS_data.cpp: PIDangularroll_P/I/D/N
│   │   ├── Angular PID pitch → multiModeQuad_ROS_data.cpp: PIDangularpitch_P/I/D/N
│   │   ├── Angular PID yaw   → multiModeQuad_ROS_data.cpp: PIDangulayaw_P/I/D/N
│   │   ├── Mixer (chart_38)  → Omega = f(u1,u2,u3,u4)
│   │   ├── Force/Torque map  → chart_93: [T_B, tau_B] = f(T1,T2,T3,T4)
│   │   ├── Motor TF 1-4      → multiModeQuad_ROS.cpp: TransferFcn1-4 (24.39/(s+24.39))
│   │   ├── Attitude control law → chart_51: desired_rate = f(desired_quat, feedback_RotMatrix)
│   │   └── Thrust allocation → chart_85: att_sp_Body = f(att_sp_NED, DCM_be)
│   │
│   ├── 6DOF (Euler Angles)   → Aerospace Blockset 6DOF Euler Angles block
│   │   ├── Rigid Body Dynamics → multiModeQuad_ROS_derivatives()
│   │   └── Gravity Compensation → system_547: Mass, g, mg
│   │
│   ├── Publish pose          → slros_initialize.cpp: Pub_multiModeQuad_ROS_438
│   └── Publish imuData       → slros_initialize.cpp: Pub_multiModeQuad_ROS_477
│
└── Config (ert.tlc)
    ├── Solver: ODE4, dt=0.005
    └── Target: ROS Noetic + Embedded Coder
```

---

## 2. SLX 文件系统到 C++ 代码映射

| SLX 系统 | 大小 | 功能 | 对应的 C++ 代码 |
|----------|------|------|----------------|
| system_root.xml | 30KB | 顶层系统 (订阅/发布/可视化) | main.cpp, slros_initialize.cpp |
| system_526.xml | 133KB | 姿态控制子系统 (核心) | multiModeQuad_ROS.cpp 中 ~4500-6400 行 |
| system_534.xml | 2.4KB | 姿态控制 S-Function | multiModeQuad_ROS.cpp 中 MATLAB Function 部分 |
| system_547.xml | 3KB | 重力补偿 (Mass × g) | multiModeQuad_ROS.cpp: derivatives() |
| system_562.xml | 3.3KB | 力矩分配 (tau_B) | chart_93: T1-T4 → tau_B |
| system_563.xml | 4.3KB | 电机速度映射 | chart_38: u1-u4 → Omega |
| system_573.xml | 2.5KB | 姿态指令转换 | chart_85: NED→Body |
| system_54.xml | 4.8KB | 伺服动力学 (tilting) | chart_67 |
| system_37.xml | 37KB | 位置/速度/角速率 PID | multiModeQuad_ROS_data.cpp: PID 参数 |

---

## 3. Stateflow 图表

| Chart ID | 函数签名 | 功能 | 对应代码 |
|----------|---------|------|---------|
| 30 | `[sec,nsec]=fcn(time)` | 时间分割 | multiModeQuad_ROS_timetosecnsec() |
| 38 | `[Ω1..4]=fcn(u1..u4)` | **电机混控器** | mixer: 控制量→各电机 |
| 51 | `desired_rate=fcn(quat, R)` | **姿态控制器** | 期望四元数→期望角速率 |
| 59 | `[sec,nsec]=fcn(time)` | 时间分割 (IMU) | 同 chart_30 |
| 67 | `fcn(torqueL,R,angle,omega...)` | 伺服动力学 | tilt-rotor 机构 |
| 85 | `att_sp_Body=fcn(att_sp_NED, DCM)` | 坐标系转换 | NED→机体坐标系 |
| 93 | `[T_B,tau_B]=fcn(T1..T4)` | **力/力矩合成** | 各电机推力→总力+力矩 |
| 104 | `out=fcn()` | "world" 字符串 | frame_id = "map" |
| 110 | — | 辅助函数 | — |

---

## 4. 数据流

```
ROS话题输入
    │
    ▼
┌─────────────────────────────────────────────┐
│  flight_mode (Int16) → 模式选择             │
│  cmd_vel_unstamped (Twist) → 速度指令       │
│  attitude (PoseStamped) → 姿态指令          │
│  cmd_vel (TwistStamped) → 角速率指令        │
│  thrust (Float32) → 推力指令                │
└──────────────┬──────────────────────────────┘
               ▼
┌─────────────────────────────────────────────┐
│  模式切换 (Switch/Switch1/Switch2)          │
│  Mode 0: 速度控制 → 姿态解算               │
│  Mode 1: 角速率直通                         │
│  Mode 2: 姿态直通                           │
└──────────────┬──────────────────────────────┘
               ▼
┌─────────────────────────────────────────────┐
│  期望角速率 (desired_rate)                  │
│  chart_51: 四元数误差→角速率                │
└──────────────┬──────────────────────────────┘
               ▼
┌─────────────────────────────────────────────┐
│  角速率 PID (roll/pitch/yaw)               │
│  P=0.3/0.3/0.6, I=0.01, D=0.05            │
└──────────────┬──────────────────────────────┘
               ▼
┌─────────────────────────────────────────────┐
│  混控器 (chart_38)                          │
│  [u1,u2,u3,u4] → [Ω1,Ω2,Ω3,Ω4]           │
└──────────────┬──────────────────────────────┘
               ▼
┌─────────────────────────────────────────────┐
│  电机动力学 (TF 24.39/(s+24.39))           │
│  4 个一阶传递函数                           │
└──────────────┬──────────────────────────────┘
               ▼
┌─────────────────────────────────────────────┐
│  力/力矩合成 (chart_93)                     │
│  [T1..T4] → [T_B, tau_B]                   │
└──────────────┬──────────────────────────────┘
               ▼
┌─────────────────────────────────────────────┐
│  6DOF 动力学 (Aerospace Blockset)          │
│  [F,τ] → [ẍ, ÿ, z̈, φ̈, θ̈, ψ̈]            │
│  → [xe,ye,ze, u,v,w, φ,θ,ψ, p,q,r]        │
└──────────────┬──────────────────────────────┘
               ▼
┌─────────────────────────────────────────────┐
│  ROS 话题输出                                │
│  pose (PoseStamped)                         │
│  imuData (Imu)                              │
└─────────────────────────────────────────────┘
```
