# Simscape 多体动力学

> 预计阅读：25 分钟 | 前置知识：Simulink 基础操作、刚体动力学基本概念、6DOF 建模经验

Simscape Multibody（原 SimMechanics）提供了一种基于物理的多体动力学建模方式，无需手动推导运动方程。本文介绍如何使用 Simscape 搭建四旋翼无人机的多体动力学模型。

---

## 1. Simscape 体系概述

### 1.1 Simscape 产品家族

| 产品 | 功能 | UAV 建模用途 |
|------|------|-------------|
| **Simscape** | 基础物理建模平台 | 多物理域建模基础 |
| **Simscape Multibody** | 多体动力学 | 机架、关节、刚体运动 |
| **Simscape Electrical** | 电气系统 | 电机、电池、ESC |
| **Simscape Fluids** | 流体系统 | 液压、燃油系统 |
| **Simscape Driveline** | 传动系统 | 齿轮箱、传动轴 |

### 1.2 Simscape vs 传统 Simulink

| 特性 | 传统 Simulink | Simscape |
|------|--------------|----------|
| 建模方式 | 信号流 (Signal-based) | 物理连接 (Physical) |
| 方程推导 | 手动推导 | 自动生成 |
| 连接方式 | 信号线 (有方向) | 物理端口 (无方向) |
| 因果关系 | 显式 (输入→输出) | 隐式 (求解器自动处理) |
| 学习曲线 | 需要推导数学模型 | 只需理解物理连接 |
| 灵活性 | 完全自定义 | 受限于库中组件 |
| 求解器 | 信号流求解器 | 需要 Simscape 求解器 |
| 适用场景 | 控制算法、自定义模型 | 机械结构、物理原型 |

---

## 2. Simscape Multibody 核心模块

### 2.1 关键模块列表

| 模块名 | 功能 | 典型用途 |
|--------|------|----------|
| `Rigid Transform` | 刚体变换（平移+旋转） | 定义部件之间的相对位置 |
| `Solid` | 定义刚体（质量、惯量、几何） | 机架、机臂 |
| `Revolute Joint` | 旋转关节（1自由度） | 旋翼旋转轴 |
| `Prismatic Joint` | 棱柱关节（平移自由度） | 起落架缓冲 |
| `Welded Joint` | 焊接关节（0自由度） | 固定连接 |
| `6-DOF Joint` | 六自由度关节 | 飞行器自由运动 |
| `External Force and Torque` | 外力/力矩输入 | 电机推力、重力 |
| `Joint Initial Condition` | 关节初始状态 | 初始位置、速度 |
| `Joint Sensor` | 关节传感器 | 测量位置、速度、力 |
| `Mechanics Explorer` | 3D 可视化 | 实时查看运动 |

### 2.2 物理端口 vs 信号端口

```
传统 Simulink:            Simscape:
  信号线 (单向)              物理端口 (双向)
  ┌─────┐ ──signal──> ┌─────┐
  │ Out │              │ In  │    信号只单向流动
  └─────┘              └─────┘

  ┌─────┐ ═══port════ ┌─────┐
  │     │              │     │    物理连接双向传递
  └─────┘              └─────┘    (力、速度等同时传递)
```

---

## 3. 构建四旋翼机架

### 3.1 整体结构

```
                    M3(CCW)           M4(CW)
                      │                 │
                      │    机臂 L       │
                ──────┤                 ├──────
                      │    ┌─────┐      │
                      └────┤机身  ├──────┘
                           │Center│
                      ┌────┤     ├──────┐
                ──────┤    └─────┘      ├──────
                      │                 │
                    M1(CCW)           M2(CW)
```

### 3.2 搭建步骤

**Step 1: 创建机架 (Body)**

```
模块: Solid
参数:
  Geometry:   > Cylinder (圆柱体)
    Radius:   0.05 m
    Length:   0.15 m
  Mass:       0.8 kg (不含电机和螺旋桨)
  Inertia:    根据几何自动计算
  Frame:      World (世界坐标系)
```

**Step 2: 创建机臂 (Arm)**

```
模块: Solid
参数:
  Geometry:   > Brick (长方体)
    Dimensions: [0.225, 0.015, 0.015] m  (长x宽x高)
  Mass:       0.05 kg
```

**Step 3: 使用 Rigid Transform 定位**

```
Rigid Transform 参数:
  Translation:
    Method:   > Cartesian
    Offset:   [0.225, 0, 0] m  (沿 X 轴平移机臂长度)

  Rotation:
    Method:   > Standard Axis
    Axis:     Z
    Angle:    0° (机臂1), 90° (机臂2), 180° (机臂3), 270° (机臂4)
```

**Step 4: 使用 Welded Joint 固定**

```
模块: Welded Joint
功能: 将机臂固定到机身上，0 自由度
```

### 3.3 完整机架组装

```
┌─────────────────────────────────────────────────────────┐
│  Simscape 四旋翼机架                                      │
│                                                         │
│  ┌────────┐  ┌───────┐  ┌────────┐  ┌───────┐          │
│  │Rigid   │  │Welded │  │ Solid  │  │Rigid  │          │
│  │Transform├─>│Joint  ├──>│ (Arm1) ├──>│Transform│──> 端口1│
│  └────────┘  └───────┘  └────────┘  └───────┘          │
│                                                         │
│  (同样结构 x4, 旋转角度分别为 0°, 90°, 180°, 270°)        │
│                                                         │
│  ┌──────────┐                                           │
│  │ 6-DOF    │ <── 飞行器自由运动                           │
│  │ Joint    │                                           │
│  └──────────┘                                           │
└─────────────────────────────────────────────────────────┘
```

---

## 4. 添加电机推力

### 4.1 推力施加方式

在 Simscape Multibody 中施加推力有三种方式：

| 方式 | 适用场景 | 实现难度 |
|------|----------|----------|
| `External Force and Torque` | 从信号端口注入力/力矩 | 简单 |
| `Actuator` 模块 (如 `Torque Source`) | 驱动关节运动 | 中等 |
| 自定义 `Force Law` | 复杂力模型 | 较难 |

### 4.2 使用 External Force and Torque

```
信号流: Simulink 信号 ──> Simscape 物理力

连接方式:
┌──────────────┐     ┌────────────────────────────┐
│ Signal-based │     │    Simscape Multibody       │
│              │     │                            │
│ Motor Model  │──> [Transform Signal to Physical]──> [External Force]
│ (PWM→推力)   │     │  (Simulink-PS Converter)    │
└──────────────┘     └────────────────────────────┘
```

### 4.3 模块连接

```
Motor Model (Simulink):
  输入: PWM (4x1)
  输出: 推力 thrust (4x1), 力矩 torque (4x1)

转换:
  Simulink-PS Converter ──> 将 Simulink 信号转换为 Simscape 物理信号

施加:
  External Force and Torque 模块
    F: [0; 0; thrust_i]  (沿机体 Z 轴)
    M: [0; 0; torque_i]  (反扭矩，沿 Z 轴)
```

---

## 5. CAD 导入工作流

### 5.1 支持的 CAD 格式

| 格式 | 说明 | 导入方式 |
|------|------|----------|
| STEP (.step/.stp) | 通用 CAD 格式 | `File > Import > CAD` |
| STL (.stl) | 3D 打印格式 | `Solid > Geometry > File` |
| XML (.xml) | URDF/SDF 格式 | `smimport()` 函数 |
| PLY (.ply) | 点云格式 | 仅用于可视化 |

### 5.2 STEP 文件导入流程

```
1. 准备 STEP 文件（简化不必要的细节：螺丝、圆角等）
2. 在 Simscape 中: File > Import Geometry
3. 选择 STEP 文件
4. 设置:
   - 单位: mm (取决于 CAD 软件)
   - 简化: Moderate (推荐)
   - 可视化: 勾选
5. 自动生成 Solid 模块，带有导入的几何
6. 设置质量和惯量（可从 CAD 软件获取）
```

### 5.3 URDF 导入

```matlab
% MATLAB 命令行
>> smimport('quadrotor.urdf')
% 自动生成 Simscape Multibody 模型
% 包含关节、刚体、坐标系定义
```

### 5.4 CAD 导入注意事项

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 模型过大，仿真极慢 | 几何过于复杂 | 在 CAD 中简化，去除小特征 |
| 质量/惯量不对 | 几何不完整 | 手动设置物理属性 |
| 坐标系不对 | CAD 坐标系约定不同 | 使用 Rigid Transform 调整 |
| 渲染异常 | 法线方向错误 | 在 CAD 中修复或使用 STL |

---

## 6. 可视化: Mechanics Explorer

### 6.1 启用 3D 可视化

```
1. 仿真运行时自动打开 Mechanics Explorer
2. 或手动打开: Simulation > Mechanics Explorer
3. 可视化设置:
   - 显示坐标系: Show Frames
   - 显示关节: Show Joint Frames
   - 背景色: 设置
   - 相机跟随: Body Tracking
```

### 6.2 可视化设置

| 设置项 | 推荐值 | 说明 |
|--------|--------|------|
| Show Frames | ✅ | 显示坐标系方向 |
| Show Joint Frames | ✅ | 显示关节自由度 |
| Camera Mode | Tracking | 相机跟随飞行器 |
| Trail | ✅ | 显示飞行轨迹 |
| Background | Sky | 增强视觉效果 |

### 6.3 动画回放

```
仿真完成后:
1. Mechanics Explorer > 回放按钮
2. 可调节回放速度
3. 可截图或录制视频
4. 支持多视角同时查看
```

---

## 7. Simscape + Simulink 联合仿真

### 7.1 架构设计

信号流和物理域之间的接口是联合仿真的关键：

```
┌─────────────────────────────────────────────────────────────┐
│  混合仿真架构                                                  │
│                                                             │
│  ┌─── Signal-based (Simulink) ────┐  ┌─── Physical (Simscape) ───┐ │
│  │                                │  │                           │ │
│  │  Controller ──> Motor Model ──────> Thrust Force ──> 6DOF   │ │
│  │       ▲                        │  │                  Joint   │ │
│  │       │    ┌──────────────┐    │  │                    │     │ │
│  │       └────│ Joint Sensor │<─────│ Position/Velocity  │     │ │
│  │            │ (PS-Simulink)│    │  │                           │ │
│  │            └──────────────┘    │  └───────────────────────────┘ │
│  └────────────────────────────────┘                              │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 接口模块

| 方向 | 模块 | 功能 |
|------|------|------|
| Simulink → Simscape | `Simulink-PS Converter` | 信号转换为物理量 |
| Simscape → Simulink | `PS-Simulink Converter` | 物理量转换为信号 |
| Simscape → Simscape | 物理端口直接连接 | 无需转换 |

### 7.3 联合仿真求解器配置

```
Configuration Parameters:
  Solver: ode15s (推荐) 或 ode23t
  原因: Simscape 方程通常是刚性的 (stiff)
  Max step size: 0.01
  Simscape > Solver: 配置物理网络求解器

  注意: 不要使用 ode45，Simscape 方程可能导致步长极小
```

---

## 8. Simscape Electrical: 电机/电池建模

### 8.1 电机建模

Simscape Electrical 提供了预置的直流电机模型：

```
模块: DC Motor (Simscape Electrical > Electromachines > DC Motor)

参数:
  Rated voltage:    12 V
  Rated speed:      10000 rpm
  Rated torque:     0.5 N·m
  Armature resistance: 0.5 Ohm
  Armature inductance: 0.001 H
  Rotor inertia:    3.357e-6 kg·m^2
```

### 8.2 电池建模

```
模块: Battery (Simscape > Electrical > Sources)

常用模型:
  - Battery (简单): 理想电压源 + 内阻
  - Battery (Table-based): 基于 SOC-OCV 查表
  - Battery (Equivalent Circuit): RC 等效电路

参数:
  Rated voltage:    12 V (3S LiPo)
  Rated capacity:   5000 mAh
  Internal resistance: 0.01 Ohm
  Initial SOC:      100%
```

### 8.3 ESC (电调) 建模

```
模块: PWM-Controlled 2-quadrant Chopper

功能: 将 PWM 信号转换为电机电压
参数:
  PWM frequency:    400 Hz
  Supply voltage:   来自 Battery
  Control signal:   PWM duty cycle (0~1)
```

---

## 9. 两种建模方法对比

### 9.1 Simscape vs Aerospace Blockset

| 特性 | Simscape Multibody | Aerospace Blockset |
|------|--------------------|--------------------|
| 建模方式 | 物理连接，自动生成方程 | 信号流，手动搭建方程 |
| 学习曲线 | 较低（不需推导方程） | 较高（需要理解动力学） |
| 灵活性 | 受限于库组件 | 完全自定义 |
| 3D 可视化 | 内置 Mechanics Explorer | 需要额外配置 |
| 多体扩展 | 容易添加关节、柔体 | 困难 |
| 控制器集成 | 需要接口模块 | 原生 Simulink |
| 仿真速度 | 较慢（物理网络求解） | 较快 |
| 代码生成 | 支持但复杂 | 容易 |
| 适用场景 | 结构设计、物理原型 | 控制算法开发 |

### 9.2 选择建议

```
选择 Simscape Multibody:
  ✅ 需要精确的物理模型（多关节、柔性体）
  ✅ 需要 3D 可视化
  ✅ 需要与电气/流体系统耦合
  ✅ 不想手动推导动力学方程

选择 Aerospace Blockset (或自建 6DOF):
  ✅ 控制算法开发为主
  ✅ 需要代码生成到飞控板
  ✅ 追求仿真速度
  ✅ 需要完全自定义的动力学方程
```

### 9.3 混合方案

实际项目中常将两种方法结合使用：

```
┌─────────────────────────────────────────────────────┐
│  混合仿真架构                                         │
│                                                     │
│  Simscape 部分:                                      │
│    - 机架结构 (Simscape Multibody)                    │
│    - 电机/电池 (Simscape Electrical)                  │
│                                                     │
│  Simulink 部分:                                      │
│    - 控制器 (自定义 Simulink 模型)                     │
│    - 空气动力学 (自定义方程)                           │
│    - 环境模型 (风、大气)                               │
│    - GPS/IMU 传感器模型                              │
└─────────────────────────────────────────────────────┘
```

---

## 10. 参考资源

### 10.1 MathWorks 官方示例

| 示例名称 | 说明 | 路径 |
|----------|------|------|
| Quadcopter Drone Model with Simscape | 官方四旋翼示例 | MATLAB > Add-Ons > Examples |
| Simscape Multibody Drone Assembly | 组装工作流示例 | MathWorks File Exchange |
| Electric Drone | 电气-机械联合仿真 | MathWorks Examples |

### 10.2 GitHub 参考项目

| 项目 | 链接 | 说明 |
|------|------|------|
| Quadcopter-Drone-Model-Simscape | github.com/mathworks/Quadcopter-Drone-Model-Simscape | MathWorks 官方四旋翼模型 |
| LADAC | github.com/iff-gsc/LADAC | 德国航空航天中心 UAV 工具箱 |

### 10.3 学习路径

```
入门:
  1. MathWorks Simscape Multibody 入门教程
  2. 官方 Quadcopter 示例拆解
  3. 从简单模型开始 (单个刚体 + 6DOF Joint)

进阶:
  4. CAD 导入实际机架
  5. Simscape Electrical 集成电机/电池
  6. 与信号流控制器联合仿真

高级:
  7. 柔性体建模
  8. 自定义物理域
  9. 多 UAV 协同仿真
```

---

## 思考题

1. **Simscape Multibody 的物理连接和传统 Simulink 的信号连接有什么本质区别？这种区别如何影响建模方式？**

2. **在 Simscape 中搭建四旋翼机架时，为什么每个机臂都需要一个 Rigid Transform 和一个 Welded Joint？能否用其他方式实现？**

3. **如果要模拟四旋翼的机臂柔性变形（如碳纤维机臂在大推力下的弯曲），Simscape 如何实现？**

4. **在 Simscape + Simulink 联合仿真中，Simulink-PS Converter 和 PS-Simulink Converter 分别起什么作用？为什么需要这两个接口模块？**

5. **对比 Simscape Multibody 和手动搭建的 6DOF 模型，在仿真结果一致的情况下，各自的优缺点是什么？**

<details>
<summary>参考答案</summary>

**1.** 本质区别：信号连接是因果关系（输入驱动输出，单向流动），物理连接是双向的（力和运动同时传递，互为因果）。影响：信号流建模需要手动推导因果关系（谁是输入、谁是输出），物理建模只需声明部件之间的连接关系，求解器自动处理因果关系。例如电机推力驱动旋翼旋转，同时旋翼反作用力矩也作用在电机上，物理连接自动处理这种双向耦合。

**2.** Rigid Transform 定义机臂相对于机身的几何变换（平移+旋转），Welded Joint 将变换后的刚体固定在机身上。可以替代的方式：(1) 直接在 Solid 模块的 Reference Frame 中设置偏移量；(2) 使用多层 Rigid Transform 串联；(3) 在一个 Rigid Transform 中同时设置平移和旋转。推荐方式是将变换和连接分开，便于模块化和参数化。

**3.** Simscape Multibody 支持柔体建模：(1) 使用 `Reduced Order Flexible Solid` 模块，基于模态叠加法，在 CAD 中生成柔体网格和模态数据后导入；(2) 使用 `Flexible Beam` 简化模型，将机臂建模为欧拉-伯努利梁；(3) 将机臂分割为多个刚体段，用柔性关节连接。方法 1 精度最高但计算量大，方法 3 最简单但精度有限。

**4.** Simulink-PS Converter 将 Simulink 信号（如控制器输出的推力指令）转换为 Simscape 物理信号，注入物理网络中作为驱动力。PS-Simulink Converter 将 Simscape 物理信号（如关节的位置、速度）转换为 Simulink 信号，供控制器使用。需要两个模块是因为 Simulink 和 Simscape 使用不同的信号类型和求解机制，不能直接连接。

**5.** Simscape Multibody 优点：不需推导方程、自动处理约束和接触、内置 3D 可视化、支持多物理域耦合、易于扩展（添加关节、柔体）。缺点：仿真较慢、代码生成较困难、灵活性受限于库组件。手动 6DOF 模型优点：仿真快、代码生成容易、完全自定义、对理解动力学原理有帮助。缺点：需要手动推导方程、无内置可视化、扩展到多体系统困难。实际项目建议：控制算法开发用手动 6DOF，物理验证和可视化用 Simscape。

</details>
