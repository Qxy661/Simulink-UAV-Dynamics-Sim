# Aerospace Blockset 详解

> 预计阅读：25 分钟 | 前置知识：Simulink 基础操作、6DOF 动力学概念、坐标变换基础

Aerospace Blockset 是 MathWorks 为航空航天仿真提供的专用模块库，包含六自由度动力学、大气模型、风场模型、坐标变换等核心模块。本文详解其在 UAV 建模中的应用。

---

## 1. Aerospace Blockset 概述

### 1.1 主要功能模块

| 模块类别 | 代表模块 | UAV 建模用途 |
|----------|----------|-------------|
| **Equations of Motion** | 6DOF (Euler Angles), 6DOF (Quaternion) | 核心动力学 |
| **Environment** | ISA Atmosphere, Wind (Dryden/Von Karman) | 环境建模 |
| **Coordinate Systems** | Direction Cosine Matrix, Euler Angles | 坐标变换 |
| **Propulsion** | Engine, Propeller | 推进系统 |
| **Flight Parameters** | Airspeed, Altitude | 飞行参数计算 |
| **Utilities** | Quaternion Operations, Unit Conversion | 工具函数 |

### 1.2 安装与验证

```matlab
% 检查是否安装
>> ver('aeroblks')

% 或在 Simulink 中查看
Library Browser > Aerospace Blockset
```

---

## 2. 6DOF 模块详解

### 2.1 欧拉角 6DOF 模块

```
模块路径: Aerospace Blockset > Equations of Motion > 6DOF (Euler Angles)

┌────────────────────────────────────────────────────┐
│                6DOF (Euler Angles)                  │
│                                                    │
│  输入端口:                                          │
│    Fxyz  (3x1): 合力 (N) [Fx; Fy; Fz]             │
│    Mxyz  (3x1): 合力矩 (N·m) [Mx; My; Mz]         │
│    Mass  (1x1): 质量 (kg)                          │
│    Inertia (3x3): 惯量矩阵                         │
│                                                    │
│  输出端口:                                          │
│    Ve   (3x1): 地球坐标系速度 (m/s)                 │
│    Xe   (3x1): 地球坐标系位置 (m)                   │
│    Euler (3x1): 欧拉角 (rad) [φ; θ; ψ]             │
│    Vb   (3x1): 机体坐标系速度 (m/s)                 │
│    pqr  (3x1): 机体角速度 (rad/s)                   │
│    DCM  (3x3): 方向余弦矩阵                        │
│                                                    │
└────────────────────────────────────────────────────┘
```

### 2.2 四元数 6DOF 模块

```
模块路径: Aerospace Blockset > Equations of Motion > 6DOF (Quaternion)

区别:
  输出: euler 替换为 quaternion (4x1) [q0; q1; q2; q3]
  优势: 无万向锁，适合大机动飞行
  劣势: 欧拉角需要额外计算
```

### 2.3 关键参数配置

| 参数 | 说明 | 推荐设置 |
|------|------|----------|
| **Units** | 角度单位 | `Radians` |
| **Mass type** | 质量类型 | `Fixed` (UAV 质量基本恒定) |
| **Inertia type** | 惯量类型 | `Fixed` |
| **Initial position** | 初始位置 [x; y; z] | `[0; 0; -100]` (NED 下 100m 高度) |
| **Initial velocity (body)** | 初始机体系速度 | `[0; 0; 0]` |
| **Initial Euler angles** | 初始欧拉角 | `[0; 0; 0]` |
| **Initial angular rates** | 初始角速度 | `[0; 0; 0]` |

### 2.4 坐标系约定

Aerospace Blockset 默认使用 **NED 坐标系**：

```
NED 坐标系 (North-East-Down):
  X: 北 (North)
  Y: 东 (East)
  Z: 下 (Down)

  与常见 UAV 文档中 Z-up 的区别:
  ┌─────────────────────────────────────────┐
  │  Z-up (常见):                           │
  │    Z 轴向上为正                          │
  │    重力: [0, 0, -g]                     │
  │    高度: Z 的正值                        │
  │                                         │
  │  NED (Aerospace Blockset):              │
  │    Z 轴向下为正                          │
  │    重力: [0, 0, +g]                     │
  │    高度: -Z 的值 (或使用 -Z)             │
  └─────────────────────────────────────────┘

  转换: 如果模型用 Z-up，需要对 Z 轴取反
```

> **重要**：使用 Aerospace Blockset 时务必注意坐标系约定。NED 下 Z 轴向下，高度 = -Ze。在模型中需要添加 `Gain(-1)` 模块进行转换。

---

## 3. 大气模型

### 3.1 ISA 标准大气

```
模块路径: Aerospace Blockset > Environment > Atmosphere Models

┌──────────────────────────────────────────┐
│           ISA Atmosphere                  │
│                                          │
│  输入: h (高度, m)                        │
│                                          │
│  输出:                                    │
│    T   (温度, K)                          │
│    a   (音速, m/s)                        │
│    P   (压强, Pa)                         │
│    rho (密度, kg/m^3)                     │
│                                          │
└──────────────────────────────────────────┘
```

### 3.2 标准大气参数表

| 高度 (m) | 温度 (K) | 密度 (kg/m^3) | 压强 (Pa) | 音速 (m/s) |
|----------|----------|---------------|-----------|------------|
| 0 | 288.15 | 1.2250 | 101325 | 340.3 |
| 500 | 284.90 | 1.1673 | 95461 | 338.4 |
| 1000 | 281.65 | 1.1117 | 89874 | 336.4 |
| 1500 | 278.40 | 1.0581 | 84556 | 334.5 |
| 2000 | 275.15 | 1.0066 | 79501 | 332.5 |
| 3000 | 268.65 | 0.9093 | 70121 | 328.6 |

### 3.3 COESA 大气模型

COESA (Committee on Extension to the Standard Atmosphere) 提供更高精度的大气模型，覆盖到 1000km 高度：

```
模块: COESA Atmosphere Model
适用: 高空 UAV (>30km)
精度: 比 ISA 更高，包含气体成分变化
```

### 3.4 自定义大气模型

对于特殊环境（如高原、热带），可自定义大气参数：

```matlab
% 自定义大气参数
T0 = 300;       % 地面温度 (K) - 热带
P0 = 101325;    % 地面压强 (Pa)
rho0 = 1.177;   % 地面密度 (kg/m^3)
L = 0.0065;     % 温度递减率 (K/m)

% 使用 Lookup Table 或 MATLAB Function 模块实现
```

---

## 4. 风场模型

### 4.1 Dryden 风湍流模型

```
模块路径: Aerospace Blockset > Environment > Wind Models > Dryden Wind Turbulence Model

Dryden 模型特点:
  - 基于有理传递函数近似
  - 计算效率高
  - 适用于低空 (< 300m) 和中等湍流
  - 参数基于 MIL-F-8785C 标准
```

### 4.2 Dryden 模型参数

| 参数 | 说明 | 典型值 |
|------|------|--------|
| **W20** | 20ft 高度风速 | 5~15 m/s |
| **Turbulence intensity** | 湍流强度 | Light / Moderate / Severe |
| **Wind speed at 20ft** | 参考风速 | 10 m/s |
| **Wingspan** | 翼展 | 1.5 m |
| **Airplane length** | 机身长度 | 0.5 m |
| **Turbulence model** | 湍流类型 | Discrete / Continuous |

### 4.3 Von Karman 风湍流模型

```
模块: Von Karman Wind Turbulence Model

与 Dryden 对比:
┌──────────────┬─────────────┬─────────────┐
│ 特性          │ Dryden      │ Von Karman  │
├──────────────┼─────────────┼─────────────┤
│ 频谱精度      │ 一般        │ 更高         │
│ 计算效率      │ 高          │ 一般         │
│ 低频特性      │ 不够准确    │ 准确         │
│ 标准          │ MIL-F-8785C│ MIL-HDBK-1797│
│ UAV 推荐      │ ✅ 首选     │ 精度要求高时  │
└──────────────┴─────────────┴─────────────┘
```

### 4.4 稳态风模型

用于模拟常值风或风切变：

```matlab
% 稳态风场
wind_steady = [5; 0; 0];  % 北向 5 m/s 风

% 风切变 (对数律)
% V(z) = V_ref * ln(z/z0) / ln(z_ref/z0)
% z0: 地面粗糙度长度 (草地 ~0.03m)
```

---

## 5. 重力模型

### 5.1 可用的重力模型

| 模型 | 说明 | 适用场景 |
|------|------|----------|
| **Simple** | 恒定 g = 9.80665 m/s^2 | 低空 UAV（推荐） |
| **WGS-84** | 基于纬度和高度变化 | 高精度需求、远程 UAV |
| **Custom** | 用户自定义 | 特殊天体 |

### 5.2 WGS-84 重力模型

```
模块: WGS84 Gravity Model

输入:
  h: 海拔高度 (m)
  lat: 纬度 (deg)

输出:
  g: 当地重力加速度 (m/s^2)

公式: Somigliana 模型
  g = g_e * (1 + k*sin^2(φ)) / sqrt(1 - e^2*sin^2(φ))

  g_e = 9.7803253359 (赤道重力)
  k   = 0.00193185265241
  e   = 0.0818191908426 (第一偏心率)
```

### 5.3 选择建议

```
UAV 飞行高度 < 5000m, 纬度变化小:
  → 使用 Simple (g = 9.80665) 即可
  → 误差 < 0.3%

远程 UAV (>100km 距离), 需要高精度导航:
  → 使用 WGS-84
  → 考虑纬度和高度对重力的影响
```

---

## 6. 坐标变换模块

### 6.1 核心变换模块

| 模块名 | 功能 | 输入 | 输出 |
|--------|------|------|------|
| `Direction Cosine Matrix` | DCM 矩阵 | 欧拉角 / 轴角 | 3x3 DCM |
| `Euler Angles to DCM` | 欧拉角→DCM | [φ; θ; ψ] | 3x3 DCM |
| `DCM to Euler Angles` | DCM→欧拉角 | 3x3 DCM | [φ; θ; ψ] |
| `Euler Angles to Quaternions` | 欧拉角→四元数 | [φ; θ; ψ] | [q0; q1; q2; q3] |
| `Quaternions to Euler Angles` | 四元数→欧拉角 | [q0; q1; q2; q3] | [φ; θ; ψ] |
| `Quaternions to DCM` | 四元数→DCM | 4x1 四元数 | 3x3 DCM |
| `DCM to Quaternions` | DCM→四元数 | 3x3 DCM | 4x1 四元数 |
| `Flat Earth to LLA` | 平面→经纬高 | [x; y; z] | [lat; lon; alt] |

### 6.2 欧拉角旋转顺序

Aerospace Blockset 默认使用 **ZYX 旋转顺序**（偏航-俯仰-滚转）：

```
R = Rz(ψ) * Ry(θ) * Rx(φ)

展开:
R = [cosθcosψ,  sinφsinθcosψ-cosφsinψ,  cosφsinθcosψ+sinφsinψ]
    [cosθsinψ,  sinφsinθsinψ+cosφcosψ,  cosφsinθsinψ-sinφcosψ]
    [-sinθ,      sinφcosθ,                cosφcosθ              ]
```

### 6.3 坐标系转换工作流

```
UAV 建模中常见的坐标系转换:

  地球系 (NED) ──[DCM]──> 机体系 (Body)
       │                       │
       │                       │
       v                       v
  地球系力/位置           机体系力/速度
       │                       │
       │                       │
       └───── 6DOF 模块 ───────┘

  转换矩阵在 6DOF 模块内部自动处理
  但气动力计算时需要手动转换:
    F_body = DCM' * F_earth
    V_earth = DCM * V_body
```

---

## 7. 四元数运算模块

### 7.1 四元数运算列表

| 模块 | 功能 | 用途 |
|------|------|------|
| `Quaternion Multiply` | 四元数乘法 | 始态叠加 |
| `Quaternion Conjugate` | 四元数共轭 | 逆旋转 |
| `Quaternion Normalize` | 归一化 | 保持单位四元数 |
| `Quaternion Inverse` | 四元数逆 | 坐标系反变换 |
| `Quaternion Modular` | 四元数取模 | 检查归一化 |
| `Quaternion Rotation` | 四元数旋转 | 向量旋转 |

### 7.2 四元数基本运算

```
四元数定义: q = [q0, q1, q2, q3] = [q0, q_vec]
  q0: 标量部分 (实部)
  q_vec: 向量部分 (虚部)

乘法:
  q1 * q2 = [q1_0*q2_0 - q1_vec·q2_vec,
             q1_0*q2_vec + q2_0*q1_vec + q1_vec × q2_vec]

共轭:
  q* = [q0, -q1, -q2, -q3]

逆:
  q^(-1) = q* / |q|^2  (单位四元数时 q^(-1) = q*)

向量旋转:
  v_rotated = q * [0, v] * q*
```

### 7.3 四元数在 UAV 中的应用

```
用途 1: 始态表示
  比欧拉角更稳定，无万向锁

用途 2: 始态插值
  SLERP (球面线性插值) 用于轨迹规划

用途 3: 传感器融合
  IMU 数据融合 (扩展卡尔曼滤波) 常用四元数

用途 4: 控制器设计
  始态误差 = q_desired * q_current^(-1)
```

---

## 8. 气动力和力矩建模

### 8.1 基于查找表的气动模型

```
模块: n-D Lookup Table

输入: 气动参数 (α, β, V, δ_ail, δ_elev, δ_rud...)
输出: CL, CD, CY, Cl, Cm, Cn (气动系数)

模型结构:
  ┌──────────────────────────────────────────────────┐
  │             Aerodynamic Model                     │
  │                                                  │
  │  α (迎角) ──> ┌────────────┐                      │
  │  β (侧滑角) ──>│ n-D Lookup │──> CD, CL, CY       │
  │  V (空速)  ──>│   Table    │──> Cl (滚转力矩系数)  │
  │  δ_elev    ──>│            │──> Cm (俯仰力矩系数)  │
  │  δ_ail     ──>│            │──> Cn (偏航力矩系数)  │
  │  δ_rud     ──>│            │                      │
  │              └────────────┘                      │
  │                                                  │
  │  气动力: F = q * S * [CD; CL; CY]                  │
  │  气动力矩: M = q * S * b * [Cl; Cm; Cn]            │
  │                                                  │
  │  其中: q = 0.5*ρ*V^2 (动压)                       │
  │        S = 参考面积                                │
  │        b = 参考展长 (或平均气动弦长)                │
  └──────────────────────────────────────────────────┘
```

### 8.2 气动参数来源

| 来源 | 方法 | 精度 | 适用阶段 |
|------|------|------|----------|
| 经验公式 | 文献中的气动估算公式 | 低 | 概念设计 |
| CFD 仿真 | 计算流体力学模拟 | 高 | 详细设计 |
| 风洞实验 | 物理吹风实验 | 最高 | 验证阶段 |
| 飞行辨识 | 实飞数据辨识 | 中-高 | 调试阶段 |
| 开源数据 | LADAC 等开源库 | 中 | 学习和原型 |

### 8.3 简化的 UAV 气动模型

对于小型四旋翼，气动力相对较小，可使用简化模型：

```
简化气动阻力:
  F_drag = -0.5 * rho * Cd * A * |V| * V

  Cd: 阻力系数 (典型值 0.5~1.5)
  A: 迎风面积 (m^2)
  V: 相对风速 (m/s)

  在 Simulink 中用 Product 和 Gain 模块实现
```

---

## 9. 内置 6DOF vs 自建 6DOF 对比

### 9.1 功能对比

| 特性 | Aerospace 6DOF | 自建 6DOF |
|------|----------------|-----------|
| 坐标系 | NED (默认) | 自定义 (通常 Z-up) |
| 始态表示 | 欧拉角 / 四元数 | 通常欧拉角 |
| 旋转矩阵输出 | ✅ 可选 | 需手动添加 |
| 机体速度输出 | ✅ | 需手动添加 |
| 质量变化 | ✅ 支持 | 需手动添加 |
| 惯量变化 | ✅ 支持 | 困难 |
| 代码生成 | ✅ 优化 | 需手动优化 |
| 可视化 | 需额外配置 | 同左 |
| 仿真速度 | 快 | 相近 |
| 学习成本 | 低 (拖入即用) | 高 (需推导方程) |

### 9.2 选择建议

```
使用 Aerospace Blockset 6DOF:
  ✅ 快速搭建原型
  ✅ 需要标准大气和风场模型
  ✅ 项目使用 NED 坐标系
  ✅ 需要四元数表示

使用自建 6DOF:
  ✅ 需要自定义动力学方程
  ✅ 需要代码生成到特定硬件
  ✅ 需要 Z-up 坐标系
  ✅ 教学目的，理解底层原理
```

---

## 10. 典型 UAV 模型架构

使用 Aerospace Blockset 搭建 UAV 模型的标准架构：

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     UAV 模型 (Aerospace Blockset)                        │
│                                                                         │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐          │
│  │   Control     │    │   Motor /    │    │   Aerodynamics   │          │
│  │   System      │───>│   Propulsion │───>│   Model          │          │
│  │   (PID)       │    │   Model      │    │   (Lookup Table) │          │
│  └──────────────┘    └──────────────┘    └────────┬─────────┘          │
│                                                    │                    │
│                                              F_b (力)                   │
│                                              M_b (力矩)                 │
│                                                    │                    │
│  ┌──────────────┐    ┌──────────────┐    ┌────────▼─────────┐          │
│  │   Wind        │    │   Gravity    │───>│   6DOF           │          │
│  │   Model       │    │   Model      │    │   (Euler/Quat)   │          │
│  │   (Dryden)    │    │   (WGS84)    │    │                  │          │
│  └──────┬───────┘    └──────────────┘    └────────┬─────────┘          │
│         │                                         │                    │
│         │    ┌───────────────────┐          ┌──────┴──────┐            │
│         │    │   Atmosphere      │          │   State     │            │
│         └───>│   Model           │          │   Output    │            │
│              │   (ISA)           │          │   (x,v,euler│            │
│              └───────────────────┘          │    omega)   │            │
│                                             └─────────────┘            │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 11. 参考资源

### 11.1 开源项目

| 项目 | 链接 | 说明 |
|------|------|------|
| LADAC | github.com/iff-gsc/LADAC | 德国航空航天中心 UAV 动力学工具箱 |
| FlightGear Interface | MathWorks 工具箱 | FlightGear 可视化接口 |
| UAV Toolbox | MathWorks 官方 | UAV 专用工具箱 |
| PX4 MATLAB Toolbox | 开源 | PX4 飞控 MATLAB 接口 |

### 11.2 学习路径

```
入门:
  1. Aerospace Blockset Getting Started
  2. 搭建简单 6DOF 模型 + ISA 大气
  3. 添加 Dryden 风湍流

进阶:
  4. 气动查找表建模
  5. 电机/螺旋桨推进模型
  6. 控制器集成

高级:
  7. 多 UAV 编队仿真
  8. 与 FlightGear 联合可视化
  9. 代码生成到飞控硬件
```

---

## 思考题

1. **Aerospace Blockset 使用 NED 坐标系，而许多 UAV 文档使用 Z-up 坐标系。在同一个模型中混用两种约定会带来什么问题？如何正确处理坐标系转换？**

2. **Dryden 和 Von Karman 两种风湍流模型在频谱特性上有什么区别？为什么小型 UAV 仿真中通常选择 Dryden 模型？**

3. **在气动力建模中，为什么需要用 n-D Lookup Table 而不是解析公式？Lookup Table 的数据点数量和分布对仿真精度有什么影响？**

4. **使用内置 6DOF 模块和自己搭建积分链（积分加速度得到速度，积分速度得到位置）相比，有什么优势？内置模块在哪些细节上做了优化？**

5. **如果要模拟一架飞越山脉的 UAV，大气模型和重力模型应该如何选择？需要考虑哪些额外因素？**

<details>
<summary>参考答案</summary>

**1.** 混用 NED 和 Z-up 坐标系会导致：(1) Z 轴方向相反，高度计算符号错误；(2) 重力向量方向错误（NED 下 g 为正，Z-up 下 g 为负）；(3) 旋转矩阵定义不同，姿态角解释错误；(4) 气动力计算中的动压和迎角可能不正确。处理方法：在模型入口处统一约定，如果主体用 Z-up，在连接 Aerospace Blockset 模块前添加坐标转换：`[N;E;D] = [x; y; -z]`，或者使用 `Axis Transformation` 模块统一处理。

**2.** Dryden 模型使用有理传递函数近似湍流频谱，高频段衰减较快，低频段有一定偏差。Von Karman 模型更精确地匹配了理论湍流频谱，特别是在低频段。小型 UAV 选择 Dryden 的原因：(1) 计算效率高，适合实时仿真；(2) 小型 UAV 飞行高度低、速度慢，Dryden 的低频误差影响不大；(3) 参数化简单，便于根据 MIL 标准设置。

**3.** 使用 Lookup Table 而非解析公式的原因：(1) 气动系数与迎角、侧滑角、操纵面偏角之间的关系是非线性的，没有通用的解析表达式；(2) 气动数据来源于 CFD 或风洞实验，本身就是离散数据点；(3) 失速、分离等复杂流动现象无法用简单公式描述。数据点数量影响：太少会导致插值误差大，太多会增加内存和计算时间。分布建议：在非线性区域（如失速迎角附近）加密数据点，在线性区域适当稀疏。

**4.** 内置 6DOF 模块的优势：(1) 使用旋转矩阵/四元数更新始态，避免欧拉角运动学方程的万向锁和奇异点；(2) 自动处理地球坐标系和机体坐标系之间的力转换；(3) 内置数值稳定性优化，如四元数归一化；(4) 支持可变质量和惯量；(5) 经过 MathWorks 验证和优化，代码生成效率高。自己搭建的积分链在始态更新时容易出现数值漂移和坐标系混乱。

**5.** 飞越山脉时需要考虑：(1) 大气模型：应使用 ISA + 地形相关修正，或自定义大气模型考虑山区温度、压强异常；(2) 重力模型：使用 WGS-84，因为山区海拔变化大（2000~5000m），简单恒定 g 误差可达 0.1%~0.3%；(3) 风场：山区风切变和湍流强度远大于平原，应使用更保守的 Dryden 参数或自定义山地风场；(4) 地形模型：需要数字高程模型 (DEM) 数据防止撞山；(5) 气压高度表误差：大气压受地形影响，需要 GPS 高度修正。

</details>
