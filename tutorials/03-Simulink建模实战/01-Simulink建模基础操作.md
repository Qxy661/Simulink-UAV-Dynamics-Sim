# Simulink 建模基础操作

> 预计阅读：20 分钟 | 前置知识：MATLAB 基础操作、控制系统基本概念

本文是 Simulink 建模实战系列的第一篇，帮助你快速掌握 Simulink 的核心操作，为后续无人机动力学建模打下坚实基础。

---

## 1. Simulink 界面总览

启动 Simulink 后，你会看到三个核心区域：

| 区域 | 功能 | 快捷键 |
|------|------|--------|
| **Library Browser** | 浏览和搜索所有可用模块库 | `Ctrl+Shift+L` |
| **Canvas (画布)** | 拖拽模块、连线、构建模型的主工作区 | — |
| **Configuration Parameters** | 求解器、步长、数据导入导出等全局设置 | `Ctrl+E` |

打开 Simulink 的两种方式：

```
方式一：MATLAB 命令行输入
>> simulink

方式二：点击 MATLAB 工具栏的 Simulink 按钮
```

创建新模型：`File > New > Model` 或 `Ctrl+N`。

---

## 2. 核心模块库分类

Simulink 的模块库按功能分为多个大类。以下是无人机建模中最常用的模块库：

### 2.1 Sources (信号源)

| 模块名 | 功能 | 典型用途 |
|--------|------|----------|
| `Constant` | 输出恒定值 | 期望姿态角、油门指令 |
| `Step` | 阶跃信号 | 测试阶跃响应 |
| `Sine Wave` | 正弦波 | 测试频率响应 |
| `From Workspace` | 从 MATLAB 工作区读取数据 | 导入飞行日志 |
| `Clock` | 当前仿真时间 | 时间相关逻辑 |
| `Signal Builder` | 分段信号编辑器 | 设计飞行任务轨迹 |
| `Ramp` | 斜坡信号 | 测试跟踪性能 |

### 2.2 Sinks (信号输出)

| 模块名 | 功能 | 典型用途 |
|--------|------|----------|
| `Scope` | 示波器，实时显示波形 | 观察姿态、位置曲线 |
| `To Workspace` | 将数据写入 MATLAB 工作区 | 后处理和绘图 |
| `Display` | 数字显示当前值 | 检查稳态值 |
| `To File` | 写入 MAT 文件 | 持久化存储 |
| `Stop Simulation` | 条件触发停止仿真 | 安全边界检测 |

### 2.3 Math Operations (数学运算)

| 模块名 | 功能 | 典型用途 |
|--------|------|----------|
| `Gain` | 乘以常数 (K*u) | 比例控制、单位转换 |
| `Sum` | 加减运算 | 误差计算、力的合成 |
| `Product` | 乘除运算 | 力矩计算 |
| `Math Function` | 数学函数 (sin, cos, sqrt...) | 旋转矩阵计算 |
| `Trigonometric Function` | 三角函数 | 坐标变换 |
| `Dot Product` | 向量点积 | 力的投影 |
| `Cross Product` | 向量叉积 | 力矩计算 |

### 2.4 Continuous (连续系统)

| 模块名 | 功能 | 典型用途 |
|--------|------|----------|
| `Integrator` | 积分器 | 速度→位置, 角速度→姿态 |
| `Derivative` | 微分器 | 数值微分(尽量避免使用) |
| `Transfer Fcn` | 传递函数 | 电机、舵机模型 |
| `State-Space` | 状态空间模型 | 线性化动力学 |
| `Transport Delay` | 传输延迟 | 通信延迟建模 |

### 2.5 Discrete (离散系统)

| 模块名 | 功能 | 典型用途 |
|--------|------|----------|
| `Unit Delay` | 单步延迟 z^{-1} | 离散控制器 |
| `Discrete Transfer Fcn` | 离散传递函数 | 数字 PID 控制器 |
| `Zero-Order Hold` | 零阶保持器 | 连续信号离散化 |
| `Rate Transition` | 速率转换 | 多速率系统接口 |
| `Discrete PID Controller` | 离散 PID | 飞控内环 |

### 2.6 Subsystems (子系统)

| 模块名 | 功能 | 典型用途 |
|--------|------|----------|
| `Subsystem` | 封装一组模块为子系统 | 模块化建模 |
| `Enabled Subsystem` | 使能子系统 | 条件执行 |
| `Triggered Subsystem` | 触发子系统 | 事件驱动 |
| `Subsystem Reference` | 引用外部子系统文件 | 团队协作 |

---

## 3. 信号路由

信号路由决定了数据如何在模块之间传递，是构建复杂模型的关键。

### 3.1 Mux / Demux

```
Mux (合并): 将多个标量信号合并为向量信号
                    ┌─────────┐
  u1 ──────────────>│         │
  u2 ──────────────>│   Mux   │──────> [u1; u2; u3] (向量)
  u3 ──────────────>│         │
                    └─────────┘

Demux (拆分): 将向量信号拆分为标量信号
                    ┌─────────┐
  [u1; u2; u3] ───>│         │──────> u1
                    │  Demux  │──────> u2
                    │         │──────> u3
                    └─────────┘
```

### 3.2 Bus Creator / Bus Selector

Bus 信号适合传递结构化数据（如 UAV 状态向量）：

```
Bus Creator (总线创建):
                    ┌─────────────────┐
  position (3x1) ──>│                 │
  velocity (3x1) ──>│  Bus Creator    │──────> UAV_State (Bus)
  attitude  (3x1) ──>│                 │
  omega     (3x1) ──>│                 │
                    └─────────────────┘

Bus Selector (总线选择):
                    ┌─────────────────┐
  UAV_State ───────>│                 │──────> position (3x1)
                    │  Bus Selector   │──────> velocity (3x1)
                    │                 │
                    └─────────────────┘
```

### 3.3 Goto / From (远程连接)

当模型较大时，用 Goto/From 标签代替长连线：

```
Goto 标签:                    From 标签:
┌──────────┐                 ┌──────────┐
│ Goto     │                 │ From     │
│ [tag:pos]│ <── signal      │ [tag:pos]│ ──> signal
└──────────┘                 └──────────┘

相同标签名自动连接，无需物理连线！
```

> **实用技巧**：使用 `Goto/From` 可以大幅减少模型中的交叉线，提高可读性。建议用有意义的标签名，如 `attitude_cmd`、`motor_speed`。

---

## 4. 求解器选择指南

求解器的选择直接影响仿真精度和速度。

### 4.1 Fixed-step vs Variable-step

| 特性 | Fixed-step (定步长) | Variable-step (变步长) |
|------|---------------------|------------------------|
| 步长 | 固定不变 | 根据误差自动调整 |
| 精度 | 取决于步长设置 | 自动保证精度 |
| 速度 | 快，可预测 | 通常较慢 |
| 代码生成 | 支持 | 不支持(需要定步长) |
| 实时仿真 | 适合 | 不适合 |
| UAV 推荐 | **嵌入式代码生成必选** | 开发调试阶段首选 |

### 4.2 常用求解器对比

| 求解器 | 类型 | 阶数 | 适用场景 | UAV 建模推荐度 |
|--------|------|------|----------|----------------|
| `ode45` | 变步长 | 4-5 | 非刚性问题，**默认首选** | 调试阶段推荐 |
| `ode15s` | 变步长 | 1-5 | 刚性问题(多时间尺度) | 电池/热模型推荐 |
| `ode23` | 变步长 | 2-3 | 中等精度需求 | 不推荐 |
| `ode4` | 定步长 | 4 | 固定步长 RK4 | **代码生成推荐** |
| `ode1` | 定步长 | 1 | Euler 法，最快但精度低 | 仅原型验证 |
| `ode3` | 定步长 | 3 | 定步长 RK3 | 一般推荐 |

### 4.3 UAV 仿真推荐配置

```
开发调试阶段:
  求解器: ode45 (Variable-step)
  最大步长: 0.01 s
  相对误差: 1e-3
  绝对误差: 1e-6

代码生成阶段:
  求解器: ode4 (Fixed-step)
  固定步长: 0.001 s (1 kHz)

硬件在环 (HIL) 阶段:
  求解器: ode4 (Fixed-step)
  固定步长: 0.001 ~ 0.005 s
  (取决于飞控板计算能力)
```

---

## 5. 采样时间概念

Simulink 支持连续和离散两种采样时间，理解它们对 UAV 多速率系统建模至关重要。

### 5.1 采样时间类型

| 类型 | 标记 | 含义 | 示例 |
|------|------|------|------|
| 连续 | `[0, 0]` | 连续时间系统 | 动力学积分器 |
| 离散 | `[Ts, 0]` | 固定周期采样 | PID 控制器 (Ts=0.001) |
| 继承 | `[-1, 0]` | 从输入信号继承 | Gain, Sum 等 |
| 可变离散 | `[-2, 0]` | 可变采样时间 | 事件驱动模块 |
| 多速率 | 不同 Ts | 不同模块不同采样率 | 内环 1kHz，外环 100Hz |

### 5.2 UAV 典型多速率架构

```
┌─────────────────────────────────────────────────────────────┐
│                    UAV 多速率控制系统                          │
│                                                             │
│  1 kHz (0.001s) ─── 电机混控、PWM 输出                       │
│      ▲                                                      │
│  400 Hz (0.0025s) ── 姿态内环 (角速率 PID)                   │
│      ▲                                                      │
│  100 Hz (0.01s) ──── 姿态外环 (角度 PID)                     │
│      ▲                                                      │
│  50 Hz (0.02s) ───── 位置环、导航算法                         │
│      ▲                                                      │
│  10 Hz (0.1s) ────── 任务规划、通信链路                       │
│      ▲                                                      │
│  1 Hz (1.0s) ─────── 日志记录、健康监测                       │
└─────────────────────────────────────────────────────────────┘
```

> **注意**：Simulink 中混合连续和离散模块时，需要使用 `Rate Transition` 模块在不同采样率之间传递数据，避免数据竞争。

---

## 6. 信号记录与数据导出

仿真完成后，需要将数据导出到 MATLAB 工作区进行分析。

### 6.1 方法一：To Workspace 模块

```
设置参数:
  Variable name: sim_out
  Save format:   Timeseries (推荐) 或 Structure
  Sample time:   -1 (继承) 或指定值
```

仿真结束后在 MATLAB 中：
```matlab
>> plot(sim_out.Time, sim_out.Data)
>> xlabel('Time (s)'); ylabel('Value')
```

### 6.2 方法二：信号记录 (Signal Logging)

右键点击任意信号线 > `Enable Signal Logging`，信号线旁出现蓝色标记。

```matlab
>> logsout  % 自动创建的 Dataset 对象
>> plot(logsout{1}.Values)
```

### 6.3 方法三：仿真输出 (Simulation Output)

```matlab
>> simOut = sim('model_name');
>> simOut.logsout        % 信号记录
>> simOut.tout           % 时间向量
>> simOut.yout           % 输出记录
```

### 6.4 数据格式对比

| 格式 | 优点 | 缺点 | 推荐场景 |
|------|------|------|----------|
| `Timeseries` | 支持多采样率、自动插值 | 占用内存稍大 | **首选** |
| `Structure` | 兼容旧版本 | 不支持多采样率 | 兼容旧代码 |
| `Array` | 最省内存 | 仅支持等间隔采样 | 大规模仿真 |

---

## 7. 模型配置参数详解

`Ctrl+E` 打开 Configuration Parameters，以下是关键设置：

### 7.1 求解器设置 (Solver)

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| Type | Fixed-step / Variable-step | 根据阶段选择 |
| Solver | ode4 / ode45 | 见第4节 |
| Fixed-step size | 0.001 | 1kHz 适用于大多数 UAV |
| Max step size | 0.01 | 变步长模式下的上限 |
| Relative tolerance | 1e-3 | 精度要求 |
| Initial step size | auto | 通常让 Simulink 自动决定 |

### 7.2 数据导入/导出 (Data Import/Export)

| 参数 | 推荐设置 | 说明 |
|------|----------|------|
| Time | `tout` | 输出时间变量名 |
| Output | `yout` | 输出变量名 |
| Signal logging | ✅ 勾选 | 启用信号记录 |
| Signal logging name | `logsout` | 信号记录变量名 |
| Limit data points | 取消勾选 | 避免数据截断 |

### 7.3 优化设置 (Optimization)

| 参数 | 推荐设置 | 说明 |
|------|----------|------|
| Inline parameters | 调试时关闭 | 允许在线修改参数 |
| Block reduction | ✅ 勾选 | 简化等效模块 |
| Conditional input branch execution | ✅ 勾选 | 提升 Switch 模块性能 |

### 7.4 代码生成 (Code Generation) — 后续章节详述

| 参数 | 推荐设置 | 说明 |
|------|----------|------|
| System target file | `ert.tlc` | 嵌入式实时目标 |
| Language | C | 嵌入式通常用 C |
| Generate code only | ✅ 勾选 | 仅生成代码不编译 |

---

## 8. 常用键盘快捷键

| 快捷键 | 功能 | 使用频率 |
|--------|------|----------|
| `Ctrl+E` | 打开模型配置 | 极高 |
| `Ctrl+D` | 更新模型(编译检查) | 极高 |
| `Ctrl+B` | 构建模型(代码生成) | 高 |
| `Ctrl+Shift+L` | 打开 Library Browser | 高 |
| `Ctrl+G` | 创建子系统 | 高 |
| `Ctrl+Shift+G` | 从子系统中移出模块 | 中 |
| `Ctrl+T` | 添加标签(Tag)到信号线 | 中 |
| `R` | 旋转选中模块 90° | 高 |
| `Ctrl+Shift+拖拽` | 从模块拉出新连线 | 高 |
| `双击信号线` | 添加信号名 | 高 |
| `Ctrl+Shift+I` | 自动布局 | 中 |
| `Space` | 暂停/恢复仿真 | 调试时高 |
| `Ctrl+.` | 停止仿真 | 调试时高 |
| `Ctrl+Z` | 撤销 | 极高 |
| `Ctrl+Y` | 重做 | 高 |

---

## 9. 初学者常见错误

| 错误现象 | 原因 | 解决方法 |
|----------|------|----------|
| 信号维度不匹配 | Mux 合并维度与模块期望不一致 | 用 `Signal Specification` 或 `Reshape` 模块调整 |
| 代数环 (Algebraic Loop) | 输出直接反馈到输入，无延迟 | 添加 `Unit Delay` 或 `Memory` 模块 |
| 仿真发散 (NaN/Inf) | 初始值不合理或步长过大 | 检查初始条件，减小步长 |
| 编译报错 "Unconnected input" | 模块输入端口未连接 | 连接信号或使用 `Ground` 模块 |
| 编译报错 "Unconnected output" | 模块输出端口未连接 | 连接信号或使用 `Terminator` 模块 |
| Scope 无波形 | 仿真未运行或信号未正确连接 | `Ctrl+D` 检查连接，运行仿真 |
| 模型运行极慢 | 变步长求解器步长过小或模型过于复杂 | 调整求解器参数或简化模型 |
| 数据类型不匹配 | 整数/浮点混合运算 | 使用 `Data Type Conversion` 模块 |
| 连续离散混用出错 | 缺少 Rate Transition | 在接口处添加 `Rate Transition` |

---

## 10. 调试工具

### 10.1 Signal Display (信号标注)

右键信号线 > `Signal Display` 或按 `Ctrl+Shift+T`，可在连线上实时显示数值。

### 10.2 Probe 模块

```
信号线 ──> [Probe] ──> [Display]
               │
               └── 可查看: 维度、采样时间、数据类型、复数属性
```

### 10.3 Display 模块

在关键节点放置 `Display` 模块，仿真时实时查看数值。

### 10.4 Assertion 模块

```
信号线 ──> [Assertion] ──> 触发错误
               │
               条件: 信号值超出预期范围时报警
```

### 10.5 Model Advisor

`Analysis > Model Advisor` 可自动检查模型中的常见问题：
- 代数环检测
- 数据类型一致性
- 未连接端口
- 潜在的溢出问题

### 10.6 Simulation Stepping (单步仿真)

| 操作 | 快捷键 | 说明 |
|------|--------|------|
| 开始 | `Ctrl+T` | 开始仿真 |
| 暂停 | `Space` | 暂停仿真 |
| 步进 | — | 逐步执行(通过工具栏) |
| 停止 | `Ctrl+.` | 停止仿真 |

---

## 11. 实用技巧

| 编号 | 技巧 | 说明 |
|------|------|------|
| 1 | 善用注释 | `Ctrl+Shift+X` 添加文本注释，双击注释可编辑 |
| 2 | 颜色区分 | 连续信号线用黑色，离散用蓝色(自动) |
| 3 | 模型浏览器 | `Ctrl+Shift+I` 打开 Model Explorer 查看所有数据 |
| 4 | 信号维度标注 | `Format > Port/Signal Displays > Signal Dimensions` |
| 5 | 采样时间颜色 | `Format > Sample Time Colors` 用颜色区分采样率 |
| 6 | 版本控制 | 保存为 `.slx` 格式，兼容 Git 管理 |
| 7 | 快速搜索 | `Ctrl+F` 在模型中搜索模块名或参数 |
| 8 | 回调函数 | 模型属性中设置 `PreLoadFcn`、`InitFcn` 等回调 |

---

## 思考题

1. **为什么 UAV 仿真推荐开发阶段用 ode45，而代码生成阶段用 ode4？**

2. **在 UAV 飞控模型中，如果内环角速率控制运行在 400Hz，外环角度控制运行在 100Hz，当外环向内环传递指令时，需要添加什么模块？为什么？**

3. **某同学搭建的 UAV 模型仿真时出现 "Algebraic loop" 警告，且仿真速度极慢。请分析可能的原因并提出至少两种解决方案。**

4. **To Workspace 模块的 Save format 选择 Timeseries 和 Structure 有什么区别？在什么场景下必须选择 Timeseries？**

5. **如果需要在仿真过程中实时修改某个增益参数（用于调参），Configuration Parameters 中需要关闭哪个选项？**

<details>
<summary>参考答案</summary>

**1.** ode45 是变步长高阶求解器，能自动调整步长保证精度，适合开发阶段快速验证算法正确性。ode4 是定步长 4 阶 Runge-Kutta 法，步长固定，支持嵌入式代码生成，因为实际飞控硬件使用固定频率的定时器中断，必须用定步长求解器。

**2.** 需要添加 **Rate Transition** 模块。因为内外环采样率不同（400Hz vs 100Hz），直接连接会导致数据竞争问题。Rate Transition 模块负责在不同采样率之间安全地传递数据，确保读写操作的原子性。

**3.** 原因：控制器输出直接反馈到输入端，形成无延迟的闭环。解决方案：
   - (a) 在反馈回路中添加 `Unit Delay` 或 `Memory` 模块，引入一个采样周期的延迟
   - (b) 使用离散 PID 控制器代替连续 PID，离散控制器自带延迟
   - (c) 在 Configuration Parameters 中勾选 "Minimize algebraic loop occurrences"

**4.** Timeseries 格式支持多采样率数据（不同信号有不同的采样时间），会自动记录每个信号的时间戳。Structure 格式要求所有信号采样时间相同。当模型中存在连续和离散模块混合使用时，必须选择 Timeseries，否则离散信号的数据点可能丢失或对齐错误。

**5.** 需要关闭 `Optimization > Inline parameters` 选项。默认情况下 Simulink 会将参数内联到生成的代码中（不可修改），关闭此选项后参数作为可调变量存在，仿真过程中可通过 `set_param` 修改。

</details>
