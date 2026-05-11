# S-Function 与自定义模块

> 预计阅读：22 分钟 | 前置知识：Simulink 基础操作、MATLAB 编程基础、C 语言基础 (可选)

当 Simulink 内置模块无法满足建模需求时，S-Function (System Function) 允许你用代码实现任意复杂的自定义模块。本文介绍 MATLAB S-Function 和 C MEX S-Function 的编写方法。

---

## 1. S-Function 概述

### 1.1 什么是 S-Function

S-Function 是 Simulink 中自定义模块的编程接口，可以用 MATLAB、C/C++、Fortran 等语言编写。本质上是一个定义了模块行为的函数，Simulink 引擎通过一组回调方法与之交互。

### 1.2 什么时候需要 S-Function

| 场景 | 是否需要 S-Function | 替代方案 |
|------|---------------------|----------|
| 简单数学运算 | ❌ | 内置 Math 模块 |
| 复杂数学公式 | ⚠️ | MATLAB Function 模块 |
| 自定义状态方程 | ✅ | — |
| 调用外部 C 库 | ✅ | — |
| 自定义离散逻辑 | ⚠️ | Stateflow |
| 硬件通信协议 | ✅ | — |
| 性能关键的算法 | ✅ (C MEX) | — |

### 1.3 S-Function 类型对比

| 类型 | 语言 | 开发难度 | 仿真速度 | 代码生成 | 适用场景 |
|------|------|----------|----------|----------|----------|
| **Level-1 MATLAB S-Function** | MATLAB | 低 | 慢 | ❌ | 原型验证 |
| **Level-2 MATLAB S-Function** | MATLAB | 中 | 慢 | ❌ | 算法原型 |
| **C MEX S-Function** | C/C++ | 高 | 快 | ✅ | **生产级代码** |
| **S-Function Builder** | C | 中 | 快 | ✅ | C 代码集成 |
| **MATLAB Function** | MATLAB | 低 | 中 | ✅ | **首选方案** |
| **System Composer** | 图形化 | 中 | 必 | ✅ | 架构设计 |

---

## 2. Level-1 vs Level-2 MATLAB S-Function

### 2.1 Level-1 (旧版，不推荐新项目使用)

```matlab
% Level-1 S-Function 格式
function [sys,x0,str,ts] = my_sfunc(t,x,u,flag)
  switch flag
    case 0  % 初始化
      [sys,x0,str,ts] = mdlInitializeSizes;
    case 1  % 连续状态微分
      sys = mdlDerivatives(t,x,u);
    case 2  % 离散状态更新
      sys = mdlUpdate(t,x,u);
    case 3  % 输出
      sys = mdlOutputs(t,x,u);
    case 9  % 仿真结束
      sys = mdlTerminate(t,x,u);
    otherwise
      error(['Unhandled flag = ',num2str(flag)]);
  end
end
```

### 2.2 Level-2 (推荐)

```matlab
% Level-2 S-Function 格式 (使用 Simulink.Block 对象)
function my_level2_sfcn(block)
  setup(block);
end

function setup(block)
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  block.SetPreCompPortInfoToDynamic;
  block.InputPort(1).Dimensions  = 3;
  block.OutputPort(1).Dimensions = 3;

  block.NumDialogPrms  = 1;
  block.DialogPrmsTunable = {'Tunable'};

  block.SampleTimes = [0.001 0];  % 1kHz 离散

  block.RegBlockMethod('Outputs', @Output);
  block.RegBlockMethod('Update', @Update);
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
end
```

### 2.3 Level-1 vs Level-2 详细对比

| 特性 | Level-1 | Level-2 |
|------|---------|---------|
| 输入端口数量 | 仅 1 个 (u) | 多个 |
| 输出端口数量 | 仅 1 个 (y) | 多个 |
| 端口维度 | 标量或固定向量 | 灵活配置 |
| 参数传递 | 通过 u 向量传递 | 对话框参数 |
| 可读性 | 较差 | 较好 |
| **推荐度** | ❌ 已过时 | ✅ MATLAB S-Function 首选 |

---

## 3. S-Function 回调方法详解

### 3.1 回调方法列表

| 回调方法 | 调用时机 | 功能 | 必须实现 |
|----------|----------|------|----------|
| `InitializeSizes` | 仿真开始前 | 配置端口、状态、采样时间 | ✅ 是 |
| `InitializeConditions` | 仿真开始时 | 设置初始状态值 | 可选 |
| `Derivatives` | 每个连续时间步 | 计算连续状态微分 | 若有连续状态 |
| `Update` | 每个离散时间步 | 更新离散状态 | 若有离散状态 |
| `Outputs` | 每个时间步 | 计算输出信号 | ✅ 是 |
| `Terminate` | 仿真结束时 | 清理资源 | 可选 |
| `CheckParameters` | 参数变化时 | 验证参数有效性 | 可选 |
| `ProcessParameters` | 参数变化时 | 处理参数变化 | 可选 |

### 3.2 InitializeSizes 详解

```matlab
function InitializeSizes(block)
  % 输入端口配置
  block.NumInputPorts = 2;
  block.InputPort(1).Dimensions = 3;   % 3维向量
  block.InputPort(1).DatatypeID = 0;   % double
  block.InputPort(1).DirectFeedthrough = true;

  block.InputPort(2).Dimensions = 1;   % 标量
  block.InputPort(2).DatatypeID = 0;

  % 输出端口配置
  block.NumOutputPorts = 1;
  block.OutputPort(1).Dimensions = 4;
  block.OutputPort(1).DatatypeID = 0;

  % 状态数量
  block.NumContStates = 0;   % 无连续状态
  block.NumDiscStates = 4;   % 4 个离散状态

  % 采样时间
  block.SampleTimes = [0.001 0];  % [Ts, Offset]

  % 对话框参数
  block.NumDialogPrms = 2;
  block.DialogPrmsTunable = {'Tunable', 'Nontunable'};
end
```

### 3.3 回调方法执行流程

```
仿真开始
  │
  ├── InitializeSizes()     ← 配置模块结构
  ├── InitializeConditions() ← 设置初始状态
  │
  ├── 每个时间步循环:
  │   │
  │   ├── Outputs()          ← 计算输出 (每个时间步)
  │   ├── Derivatives()      ← 计算连续微分 (连续模块)
  │   ├── Update()           ← 更新离散状态 (离散模块)
  │   │
  │   └── 下一个时间步 ──> 继续循环
  │
  └── Terminate()            ← 仿真结束，清理资源
```

---

## 4. Level-2 MATLAB S-Function 编写步骤

### 4.1 实例：自定义一阶滤波器

实现一个可配置截止频率的一阶低通滤波器：

```
传递函数: H(s) = 1 / (τs + 1)
离散化:   y(k) = a*y(k-1) + (1-a)*x(k)
          其中 a = exp(-Ts/τ)
```

### 4.2 完整代码

```matlab
% 文件: first_order_filter.m
function first_order_filter(block)
  setup(block);
end

%% 初始化
function setup(block)
  % 端口配置
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  block.InputPort(1).Dimensions  = 1;
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).DirectFeedthrough = true;
  block.InputPort(1).SamplingMode = 'Sample';

  block.OutputPort(1).Dimensions = 1;
  block.OutputPort(1).DatatypeID = 0;

  % 参数: 截止频率 (Hz)
  block.NumDialogPrms = 1;
  block.DialogPrmsTunable = {'Tunable'};

  % 采样时间
  block.SampleTimes = [0.001 0];  % 1kHz

  % 注册回调方法
  block.RegBlockMethod('InitializeConditions', @InitConditions);
  block.RegBlockMethod('Outputs', @Output);
  block.RegBlockMethod('Update', @Update);
end

%% 初始条件
function InitConditions(block)
  % 离散状态存储上一步输出
  block.Dwork(1).Data = 0;  % y(k-1) = 0
end

%% 输出计算
function Output(block)
  % 输出当前状态值
  block.OutputPort(1).Data = block.Dwork(1).Data;
end

%% 状态更新
function Update(block)
  % 参数
  fc = block.DialogPrms(1).Data;  % 截止频率
  Ts = 0.001;                      % 采样时间

  % 计算滤波系数
  tau = 1 / (2 * pi * fc);
  a = exp(-Ts / tau);

  % 输入
  x = block.InputPort(1).Data;

  % 一阶滤波
  y_prev = block.Dwork(1).Data;
  y = a * y_prev + (1 - a) * x;

  % 更新状态
  block.Dwork(1).Data = y;
end
```

### 4.3 使用方法

```
1. 将 first_order_filter.m 保存到 MATLAB 路径
2. 在 Simulink 中添加 S-Function 模块:
   Library Browser > User-Defined Functions > S-Function
3. 双击模块，设置 S-function name: first_order_filter
4. 设置 S-function parameters: 10  (截止频率 10Hz)
5. 连接信号，运行仿真
```

---

## 5. C MEX S-Function

### 5.1 为什么使用 C MEX

| 优势 | 说明 |
|------|------|
| 仿真速度 | 比 MATLAB S-Function 快 10~100 倍 |
| 代码生成 | 支持 Embedded Coder 代码生成 |
| 内存控制 | 精确控制内存分配 |
| 外部库 | 可调用任意 C/C++ 库 |
| 实时性 | 适合 HIL 和实时仿真 |

### 5.2 C MEX S-Function 基本结构

```c
/* 文件: first_order_filter.c */
#define S_FUNCTION_NAME  first_order_filter
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/* 参数宏定义 */
#define SAMPLE_TIME  0.001
#define NUM_PARAMS   1
#define FC_IDX       0

/* 初始化大小 */
static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    ssSetNumInputPorts(S, 1);
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    ssSetNumOutputPorts(S, 1);
    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumDiscStates(S, 1);
    ssSetNumSampleTimes(S, 1);
}

/* 初始化条件 */
static void mdlInitializeConditions(SimStruct *S) {
    real_T *x0 = ssGetRealDiscStates(S);
    x0[0] = 0.0;
}

/* 采样时间 */
static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

/* 输出 */
static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    real_T *x = ssGetRealDiscStates(S);
    y[0] = x[0];
}

/* 状态更新 */
#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *x  = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T fc = mxGetPr(ssGetSFcnParam(S, FC_IDX))[0];

    real_T tau = 1.0 / (2.0 * 3.14159265 * fc);
    real_T a = exp(-SAMPLE_TIME / tau);
    real_T u = (*uPtrs[0]);

    x[0] = a * x[0] + (1.0 - a) * u;
}

/* 终止 */
static void mdlTerminate(SimStruct *S) {
}

/* 必需的 S-Function 尾部 */
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
```

### 5.3 编译 C MEX S-Function

```matlab
% MATLAB 命令行编译
>> mex first_order_filter.c

% 指定编译器 (首次使用需配置)
>> mex -setup C

% 编译后生成:
%   first_order_filter.mexw64  (Windows)
%   first_order_filter.mexa64  (Linux)
```

---

## 6. S-Function Builder

S-Function Builder 提供图形化界面，无需手动编写完整的 C MEX 框架代码。

### 6.1 使用方法

```
1. 从 Library Browser 拖入 S-Function Builder 模块
2. 双击打开编辑器
3. 在各个标签页中填写:

   Initialization 标签:
     - 输入/输出端口配置
     - 状态数量
     - 采样时间

   Data Properties 标签:
     - 输入端口维度和数据类型
     - 输出端口维度和数据类型
     - 参数定义

   Outputs 标签: (核心)
     - 填写 C 代码计算输出
     y[0] = x[0];  // 直接写 C 代码

   Update 标签:
     - 填写状态更新的 C 代码
     x[0] = a * x[0] + (1.0-a) * u[0];

   Libraries 标签:
     - 包含头文件
     - 链接外部库

4. 点击 Build 编译
```

### 6.2 S-Function Builder vs 手写 C MEX

| 特性 | S-Function Builder | 手写 C MEX |
|------|--------------------|-----------|
| 开发速度 | 快 | 慢 |
| 灵活性 | 一般 | 完全控制 |
| 代码可读性 | 较好 | 需要熟悉框架 |
| 调试 | 支持断点 | 需要外部调试器 |
| 复杂逻辑 | 困难 | 容易 |
| 推荐场景 | 简单 C 代码集成 | 复杂算法/外部库集成 |

---

## 7. S-Function vs MATLAB Function 模块

### 7.1 MATLAB Function 模块

```
Library Browser > User-Defined Functions > MATLAB Function

特点:
  - 使用标准 MATLAB 语法
  - 支持代码生成 (Embedded Coder)
  - 自动推断数据类型
  - 不需要编写回调方法
  - 仿真速度中等
```

### 7.2 详细对比

| 特性 | S-Function (MATLAB) | S-Function (C MEX) | MATLAB Function |
|------|--------------------|--------------------|------------------|
| 语言 | MATLAB | C/C++ | MATLAB |
| 学习曲线 | 陡 | 最陡 | 平缓 |
| 仿真速度 | 慢 | **最快** | 中 |
| 代码生成 | ❌ | ✅ | ✅ |
| 连续状态 | ✅ | ✅ | ❌ (需用 Integrator) |
| 多采样率 | ✅ | ✅ | ❌ (继承) |
| 调试 | 简单 | 困难 | 简单 |
| 推荐场景 | 快速原型 | 生产代码 | **首选** |

### 7.3 选择决策树

```
需要自定义模块?
  │
  ├── 只需要数学运算和逻辑?
  │     └── ✅ 使用 MATLAB Function 模块 (首选)
  │
  ├── 需要自定义连续状态方程?
  │     └── ✅ 使用 S-Function 或 State-Space 模块
  │
  ├── 需要调用外部 C 库?
  │     └── ✅ 使用 C MEX S-Function 或 S-Function Builder
  │
  ├── 需要最快的仿真速度?
  │     └── ✅ 使用 C MEX S-Function
  │
  └── 需要代码生成?
        ├── C MEX S-Function ✅
        └── MATLAB Function ✅
```

---

## 8. 自定义气动模型 S-Function 实例

### 8.1 需求

实现一个非线性气动模型，输入为迎角 α、侧滑角 β、空速 V、舵面偏角 δ，输出为气动力和力矩。使用 S-Function 是因为需要内嵌复杂的气动公式和条件逻辑。

### 8.2 Level-2 MATLAB S-Function 实现

```matlab
% 文件: aero_model_sfcn.m
function aero_model_sfcn(block)
  setup(block);
end

function setup(block)
  % 输入: [alpha; beta; V; delta_e; delta_a; delta_r] (6x1)
  block.NumInputPorts = 1;
  block.InputPort(1).Dimensions = 6;
  block.InputPort(1).DatatypeID = 0;
  block.InputPort(1).DirectFeedthrough = true;

  % 输出: [Fx; Fy; Fz; Mx; My; Mz] (6x1)
  block.NumOutputPorts = 2;
  block.OutputPort(1).Dimensions = 3;  % 力 [Fx; Fy; Fz]
  block.OutputPort(2).Dimensions = 3;  % 力矩 [Mx; My; Mz]

  % 参数: [S; b; c_bar; rho] - 参考面积、展长、平均气动弦长、密度
  block.NumDialogPrms = 4;

  block.SampleTimes = [0 0];  % 连续

  block.RegBlockMethod('Outputs', @Output);
end

function Output(block)
  % 读取输入
  alpha = block.InputPort(1).Data(1);  % 迎角 (rad)
  beta  = block.InputPort(1).Data(2);  % 侧滑角 (rad)
  V     = block.InputPort(1).Data(3);  % 空速 (m/s)
  d_e   = block.InputPort(1).Data(4);  % 升降舵偏角 (rad)
  d_a   = block.InputPort(1).Data(5);  % 副翼偏角 (rad)
  d_r   = block.InputPort(1).Data(6);  % 方向舵偏角 (rad)

  % 读取参数
  S     = block.DialogPrms(1).Data;  % 参考面积
  b     = block.DialogPrms(2).Data;  % 展长
  c_bar = block.DialogPrms(3).Data;  % 平均气动弦长
  rho   = block.DialogPrms(4).Data;  % 空气密度

  % 动压
  q_bar = 0.5 * rho * V^2;

  % 气动系数 (简化模型)
  % 升力系数
  CL_alpha = 2*pi;  % 升力线斜率
  CL = CL_alpha * alpha + 0.5 * d_e;  % 含升降舵贡献

  % 阻力系数 (极曲线)
  CD0 = 0.02;       % 零升阻力
  AR = b^2 / S;     % 展弦比
  e = 0.8;          % Oswald 效率因子
  CD = CD0 + CL^2 / (pi * AR * e);

  % 侧力系数
  CY = -0.3 * beta + 0.2 * d_r;

  % 力矩系数
  Cl = -0.05 * beta + 0.1 * d_a;     % 滚转力矩
  Cm = -0.5 * alpha - 0.8 * d_e;     % 俯仰力矩
  Cn = 0.05 * beta - 0.08 * d_r;     % 偏航力矩

  % 计算力 (机体坐标系)
  Fx = -q_bar * S * CD;
  Fy =  q_bar * S * CY;
  Fz = -q_bar * S * CL;

  % 计算力矩
  Mx = q_bar * S * b * Cl;
  My = q_bar * S * c_bar * Cm;
  Mz = q_bar * S * b * Cn;

  % 输出
  block.OutputPort(1).Data = [Fx; Fy; Fz];
  block.OutputPort(2).Data = [Mx; My; Mz];
end
```

### 8.3 模块封装

为气动模型创建 Mask：

```
Mask Parameters:
  | Prompt         | Name     | Type  | Default     |
  |----------------|----------|-------|-------------|
  | 参考面积 S      | S        | Edit  | 0.5         |
  | 展长 b          | b        | Edit  | 2.0         |
  | 平均气动弦长 c   | c_bar    | Edit  | 0.25        |
  | 空气密度 rho     | rho      | Edit  | 1.225       |

Icon drawing:
  port_label('input', 1, '[α β V δe δa δr]');
  port_label('output', 1, 'Force');
  port_label('output', 2, 'Moment');
```

---

## 9. S-Function 调试

### 9.1 MATLAB S-Function 调试

```matlab
% 方法 1: 使用 disp/fprintf 输出调试信息
function Output(block)
  x = block.InputPort(1).Data;
  fprintf('Time: %.4f, Input: [%.3f, %.3f, %.3f]\n', ...
          block.CurrentTime, x(1), x(2), x(3));
  % ...
end

% 方法 2: 使用 keyboard 进入调试模式
function Output(block)
  if block.CurrentTime > 1.0 && block.CurrentTime < 1.002
    keyboard;  % 在 t=1s 时暂停进入调试
  end
end

% 方法 3: 使用 assignin 将数据传到工作区
function Output(block)
  assignin('base', 'debug_output', block.OutputPort(1).Data);
  assignin('base', 'debug_time', block.CurrentTime);
end
```

### 9.2 C MEX S-Function 调试

```c
/* 方法 1: mexPrintf 输出 */
static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    mexPrintf("Time: %f, Output: %f\n", ssGetT(S), y[0]);
}

/* 方法 2: 使用 mexEvalString 调用 MATLAB */
static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    char buf[256];
    sprintf(buf, "disp('Output: %f')", y[0]);
    mexEvalString(buf);
}

/* 方法 3: 外部调试器
   1. 编译时添加调试信息: mex -g file.c
   2. 附加到 MATLAB 进程
   3. 设置断点调试
*/
```

### 9.3 常见问题排查

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 编译失败 | 编译器未配置 | `mex -setup C` |
| 输出全为零 | 未在 Outputs 中赋值 | 检查 `block.OutputPort(1).Data` |
| 维度错误 | 端口维度设置不匹配 | 检查 `Dimensions` 和实际赋值 |
| 采样时间冲突 | 采样时间设置错误 | 检查 `SampleTimes` |
| 状态未更新 | 忘记注册 Update 回调 | `block.RegBlockMethod('Update', @Update)` |
| NaN 输出 | 除以零或未初始化 | 检查分母和初始条件 |

---

## 10. 性能对比

### 10.1 仿真速度测试

以一阶滤波器 (1000 步仿真) 为例：

| 实现方式 | 仿真时间 (s) | 相对速度 | 代码生成 |
|----------|-------------|----------|----------|
| MATLAB Function | 0.8 | 1x (基准) | ✅ |
| Level-2 MATLAB S-Function | 2.5 | 0.32x | ❌ |
| C MEX S-Function | 0.05 | 16x | ✅ |
| S-Function Builder (C) | 0.06 | 13x | ✅ |
| 内置 Transfer Fcn | 0.03 | 27x | ✅ |

### 10.2 选择建议总结

```
日常建模 (大多数情况):
  → MATLAB Function 模块 (简单、支持代码生成)

需要自定义状态方程:
  → Level-2 MATLAB S-Function (原型)
  → C MEX S-Function (生产)

需要调用外部 C 代码:
  → S-Function Builder (简单接口)
  → C MEX S-Function (复杂接口)

追求极致仿真速度:
  → C MEX S-Function 或 内置模块

需要代码生成到飞控:
  → MATLAB Function 或 C MEX S-Function
```

---

## 思考题

1. **MATLAB Function 模块和 Level-2 MATLAB S-Function 都可以用 MATLAB 代码实现自定义功能。它们在功能上有什么本质区别？在什么情况下必须使用 S-Function 而不能用 MATLAB Function？**

2. **在 C MEX S-Function 中，`mdlOutputs` 和 `mdlUpdate` 的调用顺序是什么？为什么这种顺序对模块的正确性至关重要？**

3. **S-Function Builder 生成的 C 代码和手写的 C MEX S-Function 代码在结构上有什么区别？如果需要优化仿真性能，应该修改 S-Function Builder 的哪个标签页？**

4. **在 UAV 飞控模型中，哪些模块适合用 S-Function 实现？请举出至少三个具体例子，并说明为什么内置模块不能满足需求。**

5. **如果要将一个 MATLAB S-Function 转换为 C MEX S-Function 以支持代码生成，需要做哪些关键修改？有哪些陷阱需要注意？**

<details>
<summary>参考答案</summary>

**1.** 本质区别：MATLAB Function 只能实现纯函数映射（输入→输出），没有自己的状态管理和采样时间控制，状态必须通过 Integrator 等外部模块实现。S-Function 可以管理自己的连续/离散状态、自定义采样时间、处理初始化和终止逻辑。必须使用 S-Function 的情况：(1) 需要多个不同的采样时间；(2) 需要在模块内部管理状态（DWork）；(3) 需要自定义初始化和终止行为；(4) 需要调用不支持代码生成的 MATLAB 函数。

**2.** 调用顺序：在每个时间步中，`mdlOutputs` 先于 `mdlUpdate` 调用。这确保了输出使用的是当前时间步的输入和上一步的状态值。如果顺序反过来，状态会先被更新，输出就会使用新状态而不是当前状态，导致输出信号比输入延迟了一个额外的时间步，破坏了模块的时间一致性。对于连续模块，`mdlDerivatives` 在 `mdlOutputs` 之后调用。

**3.** 结构区别：S-Function Builder 生成的代码包含自动生成的封装层（tlcg_sfun.c），将 Simulink API 调用封装在内部，用户代码集中在 Outputs 和 Update 函数中。手写 C MEX 直接使用 Simulink API。优化仿真性能应修改：(1) Outputs 标签页，减少不必要的计算；(2) 使用 `ssSetOptions` 设置 `SS_OPTION_CAN_BE_CALLED_CONDITIONALLY` 避免不必要的调用；(3) 使用 `mdlRTW` 将参数写入 RTW 文件避免运行时参数查找。

**4.** 适合 S-Function 的 UAV 模块：(1) 非线性气动模型：内置 Lookup Table 难以表达条件逻辑（如失速区域的突变）和多段气动模型；(2) 自适应控制器：需要维护历史数据和自适应参数的状态，MATLAB Function 的状态管理受限；(3) 传感器融合算法（如扩展卡尔曼滤波）：需要矩阵运算和状态预测/更新的复杂流程，且对仿真速度有要求（推荐 C MEX）。

**5.** 关键修改：(1) 将 MATLAB 代码逐行翻译为 C 代码，注意 C 语言的数组从 0 开始索引；(2) 将 MATLAB 的工作区变量改为 `ssGetSFcnParam` 参数读取；(3) 将 `disp`/`fprintf` 改为 `mexPrintf`；(4) 手动管理内存分配（`mxMalloc`/`mxFree`）；(5) 处理 MATLAB 的矩阵运算改用 BLAS 或手动循环。陷阱：(1) MATLAB 索引从 1 开始，C 从 0 开始，容易 off-by-one 错误；(2) MATLAB 的动态内存管理在 C 中需要手动处理；(3) 浮点精度可能有细微差异导致仿真结果不完全一致。

</details>
