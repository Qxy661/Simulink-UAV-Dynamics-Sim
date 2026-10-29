# 扩展卡尔曼滤波 (EKF) 与 UAV 状态估计

> 预计阅读：25 分钟 | 前置知识：线性代数、概率论与随机过程、IMU/GPS 传感器建模基础

---

## 1. 卡尔曼滤波的直觉

### 1.1 核心思想

卡尔曼滤波 (Kalman Filter, KF) 解决的核心问题：

> 给定一个不完美的**预测模型**和一组不完美的**测量数据**，如何最优地估计系统状态？

```
                    ┌─────────────┐
   模型预测 ───────►│             │──────► 最优估计
                    │  卡尔曼滤波 │         (均值 + 方差)
   传感器测量 ────►│             │
                    └─────────────┘

   不确定性大 ←─── 加权平均 ───► 不确定性小
   (信任度低)                    (信任度高)
```

**直觉类比——天气预报：**
- 气象模型预测明天 25°C（模型不确定度 ±3°C）
- 温度计实测 27°C（测量不确定度 ±1°C）
- 卡尔曼滤波的最优估计：约 26.7°C（更信任测量值）

### 1.2 两个基本步骤

```
┌───────────────────────────────────────────────────────────┐
│                   卡尔曼滤波迭代循环                        │
│                                                           │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐           │
│   │  预 测   │───►│  更 新   │───►│  输 出   │           │
│   │ Predict  │    │  Update  │    │  Output  │           │
│   └──────────┘    └──────────┘    └──────────┘           │
│        │              ▲                                    │
│        │              │                                    │
│        │         测量数据                                  │
│        │                                                  │
│   使用模型状态传播     使用传感器测量修正                    │
└───────────────────────────────────────────────────────────┘
```

---

## 2. 线性卡尔曼滤波方程

### 2.1 系统模型

**状态方程 (离散时间)：**

$$
\mathbf{x}_k = \mathbf{F}_{k-1} \mathbf{x}_{k-1} + \mathbf{B}_{k-1} \mathbf{u}_{k-1} + \mathbf{w}_{k-1}
$$

**测量方程：**

$$
\mathbf{z}_k = \mathbf{H}_k \mathbf{x}_k + \mathbf{v}_k
$$

其中：
- $\mathbf{x}_k$：$n \times 1$ 状态向量
- $\mathbf{F}_k$：$n \times n$ 状态转移矩阵
- $\mathbf{u}_k$：控制输入
- $\mathbf{w}_k \sim \mathcal{N}(0, \mathbf{Q}_k)$：过程噪声
- $\mathbf{z}_k$：$m \times 1$ 测量向量
- $\mathbf{H}_k$：$m \times n$ 测量矩阵
- $\mathbf{v}_k \sim \mathcal{N}(0, \mathbf{R}_k)$：测量噪声

### 2.2 预测步 (Prediction)

$$
\hat{\mathbf{x}}_k^- = \mathbf{F}_{k-1} \hat{\mathbf{x}}_{k-1}^+ + \mathbf{B}_{k-1} \mathbf{u}_{k-1}
$$

$$
\mathbf{P}_k^- = \mathbf{F}_{k-1} \mathbf{P}_{k-1}^+ \mathbf{F}_{k-1}^T + \mathbf{Q}_{k-1}
$$

- $\hat{\mathbf{x}}_k^-$：先验估计 (prior estimate)
- $\mathbf{P}_k^-$：先验协方差 (不确定性增大)

### 2.3 更新步 (Update)

**卡尔曼增益：**

$$
\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R}_k)^{-1}
$$

**状态更新：**

$$
\hat{\mathbf{x}}_k^+ = \hat{\mathbf{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \hat{\mathbf{x}}_k^-)
$$

**协方差更新：**

$$
\mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^-
$$

### 2.4 卡尔曼增益的含义

```
K → 0:  完全信任模型预测 (测量噪声 R 很大)
K → 1:  完全信任测量   (模型不确定性 P 很大)

K = P⁻Hᵀ(HP⁻Hᵀ + R)⁻¹

                 模型不确定性
        K ∝ ─────────────────────────
            模型不确定性 + 测量不确定性
```

---

## 3. 扩展卡尔曼滤波 (EKF)

### 3.1 为什么需要 EKF？

线性 KF 要求系统是**线性**的，但 UAV 的动力学和测量方程本质上是**非线性**的：
- 姿态用四元数/欧拉角表示，动力学包含三角函数
- 加速度计测量涉及旋转矩阵
- GPS 测量在不同坐标系间转换

EKF 的核心思路：在当前估计点处对非线性函数进行**一阶泰勒展开**（线性化）。

### 3.2 EKF 方程

**非线性系统模型：**

$$
\mathbf{x}_k = f(\mathbf{x}_{k-1}, \mathbf{u}_{k-1}) + \mathbf{w}_{k-1}
$$

$$
\mathbf{z}_k = h(\mathbf{x}_k) + \mathbf{v}_k
$$

**预测步：**

$$
\hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1}^+, \mathbf{u}_{k-1})
$$

$$
\mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1}^+ \mathbf{F}_k^T + \mathbf{Q}_{k-1}
$$

其中 **雅可比矩阵**：

$$
\mathbf{F}_k = \left. \frac{\partial f}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k-1}^+}
$$

**更新步：**

$$
\mathbf{H}_k = \left. \frac{\partial h}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_k^-}
$$

$$
\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R}_k)^{-1}
$$

$$
\hat{\mathbf{x}}_k^+ = \hat{\mathbf{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - h(\hat{\mathbf{x}}_k^-))
$$

$$
\mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^-
$$

---

## 4. 四旋翼 EKF 状态定义

### 4.1 状态向量

采用 15 维状态向量（误差状态形式）：

$$
\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \boldsymbol{\theta} \\ \mathbf{b}_a \\ \mathbf{b}_g \end{bmatrix}_{15 \times 1}
$$

| 子状态 | 符号 | 维度 | 说明 |
|--------|------|------|------|
| 位置 | $\mathbf{p} = [p_x, p_y, p_z]^T$ | 3 | NED 或 ENU 坐标系 |
| 速度 | $\mathbf{v} = [v_x, v_y, v_z]^T$ | 3 | 惯性系下的速度 |
| 姿态误差 | $\boldsymbol{\theta} = [\theta_x, \theta_y, \theta_z]^T$ | 3 | 误差四元数的小角度表示 |
| 加速度计零偏 | $\mathbf{b}_a = [b_{ax}, b_{ay}, b_{az}]^T$ | 3 | 估计并补偿 |
| 陀螺仪零偏 | $\mathbf{b}_g = [b_{gx}, b_{gy}, b_{gz}]^T$ | 3 | 估计并补偿 |

### 4.2 误差状态 EKF 与全状态 EKF

**全状态 EKF** 直接对四元数 $\mathbf{q}$ 进行滤波，需要处理四元数的单位约束（归一化），协方差矩阵奇异等问题。

**误差状态 EKF (ES-EKF)** 将状态分为**名义状态**和**误差状态**：

$$
\mathbf{q} = \mathbf{q}_{nominal} \otimes \delta\mathbf{q}(\boldsymbol{\theta})
$$

其中 $\delta\mathbf{q}(\boldsymbol{\theta}) \approx [1, \frac{\theta_x}{2}, \frac{\theta_y}{2}, \frac{\theta_z}{2}]^T$。

优势：误差状态维度小（3 vs 4），无约束问题，线性化更精确。

---

## 5. IMU 传播步 (Propagation)

### 5.1 连续时间动力学

IMU 以高频率（200–1000 Hz）提供角速度和加速度测量，用于**传播**状态估计。

**名义状态传播：**

$$
\dot{\mathbf{p}} = \mathbf{v}
$$

$$
\dot{\mathbf{v}} = \mathbf{R}(\mathbf{a}_m - \mathbf{b}_a) + \mathbf{g}
$$

$$
\dot{\mathbf{R}} = \mathbf{R} [\boldsymbol{\omega}_m - \mathbf{b}_g]_\times
$$

$$
\dot{\mathbf{b}}_a = 0, \quad \dot{\mathbf{b}}_g = 0
$$

其中 $[\cdot]_\times$ 为反对称矩阵，$\mathbf{a}_m$ 和 $\boldsymbol{\omega}_m$ 为 IMU 测量值。

### 5.2 离散化

使用前向欧拉法（采样间隔 $\Delta t$）：

$$
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2}(\mathbf{R}_k(\mathbf{a}_m - \mathbf{b}_{a,k}) + \mathbf{g})\Delta t^2
$$

$$
\mathbf{v}_{k+1} = \mathbf{v}_k + (\mathbf{R}_k(\mathbf{a}_m - \mathbf{b}_{a,k}) + \mathbf{g})\Delta t
$$

$$
\mathbf{R}_{k+1} = \mathbf{R}_k \exp([\boldsymbol{\omega}_m - \mathbf{b}_{g,k}]_\times \Delta t)
$$

### 5.3 误差状态传播的雅可比矩阵

$$
\mathbf{F} = \begin{bmatrix}
\mathbf{I} & \mathbf{I}\Delta t & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I} & -\mathbf{R}_k[\mathbf{a}_m - \mathbf{b}_a]_\times \Delta t & -\mathbf{R}_k \Delta t & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} & -\Delta t \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
$$

### 5.4 过程噪声协方差 Q

$$
\mathbf{Q} = \begin{bmatrix}
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \sigma_{a,wn}^2 \Delta t^2 \mathbf{I} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \sigma_{g,wn}^2 \Delta t^2 \mathbf{I} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \sigma_{a,bi}^2 \Delta t \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \sigma_{g,bi}^2 \Delta t \mathbf{I}
\end{bmatrix}
```

---

## 6. 测量更新步

### 6.1 GPS 更新

GPS 提供位置和速度测量：

$$
\mathbf{z}_{GPS} = \begin{bmatrix} \mathbf{p}_{GPS} \\ \mathbf{v}_{GPS} \end{bmatrix}, \quad
\mathbf{H}_{GPS} = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{I}_3 & \mathbf{0} & \mathbf{0} & \mathbf{0} \end{bmatrix}
$$

$$
\mathbf{R}_{GPS} = \begin{bmatrix} \sigma_{p,GPS}^2 \mathbf{I}_3 & \mathbf{0} \\ \mathbf{0} & \sigma_{v,GPS}^2 \mathbf{I}_3 \end{bmatrix}
$$

### 6.2 气压计更新

气压计提供高度（z 方向位置）：

$$
z_{baro} = p_z + v_{baro}, \quad \mathbf{H}_{baro} = \begin{bmatrix} 0 & 0 & 1 & \cdots & 0 \end{bmatrix}
$$

### 6.3 磁力计更新

磁力计提供航向角 $\psi$：

$$
z_{mag} = \psi = \text{atan2}(m_y, m_x) + v_\psi
$$

$\mathbf{H}_{mag}$ 需要对 $\psi$ 关于误差状态中的姿态误差角求偏导。

### 6.4 创新序列与一致性检验

**创新序列 (Innovation/Residual)：**

$$
\boldsymbol{\nu}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_k^-)
$$

**归一化创新平方 (NIS)：**

$$
\text{NIS}_k = \boldsymbol{\nu}_k^T \mathbf{S}_k^{-1} \boldsymbol{\nu}_k, \quad \mathbf{S}_k = \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R}_k
$$

NIS 服从 $\chi^2$ 分布，可用于检测滤波器一致性。

---

## 7. Q 与 R 矩阵调参

### 7.1 Q 矩阵 (过程噪声)

Q 矩阵反映**模型的不确定性**：

| 参数 | 调大效果 | 调小效果 |
|------|---------|---------|
| 位置过程噪声 | 滤波器更快响应测量 | 更信任模型 |
| 速度过程噪声 | 同上 | 同上 |
| 姿态过程噪声 | 更信任 IMU/磁力计 | 更信任动力学模型 |
| 零偏过程噪声 | 零偏估计更快变化 | 零偏估计更平稳 |

### 7.2 R 矩阵 (测量噪声)

R 矩阵反映**传感器的不确定性**：

| 参数 | 调大效果 | 调小效果 |
|------|---------|---------|
| GPS 位置 R | 更少信任 GPS | 更信任 GPS |
| GPS 速度 R | 同上 | 同上 |
| 气压计 R | 更少信任气压计 | 更信任气压计 |

### 7.3 调参策略

```
┌─────────────────────────────────────────────┐
│            EKF 调参流程                       │
│                                              │
│  1. 从传感器 datasheet 初始化 R               │
│     (Allan 方差 → 噪声参数)                   │
│                                              │
│  2. 从 IMU datasheet 初始化 Q                │
│     (白噪声 + 零偏不稳定性)                    │
│                                              │
│  3. 仿真测试，观察：                           │
│     - NIS 统计是否符合 χ² 分布                 │
│     - 创新序列是否零均值、白噪声                │
│     - 估计协方差是否合理                       │
│                                              │
│  4. 微调：                                    │
│     - 状态估计抖动大 → 增大对应 R              │
│     - 状态估计滞后大 → 减小对应 R              │
│     - 零偏估计不收敛 → 增大 Q_bi              │
│     - 滤波器发散 → 检查模型和数值问题           │
└─────────────────────────────────────────────┘
```

---

## 8. 滤波器对比

| 特性 | KF | EKF | UKF | Particle Filter |
|------|----|----|-----|----------------|
| **非线性处理** | 无 | 一阶线性化 | 确定性采样 | 蒙特卡洛采样 |
| **精度** | 精确 (线性系统) | 一阶近似 | 二阶精度 | 任意精度 |
| **计算量** | 最低 | 低 | 中等 | 高 |
| **实现难度** | 简单 | 中等 | 中等 | 较高 |
| **适用场景** | 线性系统 | 弱非线性 | 中等非线性 | 强非线性/多模态 |
| **UAV 应用** | 不适用 | 最常用 | 可选 | 视觉/SLAM |
| **状态维度** | 不限 | < 30 | < 30 | 受限于粒子数 |
| **雅可比矩阵** | 不需要 | 需要推导 | 不需要 | 不需要 |

**在 UAV 领域，EKF 是最广泛使用的状态估计方法**，因为 IMU 传播步近似线性（高采样率），计算效率高。

---

## 9. Simulink EKF 实现

### 9.1 整体架构

```
┌────────────────────────────────────────────────────────────┐
│                    EKF Simulink 模型架构                     │
│                                                            │
│  ┌──────────┐     ┌───────────────┐     ┌──────────┐      │
│  │ IMU 数据 │────►│ EKF 传播步    │────►│ 状态预测 │      │
│  │ 200 Hz   │     │ (IMU Propagate)│     │          │      │
│  └──────────┘     └───────────────┘     └────┬─────┘      │
│                                              │             │
│                                              ▼             │
│                   ┌───────────────┐     ┌──────────┐      │
│  ┌──────────┐    │ EKF 更新步    │◄────│ 状态缓冲 │      │
│  │ GPS 5Hz  │───►│ (GPS Update)  │     │ (延迟补偿)│     │
│  └──────────┘    └───────────────┘     └──────────┘      │
│                                                            │
│  ┌──────────┐    ┌───────────────┐                        │
│  │气压计25Hz│───►│ EKF 更新步    │                        │
│  └──────────┘    │(Baro Update)  │                        │
│                  └───────────────┘                        │
│                                                            │
│  ┌──────────┐    ┌───────────────┐                        │
│  │磁力计50Hz│───►│ EKF 更新步    │                        │
│  └──────────┘    │(Mag Update)   │                        │
│                  └───────────────┘                        │
└────────────────────────────────────────────────────────────┘
```

### 9.2 MATLAB 函数实现：传播步

```matlab
function [x, P] = ekf_propagate(x, P, imu, dt, Q)
% EKF 传播步 - 使用IMU测量传播状态
% x: 15x1 状态向量 [p;v;theta;ba;bg]
% P: 15x15 协方差矩阵
% imu: 结构体 .accel (3x1), .gyro (3x1)
% dt: 时间步长
% Q: 15x15 过程噪声协方差

    % 解包状态
    p = x(1:3);
    v = x(4:6);
    theta = x(7:9);
    ba = x(10:12);
    bg = x(13:15);

    % 旋转矩阵 (误差四元数 -> 旋转矩阵)
    R = euler2rot(theta);

    % 加速度和角速度补偿零偏
    a_corrected = imu.accel - ba;
    w_corrected = imu.gyro - bg;

    % 名义状态传播
    a_NED = R * a_corrected + [0; 0; 9.81];
    p_new = p + v * dt + 0.5 * a_NED * dt^2;
    v_new = v + a_NED * dt;
    theta_new = theta + w_corrected * dt;  % 小角度近似

    % 更新状态
    x(1:3) = p_new;
    x(4:6) = v_new;
    x(7:9) = theta_new;
    % ba, bg 保持不变 (随机游走模型)

    % 计算雅可比矩阵 F
    F = eye(15);
    F(1:3, 4:6) = eye(3) * dt;
    F(4:6, 7:9) = -R * skew(a_corrected) * dt;
    F(4:6, 10:12) = -R * dt;
    F(7:9, 13:15) = -eye(3) * dt;

    % 协方差传播
    P = F * P * F' + Q;
end

function S = skew(v)
% 3x1 向量 -> 3x3 反对称矩阵
    S = [  0   -v(3)  v(2);
          v(3)   0   -v(1);
         -v(2)  v(1)   0  ];
end
```

### 9.3 MATLAB 函数实现：更新步

```matlab
function [x, P, innovation, NIS] = ekf_update_gps(x, P, z_gps, R_gps)
% EKF GPS更新步
% z_gps: 6x1 [p_GPS; v_GPS]
% R_gps: 6x6 测量噪声协方差

    % 测量矩阵 H (6x15)
    H = zeros(6, 15);
    H(1:3, 1:3) = eye(3);  % 位置
    H(4:6, 4:6) = eye(3);  % 速度

    % 预测测量
    z_pred = H * x;

    % 创新序列
    innovation = z_gps - z_pred;

    % 创新协方差
    S = H * P * H' + R_gps;

    % NIS (归一化创新平方)
    NIS = innovation' * (S \ innovation);

    % 卡尔曼增益
    K = P * H' / S;

    % 状态更新
    x = x + K * innovation;

    % 协方差更新 (Joseph形式，数值更稳定)
    IKH = eye(15) - K * H;
    P = IKH * P * IKH' + K * R_gps * K';
end
```

### 9.4 使用 MATLAB System Object 封装

```matlab
classdef UAV_EKF < matlab.System
    properties
        % 噪声参数
        accel_noise = 0.01;      % m/s^2
        gyro_noise = 0.001;      % rad/s
        accel_bias_noise = 0.0001;
        gyro_bias_noise = 0.00001;

        % GPS参数
        gps_pos_noise = 1.5;     % m
        gps_vel_noise = 0.1;     % m/s
    end

    properties (DiscreteState)
        x  % 状态向量 15x1
        P  % 协方差矩阵 15x15
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.x = zeros(15, 1);
            obj.P = eye(15) * 0.1;
            % 初始协方差设置
            obj.P(1:3,1:3) = eye(3) * 10;     % 位置不确定度 10m
            obj.P(4:6,4:6) = eye(3) * 1;      % 速度不确定度 1m/s
            obj.P(7:9,7:9) = eye(3) * 0.1;    % 姿态不确定度 0.1rad
            obj.P(10:12,10:12) = eye(3) * 0.1;
            obj.P(13:15,13:15) = eye(3) * 0.01;
        end

        function [x_out, P_out] = stepImpl(obj, imu_accel, imu_gyro, ...
                                           gps_pos, gps_vel, gps_valid, dt)
            % 传播步 (每次IMU测量触发)
            Q = obj.compute_Q(dt);
            [obj.x, obj.P] = ekf_propagate(obj.x, obj.P, ...
                struct('accel', imu_accel, 'gyro', imu_gyro), dt, Q);

            % GPS更新步 (仅当GPS有效时)
            if gps_valid
                z_gps = [gps_pos; gps_vel];
                R_gps = diag([ones(3,1)*obj.gps_pos_noise^2; ...
                              ones(3,1)*obj.gps_vel_noise^2]);
                [obj.x, obj.P, ~, ~] = ekf_update_gps(obj.x, obj.P, ...
                                                       z_gps, R_gps);
            end

            x_out = obj.x;
            P_out = obj.P;
        end
    end
end
```

---

## 10. EKF 常见问题与解决方案

### 10.1 发散问题

| 症状 | 可能原因 | 解决方案 |
|------|---------|---------|
| 状态估计爆炸 | 模型错误 | 检查坐标系/单位 |
| 协方差增长无界 | Q 太大 | 检查噪声参数 |
| 协方差趋近零 | R 太小或数值问题 | 使用 Joseph 形式更新 |
| 创新序列持续偏大 | 模型与测量不一致 | 检查传感器校准 |
| 估计滞后 | R 太大 | 减小 R |

### 10.2 数值稳定性

- 使用 **Joseph 形式** 的协方差更新代替简单形式
- 定期检查协方差矩阵的正定性
- 对四元数及时归一化
- 使用**平方根形式** EKF 提高数值稳定性

---

## 思考题

1. **为什么在 UAV 状态估计中通常选择 EKF 而非 UKF？在什么情况下 UKF 更有优势？**

<details><summary>参考答案</summary>
在 UAV 状态估计中，IMU 以高频率（200+ Hz）传播状态，每步的时间间隔很短，此时非线性程度较低，EKF 的一阶线性化近似已经足够精确。此外 EKF 计算量小，适合实时嵌入式系统。但在以下情况下 UKF 更有优势：(1) 传感器更新间隔较大，非线性积累显著；(2) 使用视觉等高度非线性的传感器；(3) 状态维度较低（< 20），UKF 的 sigma 点采样不会带来太大的计算负担；(4) 需要避免雅可比矩阵的推导和实现。
</details>

2. **误差状态 EKF 与全状态 EKF 在四元数处理上有什么根本区别？为什么误差状态 EKF 更优？**

<details><summary>参考答案</summary>
全状态 EKF 将四元数 $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$ 直接作为状态，协方差矩阵是 4×4 的。问题在于四元数必须满足单位约束 $\|\mathbf{q}\|=1$，但 KF 不会保证这个约束，因此需要每次更新后强制归一化，这会引入偏差。误差状态 EKF 将姿态拆分为名义四元数（由模型传播，保持归一化）和 3 维误差角 $\boldsymbol{\theta}$（由 KF 更新），避免了约束问题，且雅可比矩阵维度更小（3 vs 4），计算更高效，线性化精度也更高。
</details>

3. **在 EKF 中，如果 GPS 的位置测量噪声标准差设为 1m，但实际 GPS 的精度只有 5m，会发生什么？反过来呢？**

<details><summary>参考答案</summary>
如果 R 设为 1m² 但实际噪声为 5m，滤波器会**过度信任 GPS**，导致：(1) 状态估计跟随 GPS 噪声剧烈抖动；(2) 协方差快速收缩，过度自信；(3) 速度估计也会被 GPS 噪声污染。如果 R 设为 25m² 但实际噪声只有 1m，滤波器会**不够信任 GPS**，导致：(1) 状态估计更依赖 IMU 积分，缓慢漂移；(2) 协方差收缩过慢，不确定性维持在高水平；(3) GPS 信息利用不充分，估计精度低于应有水平。
</details>

4. **NIS (归一化创新平方) 检验的原理是什么？NIS 值始终很大说明什么问题？**

<details><summary>参考答案</summary>
NIS = $\boldsymbol{\nu}^T \mathbf{S}^{-1} \boldsymbol{\nu}$，其中 $\boldsymbol{\nu}$ 是创新序列，$\mathbf{S}$ 是创新协方差。在滤波器工作正常时，NIS 服从自由度为 m（测量维度）的 $\chi^2$ 分布。对于 6 维 GPS 测量，NIS 的 95% 置信区间约为 [1.24, 14.45]。如果 NIS 始终大于上限，说明创新比预期大，可能原因包括：(1) 传感器测量存在未建模的系统误差；(2) Q 或 R 矩阵参数设置不当；(3) 模型与实际系统不匹配；(4) 传感器存在故障或未正确校准。
</details>

5. **请解释为什么 Joseph 形式的协方差更新比简单形式 $(\mathbf{I} - \mathbf{KH})\mathbf{P}^-$ 数值上更稳定。**

<details><summary>参考答案</summary>
简单形式 $\mathbf{P}^+ = (\mathbf{I} - \mathbf{KH})\mathbf{P}^-$ 在理论上等价，但在数值计算中存在问题：当 K 很小时，$\mathbf{I} - \mathbf{KH}$ 接近单位阵但不精确，舍入误差可能导致 $\mathbf{P}^+$ 丢失正定性或对称性。Joseph 形式 $\mathbf{P}^+ = (\mathbf{I} - \mathbf{KH})\mathbf{P}^-(\mathbf{I} - \mathbf{KH})^T + \mathbf{KRK}^T$ 保证了：(1) 结果始终是对称的（$ABA^T$ 形式保证对称性）；(2) $\mathbf{KRK}^T$ 项保证 $\mathbf{P}^+$ 不会比 $\mathbf{R}$ 对应的部分更小（正定性）；(3) 即使数值精度有限，结果也是协方差矩阵的有效上界。
</details>

---

> **参考资料：**
> - Fonyuy45 — EKF for UAV state estimation implementations
> - aralab-unr/HSMC-EKF-for-Quadrotor-UAVs
> - Solà, *Quaternion kinematics for the error-state Kalman filter*, 2017
> - Sola, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508
> - Trawny & Roumeliotis, *Indirect Kalman Filter for 3D Attitude Estimation*
