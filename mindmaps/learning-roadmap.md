# 学习路线思维导图

> 本文档展示 Simulink 无人机动力学仿真项目的完整学习路径，覆盖 10 个章节。
> 颜色编码：**绿色 = 基础** | **蓝色 = 核心** | **橙色 = 高级** | **红色 = 前沿**

---

## 全局学习路线图

```mermaid
mindmap
  root((Simulink<br>无人机动力学仿真<br>学习路线))
    🟢 基础阶段
      **Ch1 项目导论**
        仿真意义与应用场景
        Simulink 仿真流程概览
        多旋翼 vs 固定翼对比
        项目目标与路线图
      **Ch2 MATLAB/Simulink 基础**
        Simulink 建模环境
        模块库与信号流
        求解器选择策略
        数据导入导出
      **Ch3 坐标系与变换**
        机体坐标系 Body Frame
        地理坐标系 NED/ENU
        欧拉角与旋转矩阵
        四元数表示与转换
    🔵 核心阶段
      **Ch4 刚体动力学建模**
        牛顿-欧拉方程
        质量与惯性矩阵
        6-DOF 运动方程
        Simulink 6-DOF 模块
      **Ch5 气动力与旋翼模型**
        螺旋桨叶素理论 BEMT
        气动力/力矩系数
        旋翼挥舞动力学
        地面效应与涡环状态
      **Ch6 电机与电池模型**
        无刷电机数学模型
        ESC 电调建模
        电池放电特性
        电机-旋翼耦合
    🟠 高级阶段
      **Ch7 飞行控制算法**
        PID 串级控制
        LQR 线性二次调节器
        MPC 模型预测控制
        SMC 滑模控制
      **Ch8 传感器与状态估计**
        IMU / GPS / 气压计建模
        扩展卡尔曼滤波 EKF
        互补滤波器
        视觉里程计 VIO
    🔴 前沿阶段
      **Ch9 高级仿真与验证**
        蒙特卡洛仿真
        硬件在环 HIL 测试
        FlightGear 三维可视化
        与 PX4/ArduPilot 联调
      **Ch10 前沿专题**
        强化学习飞行控制
        数字孪生 Digital Twin
        多机协同仿真
        城市空中交通 UAM
```

---

## 分阶段详细路线

### 第一阶段：基础入门（第 1-3 章）

```mermaid
mindmap
  root((基础阶段))
    仿真基础
      Simulink 界面操作
      模块连接与信号类型
      模型层次化设计
      子系统 Subsystem 封装
    数学基础
      向量与矩阵运算
      微分方程数值解
      坐标变换数学推导
      四元数代数
    无人机基础
      多旋翼构型 X/I/Y
      自由度与约束分析
      飞行模式定义
      常见飞控硬件概览
```

### 第二阶段：核心建模（第 4-6 章）

```mermaid
mindmap
  root((核心建模))
    刚体动力学
      平移运动方程
      旋转运动方程
      惯性张量计算
      6-DOF Simulink 实现
    气动力学
      叶素动量理论
      拉力系数 CT
      扭矩系数 CQ
      入流比与前进比
      非线性气动效应
    推进系统
      KV 值与电机选型
      螺旋桨匹配
      电池内阻模型
      推力-功率曲线
```

### 第三阶段：控制与估计（第 7-8 章）

```mermaid
mindmap
  root((控制与估计))
    经典控制
      PID 参数整定
      串级控制结构
      姿态环 + 角速率环
      高度环 + 速度环
    最优控制
      LQR 权重矩阵设计
      MPC 约束优化
      线性化与工作点
    状态估计
      传感器噪声建模
      EKF 预测-更新流程
      多传感器融合
      可观测性分析
```

### 第四阶段：高级应用（第 9-10 章）

```mermaid
mindmap
  root((高级应用))
    仿真验证
      蒙特卡洛参数扰动
      故障注入测试
      HIL 实时仿真
      Software-in-the-Loop
    前沿技术
      PPO/SAC 强化学习
      Sim-to-Real 迁移
      数字孪生架构
      多智能体协同
      eVTOL 城市空运
```

---

## 学习时间建议

| 阶段 | 章节 | 建议时长 | 前置要求 |
|------|------|----------|----------|
| 基础 | Ch1-3 | 2-3 周 | MATLAB 入门 |
| 核心 | Ch4-6 | 3-4 周 | 基础阶段完成 |
| 高级 | Ch7-8 | 3-4 周 | 核心阶段完成 |
| 前沿 | Ch9-10 | 2-3 周 | 高级阶段完成 |

> **总计约 10-14 周**，每周投入 10-15 小时即可完成全部学习。

---

## 技能树依赖关系

```mermaid
graph TD
    A[Ch1 项目导论] --> B[Ch2 MATLAB/Simulink 基础]
    A --> C[Ch3 坐标系与变换]
    B --> D[Ch4 刚体动力学建模]
    C --> D
    D --> E[Ch5 气动力与旋翼模型]
    D --> F[Ch6 电机与电池模型]
    E --> G[Ch7 飞行控制算法]
    F --> G
    D --> H[Ch8 传感器与状态估计]
    G --> I[Ch9 高级仿真与验证]
    H --> I
    I --> J[Ch10 前沿专题]
    G --> J
    H --> J

    style A fill:#4CAF50,color:#fff
    style B fill:#4CAF50,color:#fff
    style C fill:#4CAF50,color:#fff
    style D fill:#2196F3,color:#fff
    style E fill:#2196F3,color:#fff
    style F fill:#2196F3,color:#fff
    style G fill:#FF9800,color:#fff
    style H fill:#FF9800,color:#fff
    style I fill:#f44336,color:#fff
    style J fill:#f44336,color:#fff
```
