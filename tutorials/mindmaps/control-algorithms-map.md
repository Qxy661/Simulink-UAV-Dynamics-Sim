# 控制算法全景图

> 本文档以思维导图形式全面梳理无人机飞行控制算法体系，从经典 PID 到前沿强化学习，覆盖五大控制范式。

---

## 控制算法全景总览

```mermaid
mindmap
  root((无人机<br>控制算法))
    PID 族
      增量式 PID
      位置式 PID
      串级 PID
      抗积分饱和
      微分滤波
      增益调度 Gain Scheduling
    最优控制
      LQR 线性二次调节器
      LQG LQR+卡尔曼
      MPC 模型预测控制
      动态规划 DP
      时间最优控制
    鲁棒控制
      SMC 滑模控制
      H-infinity 控制
      自抗扰控制 ADRC
      鲁棒 MPC
      不确定性建模
    自适应控制
      模型参考自适应 MRAC
      增益调度自适应
      L1 自适应控制
      自适应反步法
      参数在线辨识
    智能控制
      强化学习 RL
      深度神经网络 DNN
      模糊控制 Fuzzy
      遗传算法优化 GA
      神经网络自适应
```

---

## 分支一：PID 族

```mermaid
mindmap
  root((PID 控制族))
    基本 PID
      比例项 P
        快速响应
        减小稳态误差
        过大导致振荡
      积分项 I
        消除稳态误差
        积分饱和问题
        响应变慢
      微分项 D
        抑制超调
        改善动态特性
        对噪声敏感
    PID 变体
      增量式 PID
        输出增量计算
        无积分累积
        适合执行器
      位置式 PID
        直接输出位置
        需要积分限幅
      非线性 PID
        误差变增益
        分段 PID
    串级控制
      姿态环-角速率环
        外环: 姿态角控制
        内环: 角速率控制
        带宽分离原则
      高度环-速度环
        外环: 高度控制
        内环: 速度控制
      位置环-速度环
        外环: XY 位置
        内环: XY 速度
    工程技巧
      抗积分饱和 Anti-Windup
      微分先行 PID
      不完全微分
      前馈补偿 Feedforward
      增益调度 Gain Scheduling
    参数整定
      Ziegler-Nichols 法
      Cohen-Coon 法
      继电反馈法
      频率响应法
      优化搜索法
```

---

## 分支二：最优控制

```mermaid
mindmap
  root((最优控制))
    LQR 线性二次调节器
      代价函数 J
        状态权重矩阵 Q
        控制权重矩阵 R
        Q/R 调参策略
      Riccati 方程求解
      反馈增益 K
      闭环极点配置
      连续 LQR
      离散 DLQR
    LQG
      LQR + Kalman Filter
      分离定理
      最优状态估计
      随机最优控制
    MPC 模型预测控制
      预测模型
        状态预测方程
        预测时域 Np
      滚动优化
        控制时域 Nc
        代价函数最小化
        约束处理
          状态约束
          控制输入约束
          控制增量约束
      反馈校正
        误差补偿
        在线更新
      MPC 变体
        线性 MPC
        非线性 NMPC
        显式 MPC
        鲁棒 MPC
        随机 MPC
    时间最优控制
      Bang-Bang 控制
      切换曲线
      最短时间问题
    代价函数设计
      二次型代价
      终端代价
      参考跟踪代价
      障碍物避让代价
```

---

## 分支三：鲁棒控制

```mermaid
mindmap
  root((鲁棒控制))
    滑模控制 SMC
      滑模面设计
        线性滑模面
        积分滑模面
        终端滑模面
      等效控制
      切换控制
      抖振问题 Chattering
      抑制方法
        边界层法
        超螺旋算法
        高阶滑模
      应用
        姿态控制
        容错控制
        抗干扰
    H-infinity 控制
      不确定性描述
      H-inf 范数
      混合灵敏度设计
      权重函数选择
      Riccati 方程
      回路成形 Loop Shaping
    自抗扰控制 ADRC
      跟踪微分器 TD
        过渡过程
        微分信号提取
      扩张状态观测器 ESO
        总扰动估计
        实时补偿
      非线性状态误差反馈 NLSEF
      ADRC 优势
        不依赖精确模型
        强鲁棒性
        工程实现简单
    不确定性建模
      参数不确定性
      未建模动态
      外部扰动
      模型集描述
    鲁棒 MPC
      Min-Max MPC
      管道 MPC Tube MPC
      约束鲁棒不变集
```

---

## 分支四：自适应控制

```mermaid
mindmap
  root((自适应控制))
    MRAC 模型参考自适应
      参考模型设计
      自适应律
        MIT 规则
        Lyapunov 方法
      参数估计
      稳定性保证
      无人机应用
    增益调度 Gain Scheduling
      工作点选取
      线性化模型族
      插值策略
        线性插值
        凸组合
      调度变量选择
        速度
        高度
        姿态角
    L1 自适应控制
      预测器设计
      自适应律
      低通滤波器
      快速自适应
      解耦鲁棒性与自适应速度
    自适应反步法 Backstepping
      Lyapunov 函数递推
      虚拟控制量
      参数更新律
      非线性系统适用
    在线参数辨识
      递推最小二乘 RLS
      梯度下降法
      模型参数实时更新
      辨识与控制协同
```

---

## 分支五：智能控制

```mermaid
mindmap
  root((智能控制))
    强化学习 RL
      算法选择
        PPO 近端策略优化
        SAC 软演员评论家
        TD3 双延迟 DDPG
        DDPG 深度确定性策略梯度
      状态空间设计
        传感器输入
        姿态/位置信息
      动作空间设计
        电机转速
        控制力矩
      奖励函数设计
        跟踪误差惩罚
        能耗惩罚
        安全约束惩罚
      训练框架
        OpenAI Gym 环境
        PyBullet / Gazebo
        Simulink Co-Sim
        Sim-to-Real 迁移
    神经网络控制
      神经网络 PID
      模型预测 NN
      端到端控制
      神经网络观测器
    模糊控制 Fuzzy
      模糊化 Fuzzification
      模糊规则库
      推理机 Inference
      解模糊 Defuzzification
      模糊 PID
    遗传算法优化 GA
      控制参数优化
      适应度函数
      编码与选择策略
      多目标优化
    混合方法
      RL + PID
      NN + MPC
      模糊自适应 PID
      强化学习微调
```

---

## 控制算法对比总表

| 算法 | 复杂度 | 鲁棒性 | 实时性 | 模型依赖 | 调参难度 | 适用场景 |
|------|--------|--------|--------|----------|----------|----------|
| PID | 低 | 中 | 极好 | 低 | 中 | 通用基础控制 |
| LQR | 中 | 中 | 好 | 高 | 中 | 线性化模型 |
| MPC | 高 | 中高 | 中 | 高 | 高 | 约束优化 |
| SMC | 中 | 高 | 好 | 中 | 中 | 抗干扰/容错 |
| ADRC | 中 | 高 | 好 | 低 | 中 | 不确定系统 |
| MRAC | 中高 | 高 | 好 | 中 | 高 | 参数变化 |
| RL | 极高 | 中高 | 好 | 低 | 极高 | 复杂非线性 |

---

## 控制架构层级

```mermaid
graph TD
    subgraph 任务层
        A[航点跟踪] --> B[轨迹规划]
        B --> C[路径生成]
    end

    subgraph 导航层
        D[状态估计 EKF]
        E[位置控制 PID/MPC]
        F[速度控制]
    end

    subgraph 控制层
        G[姿态控制 PID/LQR/SMC]
        H[角速率控制 PID]
        I[控制分配]
    end

    subgraph 执行层
        J[电机混合器]
        K[ESC 电调]
        L[无刷电机]
    end

    A --> D
    C --> E
    E --> F
    F --> G
    G --> H
    H --> I
    I --> J
    J --> K
    K --> L

    style A fill:#4CAF50,color:#fff
    style B fill:#4CAF50,color:#fff
    style C fill:#4CAF50,color:#fff
    style D fill:#2196F3,color:#fff
    style E fill:#2196F3,color:#fff
    style F fill:#2196F3,color:#fff
    style G fill:#FF9800,color:#fff
    style H fill:#FF9800,color:#fff
    style I fill:#FF9800,color:#fff
    style J fill:#f44336,color:#fff
    style K fill:#f44336,color:#fff
    style L fill:#f44336,color:#fff
```
