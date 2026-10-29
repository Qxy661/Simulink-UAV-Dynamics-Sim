# MATLAB/Simulink 工具箱生态图

> 本文档以思维导图形式展示 MATLAB/Simulink 在无人机仿真领域的工具箱生态系统，涵盖建模、控制、传感器、代码生成和验证五大板块。

---

## 工具箱全景总览

```mermaid
mindmap
  root((MATLAB/<br>Simulink<br>生态系统))
    建模工具
      Aerospace Blockset
      Aerospace Toolbox
      Simscape Multibody
      Simscape Electrical
      Simulink 3D Animation
      UAV Toolbox
    控制工具
      Simulink Control Design
      Model Predictive Control Toolbox
      Robust Control Toolbox
      Control System Toolbox
      Reinforcement Learning Toolbox
      Fuzzy Logic Toolbox
    传感器工具
      Sensor Fusion Toolbox
      Navigation Toolbox
      GPS Toolbox
      IMU 建模模块
      视觉传感器
    代码生成
      Simulink Coder
      Embedded Coder
      HDL Coder
      GPU Coder
      AUTOSAR Blockset
      DO Qualification Kit
    验证工具
      Simulink Test
      Simulink Coverage
      Simulink Design Verifier
      Simulink Real-Time
      Simulink Desktop Real-Time
      Parallel Computing Toolbox
```

---

## 分支一：建模工具

```mermaid
mindmap
  root((建模工具))
    Aerospace Blockset
      6DOF 模块
        6DOF (Euler Angles)
        6DOF (Quaternion)
        Custom Variable Mass 6DOF
      大气模型
        ISA 标准大气
        气压高度计算
        风场模型
      动力装置
        通用发动机模型
        喷气发动机
        螺旋桨模型
      环境模型
        重力场
        磁场模型
        地球自转
    Aerospace Toolbox
      坐标系转换
        地心坐标 ECEF
        地理坐标 LLA
        本地坐标 NED/ENU
      轨道力学
        开普勒元素
        轨道传播
        卫星可见性
      飞行动力学
        稳定性导数
        配平分析 Trim
        线性化 Linearize
    Simscape Multibody
      多体动力学
        刚体建模
        关节 Joint
        约束 Constraint
      接触与碰撞
      力与力矩
      动画可视化
    Simscape Electrical
      电机建模
        BLDC 无刷电机
        永磁同步电机 PMSM
      电池模型
        等效电路
        SOC 估算
      电力电子
        PWM 生成
        ESC 电调
      传感器电路
    UAV Toolbox
      UAV 场景
        起飞着陆
        航线规划
      参考应用
        多旋翼模板
        固定翼模板
      PX4 集成
      Flight Log 分析
```

---

## 分支二：控制工具

```mermaid
mindmap
  root((控制工具))
    Simulink Control Design
      系统辨识
        频率响应
        阶跃响应
        参数估计
      模型线性化
        工作点 Operating Point
        Linearization Manager
        Bode/Nyquist 图
      控制器设计
        SISO Design Tool
        PID Tuner
        Root Locus Designer
    Model Predictive Control Toolbox
      线性 MPC
        mpc 对象创建
        预测模型
        约束设定
      非线性 MPC
        nlmpc 对象
        自定义代价函数
      显式 MPC
      自适应 MPC
      MPC Designer App
      代码生成支持
    Robust Control Toolbox
      H-infinity 设计
        hinfsyn
        mixsyn
      mu 分析
        结构化奇异值
        鲁棒稳定性
      不确定性建模
        ureal
        ucomplex
      回路成形 Loop Shaping
    Control System Toolbox
      传递函数
      状态空间模型
      零极点分析
      频率响应
      根轨迹
      PID 调节器
    Reinforcement Learning Toolbox
      环境建模
      Agent 定义
        DQN / PPO / SAC / TD3
      训练管理
        并行训练
        训练进度监控
      Simulink 集成
      预训练 Agent
    Fuzzy Logic Toolbox
      模糊推理系统 FIS
      模糊规则编辑
      自适应神经模糊 ANFIS
      Simulink Fuzzy 模块
```

---

## 分支三：传感器工具

```mermaid
mindmap
  root((传感器工具))
    Sensor Fusion Toolbox
      传感器模型
        IMU 惯性测量
          加速度计
          陀螺仪
          噪声参数 Allan Variance
        GPS 全球定位
          位置精度
          多径效应
          更新频率
        气压计
        磁力计
        超声波测距
      滤波算法
        卡尔曼滤波 KF
        扩展卡尔曼 EKF
        无迹卡尔曼 UKF
        粒子滤波
        互补滤波
      多传感器融合
        松耦合
        紧耦合
        图优化
    Navigation Toolbox
      航迹规划
        A* 算法
        RRT / RRT*
        PRM 概率路线图
      定位
        SLAM
        里程计 Odometry
      坐标系管理
      地形分析
    视觉传感器
      Camera Toolbox
        相机标定
        畸变校正
      Computer Vision Toolbox
        特征检测
        光流法
      深度学习
        目标检测
        语义分割
```

---

## 分支四：代码生成

```mermaid
mindmap
  root((代码生成))
    Simulink Coder
      模型编译
        加速模式
        快速加速模式
        代码生成模式
      C/C++ 代码
        算法代码
        调度代码
        接口代码
      PIL 处理器在环
      SIL 软件在环
    Embedded Coder
      嵌入式优化
        内存优化
        执行效率优化
        定点数支持
      目标硬件
        ARM Cortex-M
        ARM Cortex-A
        DSP 处理器
        FPGA SoC
      AUTOSAR 支持
      代码可追溯性
      MISRA C 合规
    PX4 集成
      PX4 Toolbox
        飞控代码生成
        HIL 接口
        MAVLink 通信
      DroneKit 集成
      QGroundControl 连接
    代码验证
      Code Inspector
      Polyspace Bug Finder
      Polyspace Code Prover
      DO-178C 认证
```

---

## 分支五：验证工具

```mermaid
mindmap
  root((验证工具))
    Simulink Test
      测试用例管理
        Test Harness
        Test Sequence
        Test Assessment
      回归测试
      等价类测试
      边界测试
      测试报告生成
    Simulink Coverage
      代码覆盖率
        语句覆盖
        分支覆盖
        MC/DC 覆盖
      模型覆盖率
        条件覆盖
        决策覆盖
      DO-178C 合规
    Simulink Design Verifier
      属性证明
      测试用例自动生成
      死逻辑检测
      整数溢出检测
      设计错误检测
    Simulink Real-Time
      实时目标机
      Speedgoat 硬件
      硬件在环 HIL
      确定性执行
      I/O 板卡支持
    仿真加速
      Parallel Computing Toolbox
        并行仿真
        蒙特卡洛加速
      GPU Computing
        GPU Coder
        大规模并行
      仿真模式
        Normal
        Accelerator
        Rapid Accelerator
        External
```

---

## 工具箱协作关系

```mermaid
flowchart TB
    subgraph 建模层
        A1[Aerospace Blockset] --> D[Simulink 模型]
        A2[Simscape Multibody] --> D
        A3[Simscape Electrical] --> D
        A4[UAV Toolbox] --> D
    end

    subgraph 控制层
        B1[Control System Toolbox] --> E[控制器设计]
        B2[MPC Toolbox] --> E
        B3[Robust Control Toolbox] --> E
        B4[RL Toolbox] --> E
    end

    subgraph 融合层
        C1[Sensor Fusion Toolbox] --> F[状态估计器]
        C2[Navigation Toolbox] --> F
    end

    D --> E
    D --> F
    E --> G[完整仿真模型]
    F --> G

    subgraph 验证层
        G --> H1[Simulink Test]
        G --> H2[Simulink Coverage]
        G --> H3[Design Verifier]
    end

    subgraph 部署层
        G --> I1[Simulink Coder]
        G --> I2[Embedded Coder]
        I1 --> J[PX4 飞控]
        I2 --> J
        G --> K[Simulink Real-Time]
        K --> L[HIL 测试平台]
    end

    style A1 fill:#2196F3,color:#fff
    style A2 fill:#2196F3,color:#fff
    style A3 fill:#2196F3,color:#fff
    style A4 fill:#2196F3,color:#fff
    style B1 fill:#4CAF50,color:#fff
    style B2 fill:#4CAF50,color:#fff
    style B3 fill:#4CAF50,color:#fff
    style B4 fill:#4CAF50,color:#fff
    style C1 fill:#FF9800,color:#fff
    style C2 fill:#FF9800,color:#fff
    style H1 fill:#9C27B0,color:#fff
    style H2 fill:#9C27B0,color:#fff
    style H3 fill:#9C27B0,color:#fff
    style I1 fill:#f44336,color:#fff
    style I2 fill:#f44336,color:#fff
```

---

## 许可证与版本要求

| 工具箱 | 许可证类型 | 最低版本建议 | 年费(学术) |
|--------|-----------|-------------|-----------|
| Aerospace Blockset | 需单独授权 | R2022a+ | 含在 Campus License |
| Simscape | 需单独授权 | R2022a+ | 含在 Campus License |
| UAV Toolbox | 需单独授权 | R2022b+ | 含在 Campus License |
| MPC Toolbox | 需单独授权 | R2022a+ | 含在 Campus License |
| RL Toolbox | 需单独授权 | R2022a+ | 含在 Campus License |
| Embedded Coder | 需单独授权 | R2022a+ | 含在 Campus License |
| Simulink Real-Time | 需单独授权 | R2022a+ | 含在 Campus License |

> 大多数高校可通过 MATLAB Campus License 获取全部工具箱授权。个人版需按工具箱单独购买。
