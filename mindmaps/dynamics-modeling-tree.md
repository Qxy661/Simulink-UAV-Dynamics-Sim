# 动力学建模知识树

> 本文档以思维导图形式梳理无人机动力学建模的核心知识体系，涵盖刚体动力学、气动力模型、电机模型、坐标变换和状态空间五大分支。

---

## 总览：动力学建模知识树

```mermaid
mindmap
  root((无人机<br>动力学建模))
    刚体动力学
      牛顿第二定律 F=ma
      欧拉旋转方程
      惯性张量矩阵 I
      6-DOF 运动方程
      质心与压心关系
      科氏力与离心力
    气动力模型
      叶素动量理论 BEMT
      拉力系数 CT
      扭矩系数 CQ
      桨叶挥舞动力学
      入流模型 Inflow Model
      地面效应 Ground Effect
      涡环状态 VRS
      前飞气动干扰
    电机模型
      无刷直流电机 BLDC
      电机电气方程
      电机机械方程
      ESC 电调 PWM 映射
      电池等效电路模型
      推力-转速二次关系
      功率效率曲线
    坐标变换
      机体坐标系 Body
      NED 北东地坐标系
      ENU 东北天坐标系
      旋转矩阵 R_b2i
      欧拉角 Roll/Pitch/Yaw
      四元数 Quaternion
      轴角 Axis-Angle
      万向锁问题
    状态空间
      状态向量定义 x
      输入向量 u
      输出向量 y
      连续状态方程
      离散化方法
      线性化与 Jacobian
      能控性与能观性
      平衡点分析
```

---

## 分支一：刚体动力学

```mermaid
mindmap
  root((刚体动力学))
    平移运动
      质心位置 r
      速度矢量 v = dr/dt
      加速度矢量 a = dv/dt
      力的叠加原理
      重力 Fg = mg
      气动力 Fa
      推力 Ft
      合力 F_total
    旋转运动
      角速度 omega
      角加速度 alpha
      欧拉方程: I*omega_dot + omega x I*omega = M
      惯性张量 Ixx Iyy Izz
      惯性积 Ixy Ixz Iyz
      力矩叠加原理
      气动力矩 Ma
      陀螺效应
    六自由度方程组
      平移三自由度 x y z
      旋转三自由度 phi theta psi
      耦合关系
      非线性特征
      Simulink 6DOF 实现
    参数辨识
      质量测量
      惯性张量实验测定
      摆锤法 Pendulum Test
      CAD 模型估算
```

---

## 分支二：气动力模型

```mermaid
mindmap
  root((气动力模型))
    旋翼气动力
      拉力 T = CT * rho * n^2 * D^4
      扭矩 Q = CQ * rho * n^2 * D^5
      功率 P = 2*pi*n*Q
      推力系数 CT 曲线
      扭矩系数 CQ 曲线
      效率 eta = T/P
    叶素动量理论 BEMT
      径向分段积分
      入流比 lambda
      前进比 mu
      叶素升力 dL
      叶素阻力 dD
      轴向诱导速度
      切向诱导速度
      Prandtl 修正
    非线性效应
      桨叶失速 Stall
      压缩性效应
      气动干扰 Interference
      多旋翼尾流干扰
      涡环状态 VRS
      风湍流模型 Dryden/Von Karman
    简化模型
      一阶拉力模型 T = k_T * omega^2
      一阶扭矩模型 Q = k_Q * omega^2
      JYWModel 参数表
      实验标定方法
    地面效应
      拉力增益因子
      高度依赖关系
      近地面悬停修正
```

---

## 分支三：电机模型

```mermaid
mindmap
  root((电机模型))
    无刷电机 BLDC
      反电动势常数 Ke
      转矩常数 Kt
      相电阻 Rm
      相电感 Lm
      KV 值定义
      空载转速 n0
      堵转转矩 Tstall
    电气模型
      V = Ke*omega + Rm*I + Lm*dI/dt
      电流-转矩关系 I = T/Kt
      PWM 占空比映射
      电调建模 ESC
    机械模型
      J_m * d_omega/dt = T_motor - T_prop - T_friction
      转动惯量 J_m
      摩擦力矩模型
      动态响应时间常数
    螺旋桨匹配
      拉力 T = k_T * omega^2
      扭矩 Q = k_Q * omega^2
      静态推力测试
      动态推进效率
      螺旋桨直径选择
      螺距角选择
    电池模型
      等效电路 Rint Model
      开路电压 SOC 曲线
      放电容量 C_rating
      电压降补偿
      温度效应
      电池老化模型
```

---

## 分支四：坐标变换

```mermaid
mindmap
  root((坐标变换))
    坐标系定义
      机体坐标系 Body Frame
        原点: 质心
        x轴: 机头方向
        y轴: 右翼方向
        z轴: 下方
      地理坐标系
        NED 北东地
        ENU 东北天
        地心坐标系 ECEF
      惯性参考系
    旋转表示
      旋转矩阵 R
        正交性 R^T = R^-1
        行列式 det R = 1
        SO(3) 群
      欧拉角
        Roll phi 滚转
        Pitch theta 俯航
        Yaw psi 偏航
        旋转顺序 ZYX
        万向锁 Gimbal Lock
      四元数 q = [w x y z]
        单位四元数约束
        乘法运算
        共轭与逆
        球面线性插值 SLERP
      轴角表示
        旋转轴 n
        旋转角 theta
        Rodrigues 公式
    变换链
      Body -> NED 转换
      NED -> ENU 转换
      多级变换组合
      Simulink 实现
```

---

## 分支五：状态空间

```mermaid
mindmap
  root((状态空间))
    状态定义
      位置 p = x y z
      速度 v = vx vy vz
      姿态 phi theta psi
      角速度 p q r
      电机转速 omega_1..omega_n
      状态向量 x 维度选择
    状态方程
      连续形式 x_dot = f(x, u)
      输出方程 y = g(x, u)
      非线性形式
      仿射非线性形式
    线性化
      平衡点选取
      Jacobian 矩阵 A B C D
      小扰动假设
      线性化误差分析
      Simulink Linearize 模块
    离散化
      零阶保持 ZOH
      一阶保持 FOH
      双线性变换 Tustin
      采样频率选择
    系统分析
      能控性矩阵
      能观性矩阵
      特征值与稳定性
      零极点分析
      频率响应 Bode 图
```

---

## 建模流程总结

```mermaid
flowchart TD
    A[确定建模目标] --> B[选择坐标系]
    B --> C[建立运动学方程]
    C --> D[建立动力学方程]
    D --> E[气动力/力矩模型]
    E --> F[电机/推进系统模型]
    F --> G[组装状态方程]
    G --> H[参数标定与辨识]
    H --> I[Simulink 实现]
    I --> J[仿真验证与修正]

    style A fill:#4CAF50,color:#fff
    style B fill:#4CAF50,color:#fff
    style C fill:#2196F3,color:#fff
    style D fill:#2196F3,color:#fff
    style E fill:#2196F3,color:#fff
    style F fill:#FF9800,color:#fff
    style G fill:#FF9800,color:#fff
    style H fill:#f44336,color:#fff
    style I fill:#f44336,color:#fff
    style J fill:#f44336,color:#fff
```

## 常用公式速查

| 物理量 | 公式 | 说明 |
|--------|------|------|
| 拉力 | $T = C_T \rho n^2 D^4$ | CT为推力系数，n为转速，D为直径 |
| 扭矩 | $Q = C_Q \rho n^2 D^5$ | CQ为扭矩系数 |
| 简化拉力 | $T = k_T \omega^2$ | k_T 由实验标定 |
| 简化扭矩 | $Q = k_Q \omega^2$ | k_Q 由实验标定 |
| 欧拉方程 | $I\dot{\omega} + \omega \times I\omega = M$ | 刚体旋转动力学 |
| 四元数导数 | $\dot{q} = \frac{1}{2} q \otimes \omega_q$ | 姿态运动学 |
| 电池电压 | $V = V_{oc}(SOC) - I \cdot R_{int}$ | 等效电路模型 |
