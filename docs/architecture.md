# 架构与方程

## 状态与坐标系

世界系 NED（x 北、y 东、z 向下），机体系 FRD（x 前、y 右、z 向下）。四元数 `q = [qw; qx; qy; qz]` 表示 body 到 NED 的旋转。

13 维状态：

```
x = [pn; pe; pd; u; v; w; qw; qx; qy; qz; p; q; r]
    位置(NED)      机体系速度      四元数         机体系角速度
```

欧拉角用 ZYX 顺序从旋转矩阵反解，返回 `[phi; theta; psi]`。

## 刚体方程

```
R   = dcm(q)                       % body -> NED
p_dot = R * Vb
Vb_dot = Fb/m - cross(w, Vb) + R' * [0; 0; g]
q_dot = 0.5 * qmul(q, [0; w])
w_dot = I \ (Mb - cross(w, I*w))
```

重力项写成 `R' * [0;0;g]`，z 轴向下为正，悬停时推力 `T = m*g = 14.715 N`。

## 混控

电机编号与转向：

| 电机 | 位置 | 转向 | 对滚转 | 对俯仰 | 对偏航 |
| --- | --- | --- | --- | --- | --- |
| 1 | 右前 | 逆时针 | − | + | + |
| 2 | 右后 | 顺时针 | − | − | − |
| 3 | 左后 | 逆时针 | + | − | + |
| 4 | 左前 | 顺时针 | + | + | − |

`a = arm/sqrt(2) = 0.1591 m`。转速平方到力/力矩：

```
[T ]   [ kt   kt   kt   kt ] [w1^2]
[Mx] = [-kt*a -kt*a  kt*a  kt*a] [w2^2]
[My]   [ kt*a -kt*a -kt*a  kt*a] [w3^2]
[Mz]   [ kd  -kd   kd  -kd  ] [w4^2]
```

取 `kt = 1.5e-5`、`kd = 2.0e-7`，则反扭矩系数 `kq = kd/kt = 0.01333 m`，即 `Mz = kq * (T1 - T2 + T3 - T4)`。

控制分配用 `inv(B)`，解出 `w^2` 后把负值截断为 0、把超过 `wMax = 1200 rad/s` 的截断到上限，并置饱和标志。截断后真实力/力矩由 `quad_wrench` 重新计算，控制器不会拿到“以为实现了”的力矩。

## 电机

一阶惯性，精确离散：

```
alpha = exp(-dt/tauM)          % dt = 1 ms, tauM = 0.02 s, alpha = 0.9512
w0    = wcmd + (w0 - wcmd)*alpha
```

初值 `hoverW = sqrt(m*g/(4*kt)) = 495.2 rad/s`。一步（1 ms）完成阶跃的 4.88%，一个时间常数（20 步）后到 63.2%。

## 控制器

级联结构，位置环 100 Hz（脚本 `ctrlDiv = 10`），Simulink 模型里为 1 kHz 离散。

位置环：

```
ep = ref.p - p
ev = ref.v - Vn
a  = Kp.*ep + Kd.*ev + Ki.*ip + ref.a
a(1:2) = 限幅(a(1:2), g*tan(35°))
```

推力与期望姿态：

```
v   = [0; 0; g] - a
z_b = v / norm(v)
T   = m * norm(v)               % 限幅 40 N
```

由 `z_b` 与航向参考构造完整期望旋转矩阵：先取水平参考向量 `xc = [cos(psi); sin(psi); 0]`，`yb = cross(z_b, xc)`，再正交化得到 `xb = cross(yb, z_b)`。然后反解欧拉角：

```
phi_d   = atan2(Rd(3,2), Rd(3,3))
theta_d = -asin(Rd(3,1))
psi_d   = atan2(Rd(2,1), Rd(1,1))
```

这里不能简单写 `phi_d = asin(z_b(2))`。早期版本就是这么写的，航向不为零时符号与耦合都错，阶跃响应会直接发散到几百米。

姿态环：

```
eAtt = wrap_pi([phi_d; theta_d; psi_d] - att)
M    = KpAtt.*eAtt + KiAtt.*ia - KdAtt.*w      % 限幅 [3;3;1.5] N*m
```

`wrap_pi` 把误差折到 `(-pi, pi]`。

参数：

| 参数 | 值 |
| --- | --- |
| Kp | [1.6; 1.6; 2.5] |
| Kd | [2.2; 2.2; 3.0] |
| Ki | [0; 0; 1.2] |
| KpAtt | [6; 6; 3] |
| KdAtt | [0.35; 0.35; 0.30] |
| KiAtt | [0; 0; 0.15] |
| maxTilt | 35° |
| maxT | 40 N |
| maxM | [3; 3; 1.5] N·m |

积分项存在状态结构体里（`st.ip`、`st.ia`），调用方负责传入，不用全局变量。

## 固定翼

气动模型、配平与控制器单独写在 [fixedwing.md](fixedwing.md)。与四旋翼共用同一个 13 维状态、刚体方程与积分器，区别只在力和力矩的来源，以及控制量变成 `[throttle; de; da; dr]`。

## 倾转四旋翼

四个旋翼装在机翼上，短舱绕机体 y 轴转，β=0 推力朝上、β=90° 朝前。状态扩成 21 维：13 维刚体 + β + 4 转速 + 3 舵面。物理推导、调度与包线单独写在 [tiltrotor.md](tiltrotor.md)。

与另外两个构型的关系：刚体方程、四元数工具、气动模型（`fw_aero`）完全复用，只是叠加旋翼力 `d(β)·U1` 与力偶。控制器仍是位置环 + 姿态环，区别在姿态指令按空速权重在"四旋翼支"与"固定翼支"之间混合，力矩按同一权重在旋翼与舵面之间分配。

## 积分

脚本用定步长 RK4，步长 1 ms。控制器每 `ctrlDiv` 步更新一次，中间保持零阶保持。执行机构（电机、舵机、油门）在积分之前按本步指令更新，整步内视为常值；记录也在积分之前取，和 Simulink 积分器输出的对齐方式一致。每步结束后四元数归一化。

Simulink 模型用 `ode4`，固定步长 1 ms。控制器、电机与执行机构都是 1 ms 离散图表，刚体导数是纯函数图表，可被 RK4 的四个子步反复调用。

## 数据流

```
ref (struct 或句柄)
      |
      v
  cascade ---> [T; M] ---> mixer ---> wcmd ---> motor_step ---> w
      ^                                                        |
      |                                                        v
      +------------- x (13 维) <--- RK4 <---- rigid_body <-- Fb, Mb
```

Simulink 模型是同一张图的图形版本：

```
Step/Psi -> RefMux -> Ctrl -> TM -> Mixer -> Motors -> Wrench
              ^                                        |
              |                                        v
             Int <-------------- Deriv <-------------- FM
```

## 模块清单

| 文件 | 职责 |
| --- | --- |
| `+uav/+cfg/quad.m` | 机体参数、混控矩阵与逆 |
| `+uav/+cfg/ctrl.m` | 控制器增益与限幅 |
| `+uav/+cfg/simcfg.m` | 步长、控制分频、仿真时长 |
| `+uav/+plant/q2dcm.m` | 四元数转旋转矩阵，内部归一化 |
| `+uav/+plant/quat2euler.m` | 旋转矩阵转 ZYX 欧拉角 |
| `+uav/+plant/qmul.m` | 四元数乘法 |
| `+uav/+plant/rigid_body.m` | 13 维导数 |
| `+uav/+plant/quad_wrench.m` | 转速到力/力矩，含线性阻力 |
| `+uav/+plant/mixer.m` | 力/力矩到转速指令，带截断与饱和标志 |
| `+uav/+plant/motor_step.m` | 一阶电机离散更新 |
| `+uav/+ctrl/cascade.m` | 位置环 + 姿态环 |
| `+uav/+ctrl/wrap_pi.m` | 角度归一化 |
| `+uav/+env/hover.m` | 悬停参考结构体 |
| `+uav/+env/step.m` | 阶跃参考，返回结构体或句柄 |
| `+uav/+sim/run.m` | RK4 闭环仿真主循环 |
| `+uav/+sim/metrics.m` | 误差、倾角、饱和统计 |
| `+uav/+model/*.m` | 生成 Simulink 图表源码 |
| `+uav/demo.m` | 演示与出图 |
| `+uav/+cfg/fw.m` | 固定翼质量、惯量、气动系数、舵面限幅 |
| `+uav/+cfg/fwctrl.m` | 固定翼控制器增益 |
| `+uav/+plant/fw_aero.m` | 固定翼气动力与力矩 |
| `+uav/+plant/fw_trim.m` | 定直平飞配平（迎角、升降舵、油门） |
| `+uav/+plant/actuator_step.m` | 舵机与油门一阶滞后 |
| `+uav/+plant/euler2quat.m` | ZYX 欧拉角转四元数 |
| `+uav/+ctrl/fw_cascade.m` | 固定翼高度/航向/速度回路 |
| `+uav/+env/fw_ref.m` | 固定翼任务参考（纯函数） |
| `+uav/+env/fw_mission.m` | 固定翼任务参考句柄 |
| `+uav/+sim/run_fw.m` | 固定翼 RK4 闭环仿真 |
| `+uav/+cfg/tilt.m` | 倾转四旋翼机体、旋翼、气动、调度参数 |
| `+uav/+cfg/tiltctrl.m` | 倾转四旋翼控制器增益 |
| `+uav/+plant/tilt_wrench.m` | 旋翼力 `d(β)·U1` 与力偶，叠加气动 |
| `+uav/+plant/tilt_mix.m` | 虚拟控制量到四个转速 |
| `+uav/+ctrl/tilt_schedule.m` | 参考空速到机翼权重 k 与短舱角 β |
| `+uav/+ctrl/tilt_alloc.m` | 力矩在旋翼与舵面之间按 k 分配 |
| `+uav/+ctrl/tilt_cascade.m` | 倾转四旋翼级联控制（姿态混合） |
| `+uav/+env/tilt_ref.m` | 倾转任务参考（悬停-转捩-巡航） |
| `+uav/+env/tilt_ref_v.m` | 参数化参考，用于包线扫描 |
| `+uav/+sim/run_tilt.m` | 21 维状态 RK4 闭环仿真 |

## C 代码生成

`+uav/+codegen/quad_sim.m` 是四旋翼闭环主循环的代码生成入口，逻辑与 `uav.sim.run` 逐行对应，只把参考、控制器状态、参数都收成固定尺寸的局部量。`scripts/codegen_quad.m` 调 `codegen -config:lib -GenCodeOnly`，输出到 `codegen/`；`c/main.c` 链接这些文件构成独立运行器，`c/Makefile` 用 gcc 构建。

生成的 C 与脚本实现的差别只在于 libm 的末位，14 s 后状态最大偏差 5.9e-14（角速度通道），位置与四元数在 1e-15 量级。细节见 [codegen.md](codegen.md)。

`+uav/+cfg/quad.m` 为此改成了“先算局部变量、再一次构造结构体”：分步写 `p.arm` 再读它算 `p.a`，代码生成器会报字段锁定错误。
