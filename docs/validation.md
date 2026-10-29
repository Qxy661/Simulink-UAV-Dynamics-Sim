# 验证

所有数字来自 R2024a 在本机的一次实际运行，不是估计值。复现命令：

```matlab
addpath('E:\path\to\drone-simulink');
runtests('tests')            % 25 项，约 4 s
cd scripts
build_quad_model
validate_model
uav.demo('step')
```

## 测试清单

`runtests('tests')` 共 25 项：`tPlant` 13 项、`tController` 7 项、`tClosedLoop` 5 项。全部通过，总耗时 3.9 s。

### tPlant（13）

| 测试 | 断言内容 |
| --- | --- |
| dcmIsOrthonormal | `R'*R - I` 元素级小于 1e-12，行列式为 1 |
| eulerRoundTrip | 欧拉角转矩阵再转回，误差小于 1e-10 |
| quatMultiplicationIsIdentity | 单位四元数左乘不改变结果 |
| hoverIsEquilibrium | 悬停转速下加速度约为零，垂向小于 1e-9 |
| freeFallAcceleratesDown | 无推力时 z 向加速度等于 g |
| torqueProducesAngularAcceleration | 纯滚转力矩产生对应角加速度，`wx = Mx/Ix` |
| mixerRoundTrip | 给定力/力矩经混控再算回，误差小于 1e-9 |
| mixerRejectsNegativeThrust | 负的转速平方被截断，饱和标志为真 |
| equalRotorSpeedsGiveNoMoment | 四电机同速时三轴力矩为零 |
| yawMomentSign | 1/3 加速、2/4 减速时偏航力矩为正 |
| motorStepReaches63Percent | 一阶电机一步（1 ms）后完成阶跃的 4.88%，一个时间常数（20 步）后到 63.2% |
| motorStepRespectsLimit | 指令超限时转速停在 `wMax` |
| dragOpposesVelocity | 阻力与速度反向 |

### tController（7）

| 测试 | 断言内容 |
| --- | --- |
| hoverCommandIsWeight | 悬停时推力等于 `m*g`，力矩为零 |
| northErrorCommandNoseDown | 北向正误差产生负俯仰角指令 |
| eastErrorCommandRollRight | 东向正误差产生正滚转角指令 |
| yawErrorProducesYawMoment | 航向误差产生同号偏航力矩 |
| thrustIsCapped | 大误差下推力不超过 40 N |
| momentIsCapped | 大姿态误差下力矩不超过 `maxM` |
| integralWindsUpBounded | 积分项限幅，水平通道不累积 |

### tClosedLoop（5）

| 测试 | 断言内容 |
| --- | --- |
| hoverFromOffsetConverges | 从 0.5 m 偏移起飞，末段误差小于 5 cm，无饱和 |
| positionStepConverges | 3 m 阶跃，末段误差小于 5 cm，倾角小于 40° |
| yawStepConverges | 0.5 rad 航向阶跃，末端误差小于 0.02 rad |
| repeatedRunsAreIdentical | 同一输入跑两次，结果逐位相同 |
| quaternionStaysNormalized | 全程 `norm(q) - 1` 小于 1e-10 |

## 演示结果

`uav.demo` 三个案例的实测输出：

| 案例 | 末端位置 (m) | 末段误差 max / rms (m) | 最大倾角 | 饱和占比 |
| --- | --- | --- | --- | --- |
| hover | [-0.000, -0.000, -4.999] | 0.031 / 0.020 | 0.0° | 0.0% |
| step | [3.000, 2.000, -4.999] | 0.003 / 0.002 | 32.5° | 2.6% |
| yaw | [-0.000, -0.000, -5.011] | 0.014 / 0.011 | 0.0° | 0.0% |

hover 的 3 cm 误差来自 1 m 高度爬升后的稳态偏差，主要是垂向积分项在 12 s 内没有完全消掉。step 的 32.5° 倾角是 3 m 阶跃的瞬态峰值，饱和 2.6% 集中在阶跃后的 0.3 s 内。

## 脚本与 Simulink 对拍

同一组方程，两套独立实现。脚本 1 ms RK4、控制器 1 kHz；模型 `ode4`、1 ms、控制器与电机均为 1 ms 离散图表。比较 14 s 的 13 维状态：

| 分量 | 最大偏差 | 分量 | 最大偏差 |
| --- | --- | --- | --- |
| pn | 2.0e-4 | qw | 1.2e-3 |
| pe | 5.6e-4 | qx | 2.8e-3 |
| pd | 1.2e-2 | qy | 5.0e-3 |
| u | 7.7e-3 | qz | 1.9e-3 |
| v | 2.0e-3 | p | 0.32 |
| w | 4.4e-2 | q | 0.53 |
| | | r | 5.7e-2 |

位置轨迹最大偏差 12.0 mm，终点偏差 0.038 mm。

姿态角速度在 0.3–0.5 rad/s 量级的偏差出现在阶跃后电机饱和的那 0.3 s 里，之后收敛到 1e-3 以下。原因是模型里离散控制器的输出在 1 ms 网格上更新，而脚本的控制器在 RK4 的 4 个子步里保持同一组输入，两者的执行顺序不同。位置是最严格的比较量，因为它对力和力矩的积分误差不做任何抵消。

`validate_model` 的通过条件：位置轨迹最大偏差小于 5 cm、终点偏差小于 1 mm、角速度偏差小于 1.5 rad/s。三个条件同时满足才打印“对拍通过”。

## 已知偏差与限制

- 线阻力系数 0.08 N/(m/s) 是量级估计，没有风洞或飞行数据标定。
- 电机时间常数 0.02 s 对应常见小型无刷电机加桨，未做阶跃辨识。
- 姿态控制在小角度下工作。超过 60° 的机动没有验证。
- Simulink 模型只覆盖四旋翼。固定翼、eVTOL 尚未进入模型层。
