# 验证

所有数字来自 R2024a 在本机的一次实际运行，不是估计值。复现命令：

```matlab
addpath('E:\path\to\drone-simulink');
runtests('tests')            % 38 项，约 19 s
cd scripts
build_quad_model
validate_model
build_fw_model
validate_fw_model
uav.demo('step')
uav.demo('fw')
```

## 测试清单

`runtests('tests')` 共 38 项：`tPlant` 13 项、`tController` 7 项、`tClosedLoop` 5 项、`tFixedWing` 13 项。全部通过，总耗时 19.3 s。

### tPlant（13，四旋翼）

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

### tController（7，四旋翼）

| 测试 | 断言内容 |
| --- | --- |
| hoverCommandIsWeight | 悬停时推力等于 `m*g`，力矩为零 |
| northErrorCommandNoseDown | 北向正误差产生负俯仰角指令 |
| eastErrorCommandRollRight | 东向正误差产生正滚转角指令 |
| yawErrorProducesYawMoment | 航向误差产生同号偏航力矩 |
| thrustIsCapped | 大误差下推力不超过 40 N |
| momentIsCapped | 大姿态误差下力矩不超过 `maxM` |
| integralWindsUpBounded | 积分项限幅，水平通道不累积 |

### tClosedLoop（5，四旋翼）

| 测试 | 断言内容 |
| --- | --- |
| hoverFromOffsetConverges | 从 0.5 m 偏移起飞，末段误差小于 5 cm，无饱和 |
| positionStepConverges | 3 m 阶跃，末段误差小于 5 cm，倾角小于 40° |
| yawStepConverges | 0.5 rad 航向阶跃，末端误差小于 0.02 rad |
| repeatedRunsAreIdentical | 同一输入跑两次，结果逐位相同 |
| quaternionStaysNormalized | 全程 `norm(q) - 1` 小于 1e-10 |

### tFixedWing（13，固定翼）

| 测试 | 断言内容 |
| --- | --- |
| trimIsEquilibrium | 配平状态代回刚体方程，加速度与角加速度小于 1e-9 |
| trimLiftMatchesWeight | 配平升力等于重力（2% 内），升阻比大于 10，油门在 0–0.5 |
| liftIncreasesWithAngleOfAttack | 迎角增大使机体 z 向力更负（升力变大） |
| elevatorSignIsNoseDown | 升降舵正偏使俯仰力矩减小 |
| aileronRollsRight | 副翼正偏使滚转力矩增大 |
| sideslipProducesSideForce | 侧滑产生侧力，方向与 `CYb` 一致 |
| eulerQuatRoundTrip | 欧拉角与四元数往返误差小于 1e-10 |
| actuatorStepMatchesFirstOrder | 执行机构一步后的位置等于 `1 - exp(-dt/tau)` |
| levelFlightHoldsAltitude | 10 s 平飞高度偏差小于 2 m，空速偏差小于 1 m/s |
| climbReachesReference | 爬升 30 m，末端误差小于 0.5 m，无过冲超过 1.5 m |
| headingTurnConverges | 转弯 60°，末端误差小于 2°，滚转不超过 40° |
| quaternionStaysNormalized | 12 s 全程四元数范数偏差小于 1e-12 |
| repeatedRunIsIdentical | 同一输入跑两次，结果逐位相同 |

## 演示结果

`uav.demo` 三个四旋翼案例的实测输出：

| 案例 | 末端位置 (m) | 末段误差 max / rms (m) | 最大倾角 | 饱和占比 |
| --- | --- | --- | --- | --- |
| hover | [-0.000, -0.000, -4.999] | 0.031 / 0.020 | 0.0° | 0.0% |
| step | [3.000, 2.000, -4.999] | 0.006 / 0.002 | 30.5° | 1.6% |
| yaw | [-0.000, -0.000, -5.011] | 0.013 / 0.010 | 0.0° | 1.8% |

hover 的 3 cm 误差来自 1 m 高度爬升后的稳态偏差，垂向积分项在 12 s 内没有完全消掉。step 的 30.5° 倾角是 3 m 阶跃的瞬态峰值，饱和集中在阶跃后约 0.3 s。

固定翼 `uav.demo('fw')`：

| 量 | 结果 |
| --- | --- |
| 末端高度 / 航向 / 空速 | 130.01 m / 60.0° / 22.02 m/s |
| 高度范围 | 100.0 – 130.1 m |
| 空速范围 | 16.1 – 22.4 m/s |
| 最大舵偏 | 升降舵 14.6°，副翼 15.1°，方向舵 2.4° |

## 脚本与 Simulink 对拍

同一组方程，两套独立实现。脚本 1 ms RK4，控制器 1 kHz；模型 `ode4`、1 ms、控制器与执行机构均为 1 ms 离散图表。两边都先更新执行机构、再在整步内视为常值积分，记录也都在积分之前。

四旋翼，比较 14 s 的 13 维状态：

| 分量 | 最大偏差 | 分量 | 最大偏差 |
| --- | --- | --- | --- |
| pn | 8.9e-16 | qw | 3.6e-13 |
| pe | 6.7e-16 | qx | 3.0e-14 |
| pd | 8.9e-16 | qy | 6.4e-14 |
| u | 6.7e-16 | qz | 7.1e-14 |
| v | 1.5e-15 | p | 1.0e-14 |
| w | 1.2e-15 | q | 1.1e-14 |
| | | r | 4.0e-15 |

位置轨迹最大偏差 1.1e-15 m，终点偏差 8.9e-16 m，等价于双精度舍入量级。

固定翼，比较 60 s 的 13 维状态：最大偏差 1.3e-12，位置偏差 0 mm（打印精度内）。

早期版本对拍差 12 mm，两个原因都在顺序上：脚本在积分之后才更新电机、且日志记录的是积分后的状态。把执行机构更新与记录都移到积分之前，偏差降到 1e-15。

`validate_model` 与 `validate_fw_model` 的通过条件都是 13 维状态最大偏差小于 1e-9，打印“对拍通过”即代表两套实现给出同一条轨迹。

## 已知偏差与限制

- 线阻力系数 0.08 N/(m/s) 是量级估计，没有风洞或飞行数据标定。
- 固定翼气动导数是公开文献的通用量级，未用某型飞机的风洞数据。
- 电机时间常数 0.02 s、舵机 0.05 s 对应常见小型电动模型，未做阶跃辨识。
- 姿态控制在小角度下工作。超过 60° 的机动没有验证。
- 固定翼增益只在 18 m/s 平飞点整定。
