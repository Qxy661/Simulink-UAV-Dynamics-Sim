# 验证

所有数字来自 R2024a 在本机的一次实际运行，不是估计值。复现命令：

```matlab
addpath('/path/to/drone-simulink');
runtests('tests')            % 49 项，约 35 s
cd scripts
build_quad_model
validate_model
build_fw_model
validate_fw_model
build_tilt_model
validate_tilt_model
validate_c_runner             % 需要先编译 c/ 下的运行器
uav.demo('step')
uav.demo('fw')
uav.demo('tilt')
```

## 测试清单

`runtests('tests')` 共 49 项：`tPlant` 13 项、`tController` 7 项、`tClosedLoop` 5 项、`tFixedWing` 13 项、`tTilt` 11 项。全部通过，总耗时 36.9 s。

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

### tTilt（11，倾转四旋翼）

| 测试 | 断言内容 |
| --- | --- |
| hoverIsEquilibrium | 悬停转速、β=0 代回刚体方程，加速度小于 1e-9，角加速度小于 1e-12 |
| mixRoundTrip | U → 转速 → 反解 U，往返相对误差小于 1e-10 |
| betaZeroGivesQuadRoll | β=0 时左右差动给出大于 1 N·m 滚转；β=90° 时该通道小于其 1/10 |
| yawMomentFromReactionTorque | 1/3 快、2/4 慢时偏航力矩为正 |
| betaScheduleIsMonotone | 空速 3→14 m/s 扫描，β 单调不减，两端等于 βmin/βmax |
| rotorTakesAllAtLowSpeed | k=0 时舵面份额为 0，转速接管配平 |
| surfaceTakesOverAtHighSpeed | k=1、qbar=200 时副翼偏角大于 1e-3 rad |
| missionHoldsAltitude | 45 s 任务高度偏差小于 1 m |
| missionReachesCruiseTilt | β 超过 50°，末端空速大于 9.5 m/s，俯仰不超过 20° |
| missionTracksReference | 末端水平误差小于 2 m |
| repeatedRunIsIdentical | 同一输入跑两次，21 维状态逐位相同 |

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

倾转四旋翼 `uav.demo('tilt')`：

| 量 | 结果 |
| --- | --- |
| 末端高度 / 空速 / 短舱 | 20.00 m / 9.99 m/s / 57.3° |
| 高度偏差 | max 0.33 m |
| 水平误差 | 过渡段 max 3.68 m，末段 1.08 m |
| 俯仰范围 | −7.8° – +5.8° |
| 转速峰值 | 878 rad/s，无饱和 |

包线（同任务换巡航速度，40 s）：8 m/s 高度偏差 0.50 m、10 m/s 0.33 m、12 m/s 15.61 m、14 m/s 126.26 m、16 m/s 发散。12 m/s 以上机翼升力不足以补旋翼竖直分量的缺口，属于配置边界。细节见 [tiltrotor.md](tiltrotor.md)。

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

倾转四旋翼，比较 45 s 的 21 维状态：最大偏差 6.65e-12，位置 0.000 mm，姿态 1.0e-14 rad，短舱逐位一致。短舱、转速、舵面的滞后在两边都写成同一条一阶方程，所以 21 维全量可比，不只是 13 维刚体。

早期版本对拍差 12 mm，两个原因都在顺序上：脚本在积分之后才更新电机、且日志记录的是积分后的状态。把执行机构更新与记录都移到积分之前，偏差降到 1e-15。

`validate_model`、`validate_fw_model`、`validate_tilt_model` 的通过条件是状态最大偏差小于容差（1e-9 / 1e-9 / 1e-5），打印“对拍通过”即代表两套实现给出同一条轨迹。

## 脚本与 C 对拍

`scripts\validate_c_runner.m` 跑 `c\build\uav_quad_runner.exe`，把它的 CSV 与 `uav.sim.run` 的同任务结果逐点比较。1400 个样本、13 维状态：

| 通道 | 最大偏差 |
| --- | --- |
| pn、pe、pd | 4.4e-16 ~ 8.9e-16 |
| u、v、w | 8.9e-16 ~ 2.6e-15 |
| qw、qx、qy、qz | 2.5e-16 ~ 6.4e-16 |
| p、q、r | 6.5e-15 ~ 5.9e-14 |

末端位置两边都是 `[2.999999, 2.000000, -4.999220]`。判据 1e-6，实测 5.9e-14。

C 侧与 MATLAB 侧用的是同一份生成的 C 代码，差别只来自 gcc 的 libm 与 MATLAB 对 `exp`/`sin`/`atan2` 的末位不同；14 s 积分把 1e-16 放大到 1e-14。

CI 不需要 MATLAB：`.github/workflows/ci.yml` 用 gcc 编译、跑运行器，再拿 `c/ref/quad_step_ref.csv`（MATLAB 侧生成、入库）比对，容差 1e-9。

## 已知偏差与限制

- 线阻力系数 0.08 N/(m/s) 是量级估计，没有风洞或飞行数据标定。
- 固定翼气动导数是公开文献的通用量级，未用某型飞机的风洞数据。
- 电机时间常数 0.02 s、舵机 0.05 s 对应常见小型电动模型，未做阶跃辨识。
- 姿态控制在小角度下工作。超过 60° 的机动没有验证。
- 固定翼增益只在 18 m/s 平飞点整定。
