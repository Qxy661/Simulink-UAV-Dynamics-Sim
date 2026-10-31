# 验证

所有数字来自 R2024a 在本机的一次实际运行，不是估计值。复现命令：

```matlab
addpath('/path/to/drone-simulink');
runtests('tests')            % 92 项，约 68 s
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

`runtests('tests')` 共 92 项：`tPlant` 13 项、`tController` 7 项、`tClosedLoop` 5 项、`tFixedWing` 13 项、`tTilt` 36 项、`tEstimation` 9 项、`tDisturbance` 9 项。全部通过。

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

### tTilt（30，倾转四旋翼）

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
| backTransitionRespectsEnergyBudget | 16 m/s 往返减速段的高度峰值落在 [0.5, 1.6]·V²/(2g) 区间（能量界，见 `research/REPORT_r3_energy.md`） |
| repeatedRunIsIdentical | 同一输入跑两次，21 维状态逐位相同 |
| deadRotorIsNotCommanded | 1 号电机完全失效时，分配器给它 0 转速、0 推力，其余三台仍能凑出总推力 |
| faultHardReducesDrift | 降效 35% 时按"完全失效"处理，巡航段位置漂移小于 3 m、高度偏差小于 2 m（对照：按真实降效分配 4.76 m，见 `research/REPORT_r4_fault.md`） |
| faultDetectorLatency | 残差检测器在第 3 拍（门限 60 rad/s、保持 3 拍）判死失效电机，其余三台不误报 |
| detectedFaultMatchesOracle | 只靠残差检测（不告知 η）与先知先觉策略的巡航段漂移差小于 0.3 m（实测 0 m，检测延迟 40 ms，见 `research/REPORT_r5_detection.md`） |
| thrustMarginFailsDesignRule | 本配置推力裕度落在 (1.5, 2.0) 区间；单桨失效存活要求 wMax ≥ √2·hoverW ≈ 1229 rad/s，当前 1100 不满足（见 `research/REPORT_r6_hover_fault.md`） |
| hoverSurvivesWithAdequateMargin | 把 wMax 提到 1300（裕度 2.24）后，20 m 悬停断 1 号机：高度损失小于 1 m、漂移小于 5 m、滚转小于 45° |

### tEstimation（8，传感器与 EKF）

| 测试 | 断言内容 |
| --- | --- |
| gyroBiasConverges | 零偏 0.02 rad/s，60 s 内三轴估计误差小于 10% |
| gpsDropoutDriftsSlowly | 15–45 s 丢星，靠光流维持，末端水平漂移小于 5 m |
| idealSensorsTrackHover | 全零噪声闭环，30 s 悬停末端误差小于 10 cm |
| flowAidingImprovesHover | 同样噪声下，开光流后末段 RMS 小于关闭时的一半 |
| estimateUnbiasedInHover | 悬停末段位置估计误差均值小于 10 cm |
| hoverAccuracyIsSubDecimetre | 默认噪声 60 s 悬停，末段 RMS 小于 0.12 m |
| attitudeEstimateStaysSmall | 姿态估计误差末段均值小于 2° |
| repeatedRunIsIdentical | 固定种子下两次运行 13 维状态逐位相同 |

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
| 末端高度 / 空速 / 短舱 | 20.01 m / 22.00 m/s / 90.0° |
| 高度偏差 | 加速段 max 4.00 m，末端 0.01 m |
| 水平误差 | max 4.00 m，末段 0.05 m |
| 俯仰范围 | −9.5° – +2.9° |

包线（`scripts/research/r3_envelope_sweep.m`，45 s）：峰值高度偏差 8 m/s 0.16、10 m/s 0.40、12 m/s 1.52、14 m/s 2.52、16 m/s 3.12、18 m/s 3.53、20 m/s 3.80、22 m/s 4.00、24 m/s 4.29 m；14 m/s 起 β=90°，35 s 后偏差 0.00–0.37 m，无饱和、无发散。v1.0 那组「12 m/s 掉高、14 m/s 发散」是误读：拆力预算后是**爬高**，真因是舵面力矩被固定限幅截断。细节与失败记录见 [tiltrotor.md](tiltrotor.md) 和 `research/REPORT_r1_envelope.md`。

## 状态估计闭环

`uav.sim.run_est`，60 s 悬停，控制器只用 EKF 估计值：

| 量 | 结果 |
| --- | --- |
| 位置 RMS（30 s 后） | 0.079 m |
| 位置最大偏差 | 0.130 m |
| 末端误差 | 0.078 m |
| 最大倾角 | 6.0° |
| 陀螺零偏估计 | [0.0199, −0.0149, 0.0099] rad/s |
| GPS 丢星 30 s（靠光流） | 末端漂移 0.09 m，最大 0.15 m |
| 理想传感器闭环 | 末端 0.052 m，RMS 0.055 m |

对比：只给 GPS 位置时速度估计有 0.53 m/s 常值偏差、悬停误差 0.87 m；加 GPS 地速后降到 0.01 m/s 与 0.20 m；再加 0.05 m/s 的下视光流，误差 0.079 m。光流噪声降到 0.02 m/s 只到 0.065 m，说明剩余误差是位置环对传感器噪声的响应，不是调参问题。细节见 [estimation.md](estimation.md)。

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

倾转四旋翼，比较 45 s 的 21 维状态：最大偏差 1.28e-11，位置 0.000 mm，姿态 1.0e-14 rad，短舱逐位一致。短舱、转速、舵面的滞后在两边都写成同一条一阶方程，所以 21 维全量可比，不只是 13 维刚体。

早期版本对拍差 12 mm，两个原因都在顺序上：脚本在积分之后才更新电机、且日志记录的是积分后的状态。把执行机构更新与记录都移到积分之前，偏差降到 1e-15。

`validate_model`、`validate_fw_model`、`validate_tilt_model` 的通过条件是状态最大偏差小于容差（1e-9 / 1e-9 / 1e-5），打印“对拍通过”即代表两套实现给出同一条轨迹。

## 脚本与 C 对拍

`scripts\validate_c_runner.m` 依次跑三个运行器，把 CSV 与对应脚本实现的同任务结果逐点比较，并把参考序列写入 `c\ref\`：

| 构型 | 样本 x 状态 | 最大偏差 | 出现位置 | 末端位置（两边一致） |
| --- | --- | --- | --- | --- |
| quad | 1400 x 13 | 5.86e-14 | 第 118 样本，角速度 p | `[2.999999, 2.000000, -4.999220]` |
| fw | 6000 x 13 | 9.10e-13 | 第 3291 样本，位置 pn | `[809.219198, 662.627464, -130.008134]` |
| tilt | 4500 x 21 | 1.42e-12 | 第 1791 样本，第 15 维 om1 | `[714.735936, 0.000000, -20.012150]` |

四旋翼逐通道更细：

| 通道 | 最大偏差 |
| --- | --- |
| pn、pe、pd | 4.4e-16 ~ 8.9e-16 |
| u、v、w | 8.9e-16 ~ 2.6e-15 |
| qw、qx、qy、qz | 2.5e-16 ~ 6.4e-16 |
| p、q、r | 6.5e-15 ~ 5.9e-14 |

判据 1e-6。C 侧与 MATLAB 侧用的是同一份生成的 C 代码，差别只来自 gcc 的 libm 与 MATLAB 对 `exp`/`sin`/`atan2` 的末位不同；积分越长放大越多（14 s 到 1e-14，60 s 到 1e-12，倾转 45 s 到 1e-11，因为 21 维里有转速环）。

CI 不需要 MATLAB：`.github/workflows/ci.yml` 按构型矩阵用 gcc 编译、跑运行器，再拿 `c/ref/{quad_step_ref,fw_mission_ref,tilt_mission_ref}.csv`（MATLAB 侧生成、入库）比对，容差 1e-9。

## 阵风与 ESO

40 s 悬停，平均风 [3;2;0] m/s、Dryden 湍流 2 m/s，同一随机种子：

| 控制器 | 水平 RMS m | 最大误差 m | 最大倾角 ° |
| --- | --- | --- | --- |
| PID 级联 | 0.145 | 0.190 | 1.9 |
| PID + 位置积分 | 0.078 | 0.125 | 1.9 |
| PID + ESO | 0.006 | 0.012 | 1.9 |
| PID + ESO，5 cm 量测噪声 | 0.011 | 0.022 | 10.7 |

ESO 的扰动估计 `z3` 稳态 [0.203 0.095 −0.019] m/s²，真值 `drag*W/m` 是 [0.160 0.107 0]。北向偏高 27%，因为机体为了抗风本身在倾斜，倾角带进来一个稳态分量；东向与真值基本重合。量测噪声下倾角从 1.9° 涨到 10.7°，是这套观测器在低成本 IMU 上的主要代价。

详细推导与复现命令见 [disturbance.md](disturbance.md)。

## 控制器基准与失效
`scripts/benchmark_controllers.m` 在 3.6 m 阶跃上跑三种控制器，14 s，采样 10 ms。
| 控制器 | 调节时间 s | 峰值误差 m | 末段 RMS m | 最大倾角 ° | 饱和占比 % | 单次耗时 s |
| --- | --- | --- | --- | --- | --- | --- |
| PID 级联 | 4.19 | 3.6104 | 2.16e-3 | 30.45 | 1.64 | 1.60 |
| LQR | 4.47 | 3.6056 | 1.25e-4 | 54.58 | 1.29 | 2.10 |
| MPC | 5.99 | 3.6056 | 2.02e-3 | 75.03 | 1.07 | 7.48 |
三条轨迹与误差曲线写入 `figures/benchmark_step.png`。LQR 的 RMS 最好但倾角 54.6°，已经离开 35° 限制不远；MPC 输入最平顺，饱和占比最低，但 0.2 s 预测窗口加无积分让它收得慢。
失效注入 `uav.sim.fault_inject(p, c, s, tFail, kFail, eta)`：
- `eta = 0.35`、`tFail = 5 s`、`kFail = 1`：故障前误差 0，故障后最大 0.01 m，末端 0.00 m，偏航不漂，最高转速 963 rad/s，无饱和。t = 10 s 转速 `[837.1 495.2 495.2 495.2]`。
- `eta = 0`（完全停转）：20 s 飞出去 2.34 km，发散。三旋翼的反扭矩缺口让机体以约 7 rad/s² 自旋，位置环在自旋坐标系里失效。本仓库没有实现偏航自由的三旋翼控制律，这一项写进限制。
`scripts/make_fault_gif.m` 生成 `figures/demo_fault.gif`（57 帧、711 KB），左俯视机体与航迹、右四电机归一化转速。柱状图用转速而不是有效推力：失效电机的转速被分配器补高，`act .* w^2` 与正常电机相等。
## 已知偏差与限制

- 线阻力系数 0.08 N/(m/s) 是量级估计，没有风洞或飞行数据标定。
- 固定翼气动导数是公开文献的通用量级，未用某型飞机的风洞数据。
- 电机时间常数 0.02 s、舵机 0.05 s 对应常见小型电动模型，未做阶跃辨识。
- 姿态控制在小角度下工作。超过 60° 的机动没有验证。
- 固定翼增益只在 18 m/s 平飞点整定。

## 能量一致减速（r16）

| 测试 | 断言 |
| --- | --- |
| `energyProfileDecelTracksReference` | 沿能量轨迹减速：减速段峰值偏差 <15 m，末端悬停偏差 <1.5 m |

实测 10.72 m / 0.36 m；对照定高方案 23.81 m。

## 转捩段侧风（r17）

| 测试 | 断言 |
| --- | --- |
| `esoReducesPitchInTransitionWind` | 4 m/s 侧风下 ESO 把转捩段俯仰峰值压到无 ESO 的 0.7 倍以下（实测 18.2°→8.4°） |

轨迹误差不变（9.65→9.36 m），见 `research/REPORT_r17_transition_wind.md`。
