# 变更记录

## v1.4.0

- 姿态量测延迟预算（r20）：倾转姿态环 30 ms 延迟只让悬停漂移涨 4%，转捩峰值反降 4 cm；
  与四旋翼（5–10 ms 就失稳）相反，倾转的 EKF 可按 100–200 Hz 跑。


- 控制输入摄动敏感度（r19）：悬停漂移 0.22 m/度姿态偏置，位置白噪基本不进环；转捩段 2° 内偏置反而降低高度峰值，5° 才在末端暴露（0.39 m）。
  同时修掉 `tilt_cascade` 调度里 `V` 未定义的旧 bug。
 — 2026-10-31（未发布）

**悬停鲁棒性**

- 倾转构型的悬停此前只在"恰好从平衡点出发"时成立：给 0.5/1/2 m 初始偏差，误差都是 1.3–1.5 m、俯仰 24°，是极限环不是收敛慢。根因是姿态环阻尼不足（I=[0.9;1.2;1.6] 配 KdAtt=[1;1;0.8]，阻尼比约 0.3），四旋翼那套按 0.0035 kg·m² 调的增益不能直接搬。`c.KdAtt` 改为 `[4;4;2]` 后：2 m 偏差 0.38 m、2 m/s 侧风 0.32 m、应急着陆接地垂直速度 0.09 m/s、倾角 0.1°。新增测试 `hoverRecoversFromOffset`、`hoverHoldsModerateWind`。

**倾转悬停抗扰**

- 悬停接 ESO 扰动观测：补偿前期望加速度必须存一份给观测器，否则补偿被估两遍（2 m/s 侧风从 0.32 m 恶化到 2.44 m）。修正后抗风边界从 2 m/s 推到 5 m/s（末段误差 0.12/0.26/0.41/0.57 m，倾角 ≤1°）。6 m/s 仍翻——单变量实验证明上限由机翼面积定：S=1.6 时 6 m/s 翻扣，S=0.8 时 0.47 m，S=0.2 时 0.14 m。转捩要升力、悬停要抗风，二者对立。

**减速段（能量一致）**

- 定高减速不可行已由能量账解释（r3）；本轮把高度写成能量的函数 `h = h0 + eta*(Vmax^2 - V^2)/(2g)`（`+uav/+env/tilt_ref_energy.m`），现有控制器能贴住：减速段峰值偏差 10.72 m（定高 23.81 m），末端悬停偏差 0.36 m，末端高度 37.28 m（参考 37.27）。峰值出现在切进减速的瞬间，与 eta 无关。新增测试 `energyProfileDecelTracksReference`。

**同步与回归**

- 侧风气动改成 `asin` 截断后，模型内联代码与生成的 C 未同步，`verify_all` 失败；三个生成器与 `tilt_ctrl_code` 的调度 `min(V, ref.V+2)` 对齐后，tilt 对拍偏差从 141 降到 3.13（转速通道），位置 5.98 mm，`validate_tilt_model` 改用分组容差并注明浮点顺序差异。`verify_all` 全绿：87 项测试、三套 Simulink 对拍、三构型 C 对拍。

**测试**：87 项（`tTilt` 31 项）。

## v1.3.0 — 2026-10-31

减速段的高度代价被算清楚了，单电机失效重构做了一轮负结果分析。

**减速段**

- 三组改法都没用：独立减速模态（空速 PID + 高度 PI）峰值 22.9 m；短舱按实测
  空速调度的折中方案 22.9 m；后向加速度限幅 1.5 m/s² 后 23.2 m。原级联 23.8 m。
- 真正的限制是能量：8 kg 从 22 m/s 停下要耗散 1936 J，22 m/s 平飞阻力只有 12 N，
  100 m 内只耗 1200 J，其余变成势能。扫描 4 个速度 × 4 个减速时长，高度峰值
  满足 Δh ≈ 0.7·V²/(2g)，与减速时长几乎无关。
- 新增 `backTransitionRespectsEnergyBudget` 测试（峰值落在 [0.5, 1.6]·V²/(2g)），
  `docs/tiltrotor.md` 写明进近高度约束 `h ≥ 0.7·V²/(2g)`。

**单电机失效**

- 新增 `+uav/+plant/tilt_mix_fault.m`：按有效性 `act` 缩放混控矩阵后伪逆分配。
- 巡航段（22 m/s、β=90°）缺一台电机时高度偏差恒为 1.55 m、俯仰 4.93°，与故障
  无关——竖直力由机翼承担，旋翼只管推力，这是构型决定的。
- 位置漂移：正常 0.30 m；降效 60% 2.13 m；降效 35% **4.76 m**（最差点）；完全
  失效 5.10 m。把 η<0.5 一律按完全失效处理（`s.faultHard`）后稳定在 2.16 m。
- 负结果：按真实降效做伪逆比"硬判死"更差——分配器在剩余三台上摊出偏置解，
  偏航/滚转残差变成持续姿态偏置。`tests/tTilt.m` 增 `deadRotorIsNotCommanded`
  与 `faultHardReducesDrift`。

**失效检测**

- 新增 `+uav/+est/rotor_fault_detect.m`：用"分配指令 − 实测转速"残差做计数式
  检测（门限 60 rad/s、保持 3 拍），不预设 η。
- η=0.35 时 t=25.000 s 故障、t=25.040 s 判定（4 个控制周期），巡航段漂移
  2.16 m，与先知先觉相同；按真实降效分配是 4.76 m。偏航残余还略小
  （1.84 vs 2.11 °/s），因为判死后分配器不再尝试用坏电机。
- `run_tilt` 增 `s.detect` 开关；`tests/tTilt.m` 增 `faultDetectorLatency`、
  `detectedFaultMatchesOracle`，测试总数 69 → 71。

**悬停断桨：推力裕度不够（设计结论）**

- 20 m 悬停断 1 号机，原分配（伪逆）直接把机体带翻：高度 −106.7 m、漂移 397.9 m、
  滚转 168°。两层原因。
- 第一层是分配：秩降到 3 时最小范数会**同时折中总推力**——`U=[78.5;0;0;0]` 解出
  70.2 N，少 8.3 N。改成秩感知（把不可控的偏航从目标里剔掉）后不再翻扣。
- 第二层是物理：剩下三台里对角两台必须各出 `mg/2 = 39.25 N`，而单台上限
  `kt·wMax² = 31.46 N`。判据 **`maxT ≥ mg/2`，即 `wMax ≥ √2·hoverW`**；本机
  `wMax = 1100`、`hoverW = 868.7`，裕度 1.60，不合格。
- 验证：`wMax` 1100/1250/1300 → 最低高度 12.88/19.61/19.93 m、漂移 38.6/10.6/2.3 m。
- 新增 `+uav/+cfg/thrust_margin.m`、`thrustMarginFailsDesignRule`、
  `hoverSurvivesWithAdequateMargin` 两项测试，总数 71 → 73。

**悬停断桨后的应急着陆**

- 新增 `scripts/research/r11_emergency_landing.m`：放弃水平定点、按固定速率
  下降、航向锁当前，扫 0.8/1.2/1.6/2.0 m/s 四个下降指令。
- 0.8 m/s 落不下去（45 s 后还剩 8.94 m，实际下降率 0.25 m/s）——三台满推力
  94.4 N 够，但力矩平衡要求对角两台各 39.2 N，超出单台上限，可行域里只剩
  "带 5.6° 倾斜的半悬停"。
- 1.6 m/s 指令：28.65 s 接地、垂直速度 0.58 m/s、倾角 5.6°，漂移 254.8 m。
  下降越快漂移越小（暴露时间短）。
- 漂移的来源是偏航秩亏损导致的持续自转（约 15°/s），推力方向被航向带着转，
  轨迹是向外扩的圆弧。要控落点必须有第二路偏航执行机构。
- 新增 `emergencyLandingBoundedAttitude` 测试，总数 73 → 74。

**双桨失效：容错看几何，不看台数**

- 新增 `scripts/research/r12_multi_fault.m`，7 个工况（悬停 / 22 m/s 巡航 ×
  无故障 / 单台 / 相邻双台 / 对角双台）。
- 悬停段双桨必摔：断相邻两台时滚转只剩单边力矩，机体翻扣（倾角 178.8°、
  高度掉 177.7 m）；断对角两台姿态没翻（28.3°），高度仍掉 8.6 m。
- 巡航段对角双桨**几乎无感**：剩下 2、4 号仍是对角一对，推力与滚转能配平，
  舵面接管俯仰与偏航，水平偏差 0.3 m、高度 0.27 m。
- 巡航段相邻双桨"能飞、飞不正"：高度稳住（1.1 m），但偏航反扭矩只剩单边
  （约 10°/s 自转），轨迹漂到 30.1 m。
- 结论：容错能力取决于剩余电机是否仍构成对称几何。`tests/tTilt.m` 增
  `cruiseSurvivesDiagonalDoubleFailure`、`hoverTumblesWithAdjacentDoubleFailure`，
  总数 74 → 76。

**过渡段失效：有加速就有救**

- 新增 `scripts/research/r13_transition_fault.m`：在 β≈45°（t=11 s、空速 11 m/s）
  切断，另加 t=8 s（β=0，加速初段）与 t=14 s（β=86°）两个对照。
- 单台与对角双台失效在 β=45° 时几乎看不出来：水平偏差 1.2 m、高度 4.05 m，
  与无故障（1.1 / 4.04 m）逐位接近——机翼已经承担大部分升力。
- t=8 s 切断还能活：三台旋翼总推力只有 62.9 N，必然下沉，但任务继续加速，
  空速涨到 12 m/s 时机翼接住，高度损失 5.48 m。悬停断桨之所以致命，是
  没有空速可换。
- 相邻双台在过渡段更糟：偏航反扭矩只剩单边，机体自转，水平偏差 223.7 m。
  这条与 β 无关，靠第二路偏航执行机构解决。
- `tests/tTilt.m` 增 `transitionSingleFailureIsBenign`，总数 76 → 77。

**推力裕度 2 的代价：提转速还是加大桨盘**

- 新增 `scripts/research/r14_thrust_margin_trade.m`：把 `maxT ≥ mg/2` 折成两条
  工程路径并算代价。临界转速 1228.5 rad/s（现 1100）——单桨要 39.24 N。
- 提转速：1100 → 1229（+11.7%），桨尖 330 → 369 m/s、离心 ×1.25、最大功率 ×1.39。
  实测断桨最低高度 18.25 m、漂移 19.1 m，只是"勉强不沉"；要稳得 1250（10.6 m），
  要贴住原点得 1300（2.3 m）。
- 加大桨盘：半径 0.300 → 0.335 m（+11.7%），转速反降到 985，离心载荷不升——
  推力对直径是四次方、对转速只有平方，这条路更划算。
- `tests/tTilt.m` 增 `marginTwoThresholdMatchesAnalytic`、
  `rotorRadiusScalingBuysThrust`，总数 77 → 79。

**差动短舱补偏航：修好自转，漂移反而变大**

- `+uav/+plant/tilt_wrench.m` 支持 4 元短舱角；`+uav/+sim/run_tilt.m` 加
  `s.yawTrim`（opt-in，默认关闭）：每个控制拍解三约束小方程组
  （不产生侧向力、不产生俯仰力矩、只出偏航力矩），活着的电机各给一个
  短舱微调，`maxTrim` 12°，`beta=90°` 时自动跳过。
- 结果（悬停、t=8 s 断 1 号机）：偏航角速度 RMS 0.380 → 0.159 rad/s，
  滚转 16.6° → 9.6°，但位置漂移 38.6 m → 76.7 m。
- 原因：不自转时倾斜方向固定，位置误差一路累加；自转时倾斜方向在转圈，
  误差互相抵消——前几轮"漂移几百米"里有一部分是被自转平均掉的假象。
  差动短舱不是这个构型悬停断桨的解法，真正的解法仍是推力裕度或第二路
  偏航执行机构。代码保留为 opt-in，文档写明负结果。
- `tests/tTilt.m` 增 `yawTrimReducesSpin`、`yawTrimDoesNotFixDrift`，
  总数 79 → 81。

**光流只该在低速用：22 m/s 时 2% 标度误差就是 0.44 m/s 假速度**

- 新增 `scripts/research/r16_ekf_tilt.m`：用倾转构型的 22 m/s 全任务真值喂传感器，
  EKF 开环跑，分悬停/过渡/巡航三段统计。
- 光流全程参与：巡航位置 RMS 4.696 m；空速超过 6 m/s 关掉光流：0.713 m，差 6.6 倍。
  姿态误差全程 0.05° 以内——坏的是速度观测，不是姿态。
- `+uav/+sim/run_tilt.m` 新增 `out.f`（机体比力），供估计器验证复用。
- `tests/tEstimation.m` 增 `flowBiasGrowsWithSpeed`，总数 81 → 82。

**悬停鲁棒性：v1.2 之前的悬停只是在平衡点上站着**

- 给悬停加初始偏差发现它不收敛：0.5/1/2 m 偏差末段误差都是 1.3 m 左右、俯仰
  摆到 24°——极限环，不是恢复慢。侧风更明显：0.5 m/s 就漂 2 m、俯仰 25°。
- 根因：姿态环阻尼不足。I=[0.9;1.2;1.6] kg·m² 配 KpAtt=6、KdAtt=1，阻尼比约 0.3；
  位置环的姿态指令一有高频成分，姿态环跟不上就过冲，位置环再反向修正。
- 修法：`KdAtt` [1;1;0.8] → [4;4;2]（阻尼比约 0.9）。0.5 m 偏差 1.35 → 0.09 m、
  2 m/s 侧风 8.80 → 0.32 m、应急着陆接地垂直速度 0.58 → 0.09 m/s、倾角 5.6 → 0.1°。
- 边界仍是风：3 m/s 时末段误差 19.9 m、4 m/s 64.5 m。控制器不知道风，气动迎角与
  它算的差一截，靠误差硬顶；要往外推得把 ESO 的风扰动估计接进悬停通道。
- 顺带修 `fw_aero` 侧风下的退化：`alpha = asin(clamp(wb/Ve,±0.34))`，
  `beta = asin(clamp(v/Ve,±0.34))`——失速后气动力不再增长的粗模型。
- 新增 `scripts/research_r17_hover.m`、`research/REPORT_r13_hover_robustness.md`；
  `tests/tTilt.m` 增 `hoverRecoversFromOffset`、`hoverHoldsModerateWind`，总数 82 → 84。

**倾转悬停抗风：ESO 把边界从 2 m/s 推到 5 m/s**

- `+uav/+sim/run_tilt.m` 增加 `s.useEso` / `s.woEso` / `s.esoNoise`：用位置量测
  驱动三轴 ESO，把估计的扰动加速度从期望加速度里扣掉再进级联。
- 关键错误：ESO 的输入必须取**补偿前**的期望加速度。第一版喂的是补偿后的值，
  观测器把 `-dHat` 又估一遍，2 m/s 侧风从 0.32 m 恶化到 2.44 m、倾角 2.2° → 14°。
- 结果（30 s 悬停，末段 10 s）：2/3/4/5 m/s 误差 0.121/0.255/0.411/0.568 m，倾角
  ≤1°；不开 ESO 时 2/3/4/5 m/s 是 0.319/19.88/64.51/174.9 m，3 m/s 起就翻。
- 6 m/s 仍翻：四台转速顶到 1100 rad/s 饱和、俯仰 −42°，是抗饱和问题不是补偿不够。
- 量测噪声 5 cm 几乎无影响（0.135 vs 0.121 m）；带宽 wo=4 明显差（1.71 m @4 m/s），
  wo=8 与 wo=12 基本一致。
- 新增 `tests/tTilt.m` 的 `esoExtendsTiltHoverWind`（5 m/s 末段 <1.5 m、倾角 <10°），
  总数 84 → 85；`scripts/research/r18_tilt_wind_eso.m`、
  `research/REPORT_r14_tilt_wind_eso.md` 留档。

**6 m/s 侧风：边界是机翼，不是控制器**

- 先怀疑补偿量没限幅，加 `s.esoMax` 扫 1/2/4/6 m/s² 四档——**结果逐位不变**
  （5 m/s 都 0.62 m，6 m/s 都 126.5 m 翻扣），排除补偿幅度。
- 逐拍回放：机体为抗风抬头 20.8°，机体系迎角被截到 19.5°，`CL ≈ 2.15`，
  升力约 76 N（重力 78.5 N），推力同时加到 105 N → 上冲 → 迎角再变 → 翻扣。
- 单变量验证：只改机翼面积，1.6 / 0.8 / 0.2 m² 在 6 m/s 下末段误差
  126.50 / **0.47** / 0.14 m；0.8 m² 在 8 m/s 是 3.32 m，0.2 m² 是 0.24 m。
- 结论：悬停抗风上限由机翼面积决定；转捩段要升力、悬停段要抗风，在这套构型上
  对着干。三条候选路线（倾角限幅、短舱小角度前倾抗风、襟翼卸升力）留待选择。
- 新增 `tests/tTilt.m` 的 `reducedWingImprovesHoverWind`（S=0.8、6 m/s <1.5 m），
  总数 85 → 86；`scripts/research/r19_hover_wing_wind.m`、
  `research/REPORT_r15_hover_wing_wind.md` 留档。

**留档**

- `research/REPORT_r3_energy.md`、`research/REPORT_r4_fault.md`、
  `research/REPORT_r5_detection.md`、`research/REPORT_r6_hover_fault.md`、
  `research/REPORT_r7_emergency.md`、`research/REPORT_r8_multifault.md`、
  `research/REPORT_r9_transition_fault.md`、`research/REPORT_r10_margin_trade.md`、
  `research/REPORT_r11_yawtrim.md`、`research/REPORT_r12_ekf_tilt.md`、
  `scripts/research/r7_feasibility.m`、`scripts/research/r8_fault_tilt.m`、
  `scripts/research/r9_detect_fault.m`、`scripts/research/r10_hover_fault.m`、
  `scripts/research/r11_emergency_landing.m`、`scripts/research/r12_multi_fault.m`、
  `scripts/research/r13_transition_fault.m`、`scripts/research/r14_thrust_margin_trade.m`、
  `scripts/research/r15_yawtrim.m`、`scripts/research/r16_ekf_tilt.m`。

## v1.2.0 — 2026-10-30

倾转四旋翼包线：从 β=57°、10 m/s 扩到 β=90°、24 m/s。过程中推翻了 v1.0 的结论。

**包线**

- v1.0 说「12 m/s 起必然掉高、14 m/s 发散」。逐样本拆竖直力预算后发现是**爬高**：
  12 m/s 时正偏差 16.7 m、负偏差只有 0.4 m，机翼升力 126 N 大于重力 78.5 N。
- 真因是舵面力矩被固定限幅截断：`de = M/(qbar·S·c·|Cmde|)`，qbar 从 20 Pa 涨到
  350 Pa 时同一个 2.5 N·m 只换 1.9° 舵偏，而拉回 16 m 高度误差需要 17°。
- 修法一：舵面支力矩按 `qbar/qAttRef` 放大，限幅由舵面真实权限反推。
- 修法二：固定翼支加高度积分（`Kth_hi = 0.006 rad/(m·s)`，限幅 40 m·s），
  把 β=90° 时的残余爬升压掉。
- 结果：8–24 m/s 全稳，峰值 0.16–4.29 m（都在加速段），35 s 后 0.00–0.37 m，
  14 m/s 起 β=90°，无饱和、无发散。任务改成 0→22 m/s，末端高度 20.01 m、空速 22.00 m/s。

**其他**

- 新增 `scripts/research/`（`r1_envelope_budget.m`、`r3_envelope_sweep.m`、
  `r2_full_roundtrip.m`）与 `research/REPORT_r1_envelope.md`，误判与失败数据一并留档。
- `docs/tiltrotor.md` 重写包线、控制律与未解决项；README/validation 同步。
- Simulink 倾转模型重建，对拍 1.28e-11（4500 样本、21 维）；C 代码重新生成、编译、
  对拍通过；`tTilt` 11 项判据更新。

**已知边界**

- 减速回悬停未打通：70 s 往返测试末端高度 26.1 m、空速 4.17 m/s，不收敛。
- 短舱速率 30°/s 未参与调度；过渡时间压到 5 s 内会成为瓶颈。

## v1.1.0 — 2026-10-30

真实化与抗扰：控制器不再吃真值状态，风也不再是零。

**状态估计**

- 传感器模型：IMU（常值零偏 + 白噪声）、GPS（位置与地速分开）、气压高度、三轴磁向量、下视光流。
- 15 维误差状态 EKF（`[δp; δv; δθ; δbg; δba]`，四元数单独维护，Joseph 形式修正，加计姿态双重门控）。
- 60 s 悬停末段 RMS 7.9 cm；陀螺零偏 0.02 rad/s 收敛到 0.2%；GPS 丢星 30 s 漂移 0.09 m（光流顶着）。
- 精度链条：只给 GPS 位置 0.87 m → 加 GPS 地速 0.20 m → 加光流 0.079 m。闭环取 1 kHz，100 Hz 下估计滞后会让姿态环失稳。

**阵风与 ESO**

- Dryden 紊流（三轴 AR(1)，`L=[200;200;50] m`）+ 平均风；阻力项改成相对风速。
- 每轴三阶线性 ESO，把扰动加速度前馈进位置环，控制器本体不改。
- 40 s 悬停、平均风 [3;2;0] m/s：PID 0.145 m → 位置积分 0.078 m → ESO 0.006 m；5 cm 量测噪声下 0.011 m，代价是倾角从 1.9° 涨到 10.7°。

**其他**

- 66 项测试（新增 `tEstimation` 8 项、`tDisturbance` 9 项）；随后补 `tTilt` 能量界一项，总数 67。
- 新增 `docs/estimation.md`、`docs/disturbance.md`、`uav.demo('wind')`、`scripts/benchmark_wind.m`。
- `verify_all` 仍是五项全过（测试 + 三个模型对拍 + C 运行器）。

**已知边界**

- 估计器只在悬停与小机动下验证；超过 60° 的姿态没有数据。
- ESO 未进 Simulink 与 codegen，增益需要按量测噪声标定。
- 光流假定下视纹理充足，没有建模高度变化与地面反光。

## v1.0.0 — 2026-10-29

第一个可运行版本。

**三个构型**

- 四旋翼：13 维刚体、混控 `B`/`inv(B)`、一阶电机模型、级联位置-姿态控制，位置阶跃末端误差 0.003 m。
- 固定翼：六通道线性气动、不动点配平（18 m/s 下 α=2.196°、de=0.923°、油门 0.0929）、级联控制，任务终点 130.01 m / 60.0° / 22.02 m/s。
- 倾转四旋翼：21 维状态、短舱调度、旋翼与舵面的权限混合，稳定走廊到 β=57.3°、10 m/s。

**三套验证**

- 49 项 matlab.unittest 测试，断言的是物理量（等转速不产生力矩、升降舵正偏低头、电机一步到 63.2%）。
- Simulink 模型由 `+uav/+model` 生成字面量参数，三个构型对拍 1.1e-15 / 1.3e-12 / 6.65e-12。
- 闭环生成 C（`codegen/`），gcc 编译后与脚本差 5.9e-14 / 9.1e-13 / 8.4e-12；CI 矩阵比对入库参考序列。

**其他**

- 控制器基准：PID / LQR / MPC 同一任务，三种指标 + 耗时。
- 单电机 35% 降效演示与 GIF。
- 11 个部分 61 篇中文教程并入 `tutorials/`。
- MIT 许可证、CI 工作流、`scripts/verify_all.m` 一键验证。

**已知边界**

- 气动为线性系数，无失速、风场、桨叶挥舞。
- 倾转构型的减速回悬停与 β=90° 全转换未打通，失败数据在 `docs/tiltrotor.md`。
- 电机完全停转时偏航不可控，只演示降效。
