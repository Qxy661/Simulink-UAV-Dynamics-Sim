# 阵风与扰动抑制

前面几份文档里，无人机都在无风的空气里飞。这一层加两样东西：Dryden 大气紊流，和一个用来抵消它的扩张状态观测器（ESO）。全部在脚本侧完成，Simulink 模型与生成的 C 不含这部分。

## 1. 风模型

`+uav/+env/wind_dryden.m`：

```
gust = uav.env.wind_dryden(tEnd, dt, V, sigma, seed, Wmean)
```

按 Dryden 谱的三个方向各生成一条有色噪声，NED 坐标系：

```
x[k+1] = a*x[k] + b*randn
a = exp(-V*dt/L),  b = sigma*sqrt(1 - a^2)
L = [Lu; Lv; Lw] = [200; 200; 50]   % 低空，米
```

这样生成序列的稳态方差是 `sigma^2`，自相关时间 `L/V`。默认 `V = 10 m/s`、`sigma = 2 m/s`：1 米级无人机在 5 级风里遇到的湍流强度。`Wmean` 是平均风，直接叠加在湍流上。

平均风与湍流在动力学里的作用不同：平均风经阻力项变成一个常值力偏置（相当于给机体挂了个固定方向的加速度），湍流则是宽带的。ESO 对这两者的抑制能力也不同，下面的表分开看。

机体收到的气动力里，阻力项用相对速度：

```
Fb = [0; 0; -sum(T)] - drag * (Vb - Wb)
```

`+uav/+plant/quad_wrench.m` 的第 4 个参数 `Wb` 就是机体坐标系下的风速，省略时按无风处理。

## 2. ESO

`+uav/+ctrl/eso_init.m`、`+uav/+ctrl/eso_step.m`。每个轴一个三阶线性观测器：

```
z1' = z2 + 3*wo*(p - z1)
z2' = z3 + aCmd + 3*wo^2*(p - z1)
z3' = wo^3*(p - z1)
```

`z1` 追位置，`z2` 追速度，`z3` 就是"模型里没写进去的那部分加速度"——风阻、未建模阻力、质量偏差都落在它上面。`wo` 是观测器带宽，默认 8 rad/s。

补偿方式不动控制器本体：`run_wind` 每步把 `rEff.a = rNow.a - dHat` 传给 `uav.ctrl.cascade`。级联控制器的位置环本来就吃前馈加速度，减掉估计出的扰动加速度，等于让姿态环去抵消风。

为什么不用积分项：位置环的积分把风引起的稳态位置误差慢慢磨掉，但它对突加的风要等误差积累起来才出力，且积分本身在姿态环的相位裕度里再吃掉一块。ESO 是从速度通道直接观测加速度偏差，快得多。

## 3. 对比

`scripts/benchmark_wind.m`，40 s 悬停，平均风 `[3; 2; 0]` m/s，湍流强度 2 m/s，同一个种子：

| 控制器 | 水平 RMS | 最大误差 | 平均偏移 | 最大倾角 |
| --- | --- | --- | --- | --- |
| PID 级联 | 0.145 m | 0.190 m | [0.13 0.06] m | 1.9° |
| PID + 位置积分（Ki 0.3） | 0.078 m | 0.125 m | [0.07 0.01] m | 1.9° |
| PID + ESO | 0.006 m | 0.012 m | [0.00 0.00] m | 1.9° |
| PID + ESO，量测噪声 5 cm | 0.011 m | 0.022 m | [0.00 0.00] m | 10.7° |

ESO 把 RMS 从 0.145 m 压到 0.006 m，约 24 倍；位置积分只压到 0.078 m，约一半——因为积分要等误差积累，而 ESO 在速度通道里先看见了加速度偏差。

最后一行是代价：位置量测有 5 cm 噪声时，ESO 把噪声放大成扰动的估计，倾角从 1.9° 涨到 10.7°，RMS 回到 0.011 m。实际系统里 ESO 的带宽要和量测噪声一起定，`wo` 调高抑制更快，但噪声也更凶。

图在 `figures/benchmark_wind.png`：左边俯视轨迹（PID 被吹出约 0.15 m，ESO 几乎贴着原点），右边误差曲线。

## 4. 测试

`tests/tDisturbance.m` 9 项：

- `drydenStatisticsMatchInput`：900 s 长窗口、4 个种子，均值偏差 < 0.5 m/s，标准差偏差 < 20%。
- `zeroIntensityGivesMeanWind`：`sigma = 0` 时序列恒等于平均风。
- `noWindHasNoOffset`：无风时 PID 悬停误差在 1 cm 内。
- `windCausesSteadyOffset`：3 m/s 平均风下纯 PID 的水平误差超过 5 cm，说明风确实进了动力学。
- `esoHalvesTrackingError`：同一阵风下 ESO 的末段 RMS 小于纯 PID 的一半。
- `esoEstimatesWindAcceleration`：`z3` 稳态接近 `drag*W/m`，容差 35%（估计里还有湍流与机体自身运动）。
- `esoSurvivesMeasurementNoise`：5 cm 量测噪声下仍优于纯 PID 一半以上。
- `esoIsStableWithoutDisturbance`：无风时 ESO 不破坏悬停。
- `repeatedRunIsIdentical`：同种子逐位相同。

## 5. 限制

- Dryden 模型是低空、定空速的经验谱，没有考虑高度变化与阵风梯度。
- 风只加到阻力项上，没有建模旋翼对风的非定常响应（桨叶挥舞、入流变化）。真实多旋翼在阵风下的响应比这个模型更剧烈。
- ESO 用位置量测做输入，假定控制指令链路的延迟已知；实际系统里执行机构滞后会让 `z3` 出现相位偏差。
- 这一层没有进 Simulink 与 codegen。原因是 ESO 需要稳定增益与量测噪声标定，跨构型复用的意义不大；模型与 C 保持"控制律可复现"的最小集。

## 6. 复现

```matlab
addpath(pwd);
run('scripts/benchmark_wind.m')     % 出表与 figures/benchmark_wind.png
runtests('tests/tDisturbance.m')    % 9 项
```

单次运行：

```matlab
p = uav.cfg.quad(); c = uav.cfg.ctrl();
gust = uav.env.wind_dryden(40, 1e-3, 10, 2.0, 7, [3; 2; 0]);
ref = @(t) uav.env.hover(0, 0, -5, 0);
out = uav.sim.run_wind(p, c, struct('dt', 1e-3, 'ctrlDiv', 1, 'tEnd', 40), ...
                       ref, gust, [], struct('useEso', true));
```
