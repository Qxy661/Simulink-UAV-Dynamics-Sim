# 固定翼

固定翼与四旋翼共用同一套状态、刚体方程与积分器；力与力矩换成气动模型，控制量变成油门与三个舵面。参数在 `uav.cfg.fw()`，控制器增益在 `uav.cfg.fwctrl()`。

## 机体与环境

| 量 | 值 |
| --- | --- |
| 质量 | 5 kg |
| 翼面积 S | 0.55 m² |
| 翼展 b | 2.5 m |
| 平均气动弦 c | 0.22 m |
| 惯量 I | diag(0.45, 0.60, 0.85) kg·m² |
| 密度 | 1.225 kg/m³ |
| 最大推力 | 40 N |
| 舵面限幅 | 升降舵 25°，副翼 20°，方向舵 25° |
| 时间常数 | 油门 0.15 s，舵机 0.05 s |

## 气动

迎角、侧滑角：

```
alpha = atan2(wb, u)
beta  = asin(v / max(|Vb|, 1))
qbar  = 0.5 * rho * V^2
```

| 通道 | 系数式 |
| --- | --- |
| 升力 | `CL = CL0 + CLa*alpha + CLq*q*c/(2V) + CLde*de` |
| 阻力 | `CD = CD0 + kind*CL^2` |
| 侧力 | `CY = CYb*beta + CYdr*dr` |
| 滚转 | `Cl = Clb*beta + Clp*p*b/(2V) + Clr*r*b/(2V) + Clda*da` |
| 俯仰 | `Cm = Cm0 + Cma*alpha + Cmq*q*c/(2V) + Cmde*de` |
| 偏航 | `Cn = Cnb*beta + Cnp*p*b/(2V) + Cnr*r*b/(2V) + Cndr*dr` |

气动力换到机体轴，`T` 沿机体 x 轴：

```
Xa = -D*cos(alpha) + L*sin(alpha)
Za = -D*sin(alpha) - L*cos(alpha)
Fb = [Xa + T; Y; Za]
```

系数取值：

| 系数 | 值 | 系数 | 值 |
| --- | --- | --- | --- |
| CL0 | 0.25 | Cm0 | 0.05 |
| CLa | 5.0 /rad | Cma | −0.80 /rad |
| CLq | 3.9 | Cmq | −12.0 |
| CLde | 0.40 /rad | Cmde | −1.20 /rad |
| CD0 | 0.025 | Clb | −0.090 /rad |
| kind | 0.045 | Clp | −0.50 |
| CYb | −0.60 /rad | Clr | 0.080 |
| CYdr | 0.19 /rad | Clda | 0.180 /rad |
| Cnb | 0.060 /rad | Cnp | −0.030 |
| Cnr | −0.120 | Cndr | 0.090 /rad |

`Cmde` 为负：舵面后缘下偏产生低头力矩。控制器里凡是俯仰误差项都带负号，早期版本漏了这个符号，姿态在 2 s 内翻掉。

## 配平

定直平飞时航迹角为零，俯仰角等于迎角。机体 x 轴上有三项要平衡：阻力的前向分量、升力的前向分量、重力的前向分量。

```
T = D*cos(a) - L*sin(a) + W*sin(a)
```

升力需求与阻力耦合（`L*cos(a) + D*sin(a) = W*cos(a)`），用 6 轮不动点迭代收敛。18 m/s 的结果：

| 量 | 值 |
| --- | --- |
| 迎角 | 2.196° |
| 升降舵 | 0.923° |
| 油门 | 0.0929（推力 3.72 N） |
| CL | 0.448 |
| 升阻比 | 13.2 |
| 配平残差 | 加速度 1e-15 量级 |

把配平状态代回 `rigid_body`，`Vb_dot` 与 `w_dot` 都在 1e-12 以下。第一次写的时候漏了 `W*sin(a)`，残差 −0.379 m/s²，飞机匀速下沉。

## 控制器

三条回路，全部以配平量为前馈：

```
高度:  theta_cmd = alpha_trim + Kth_h*(h_ref - h) - Kth_hd*hdot
       hdot = V*sin(theta - alpha)
俯仰:  q_cmd = Kq_th*(theta_cmd - theta)
       de = de_trim - Kde_q*(q_cmd - q) - Kde_th*(theta_cmd - theta)
航向:  phi_cmd = Kphi_psi*wrap_pi(psi_ref - psi)
滚转:  p_cmd = Kp_phi*(phi_cmd - phi)
       da = Kda_p*(p_cmd - p)
偏航:  dr = -Kdr_r*r + Kdr_phi*phi
速度:  thr = thr_trim + KpV*(V_ref - V) + iV
```

| 参数 | 值 |
| --- | --- |
| Kth_h | 0.030 rad/m |
| Kth_hd | 0.040 rad/(m/s) |
| thMax | 0.35 rad |
| Kq_th | 2.0 1/s |
| Kde_q | 0.60 |
| Kde_th | 0.50 |
| KpV | 0.150 |
| KiV | 0.020 |
| iVmax | 0.30 |
| Kphi_psi | 0.80 |
| phiMax | 0.52 rad |
| Kp_phi | 3.0 |
| Kda_p | 0.40 |
| Kdr_r / Kdr_phi | 0.50 / 0.10 |

配平量在控制器里按当前空速参考现场求解，算法与 `fw_trim` 相同，因此改速度参考时前馈跟着变，不依赖某一张查找表。

## 任务与结果

`uav.demo('fw')` 跑 60 s：t=5 s 从 100 m 爬升到 130 m，t=20 s 右转 60°，t=35 s 加速到 22 m/s。

| 量 | 结果 |
| --- | --- |
| 末端高度 | 130.01 m |
| 末端航向 | 60.0° |
| 末端空速 | 22.02 m/s |
| 高度范围 | 100.0 – 130.1 m |
| 空速范围 | 16.1 – 22.4 m/s |
| 最大滚转角 | 28.2° |
| 最大俯仰角 | 19.7° |
| 最大升降舵 | 14.6° |
| 最大副翼 | 15.1° |
| 最大方向舵 | 2.4° |

爬升过程空速掉到 16.1 m/s，是油门回路带宽有限、同时升力需求增加导致的，属真实耦合，不是数值问题。转弯时滚转 28°、航向用 15 s 收敛到 60°。

## Simulink 模型

`scripts/build_fw_model.m` 生成 `models/fw_closed_loop.slx`，三个图表：

| 图表 | 采样 | 输入 → 输出 |
| --- | --- | --- |
| Ctrl | 1 ms 离散 | `x(13), ref(3)` → `cmd(4)` |
| Act | 1 ms 离散 | `cmd(4)` → `act(4)`，持久变量保存执行机构位置 |
| Plant | 纯函数 | `x(13), act(4)` → `xdot(13)` |

`scripts/validate_fw_model.m` 用同一参考与初值跑脚本实现，比较 60 s 的 13 维状态，最大偏差 1.3e-12（位置 0 mm 量级）。

图表里的配平、气动与刚体方程都是生成时写死的字面量加局部函数，不调用 `+uav` 包。原因见 README 的设计取舍：MATLAB Function 块里调用外部函数会让 Simulink 推断不出信号尺寸。

## 限制

- 线性气动导数，没有失速、没有分离、没有非定常。
- 增益在 18 m/s 整定，低速与大迎角段没有调度。
- 配平只在平飞点求解，没有爬升、转弯的配平，爬升靠外环修正。
- 侧风、阵风、地面效应都没建模。
