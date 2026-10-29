# 无人机 Simulink 仿真框架

[![c-codegen](https://github.com/Qxy661/drone-simulink/actions/workflows/ci.yml/badge.svg)](https://github.com/Qxy661/drone-simulink/actions/workflows/ci.yml)

一套可运行的无人机闭环仿真代码：MATLAB 包做数值实现，Simulink 模型做结构验证，两者独立编写同一组方程，互相对拍；核心循环还能生成 C，脱离 MATLAB 运行。目前包含四旋翼、固定翼、倾转四旋翼三个构型。

不是教程仓库，也不包含论文。代码能跑，测试能过，数值能复现。

## 当前状态

| 项目 | 结果 |
| --- | --- |
| 单元与闭环测试 | 49 项全部通过 |
| 四旋翼位置阶跃 | 末端误差 0.003 m，最大倾角 30.5° |
| 固定翼任务 | 爬升 30 m、转弯 60°、加速到 22 m/s，终点高度 130.01 m、航向 60.0°、空速 22.02 m/s |
| 倾转四旋翼过渡 | β 0→57.3°、空速 0→9.99 m/s，高度偏差 0.33 m，水平误差末段 1.08 m |
| Simulink 对拍 | 四旋翼 14 s 最大偏差 1.1e-15，固定翼 60 s 1.3e-12，倾转四旋翼 45 s 6.65e-12 |
| C 代码生成 | 三个构型生成 C，gcc 编译后同一任务与脚本差 5.9e-14 / 9.1e-13 / 8.4e-12 |
| 控制器基准 | PID 4.19 s / LQR 4.47 s / MPC 5.99 s 调节到 5 cm；35% 降效恢复误差 < 1 cm，见 [docs/benchmark.md](docs/benchmark.md) |
| Simulink 版本 | R2024a，只用基础模块与 MATLAB Function |

## 快速开始

```matlab
addpath('/path/to/drone-simulink');
uav.demo('step');          % 四旋翼位置阶跃，出图到 figures/
uav.demo('hover');         % 四旋翼定点悬停
uav.demo('yaw');           % 四旋翼航向阶跃
uav.demo('fw');            % 固定翼爬升-转弯-加速
uav.demo('tilt');          % 倾转四旋翼悬停-转捩-巡航
```

跑测试：

```matlab
runtests('tests')
```

生成并校验 Simulink 模型：

```matlab
cd scripts
build_quad_model           % models/quad_closed_loop.slx
validate_model             % 与脚本实现对拍
build_fw_model             % models/fw_closed_loop.slx
validate_fw_model
build_tilt_model           % models/tilt_closed_loop.slx
validate_tilt_model
```

不用 MATLAB 跑三个构型（需要 gcc）：

```
cd c
mingw32-make
mingw32-make run           % 写 out/quad.csv、out/fw.csv、out/tilt.csv
build\uav_quad_runner      % 单独跑四旋翼
```

## 目录

```
+uav/
  +cfg/        参数：四旋翼机体、四旋翼控制器、固定翼机体、固定翼控制器、倾转四旋翼机体/控制器、仿真设置
  +plant/      刚体六自由度、四旋翼混控与电机、固定翼气动与配平、倾转四旋翼力与力矩、四元数工具
  +ctrl/       四旋翼级联控制器、固定翼级联控制器、倾转调度与分配、角度归一化
  +env/        四旋翼悬停与阶跃参考、固定翼任务参考、倾转任务参考
  +sim/        三个构型的积分器、指标统计
  +model/      Simulink 图表源码生成器
  +codegen/    C 代码生成入口
  demo.m       演示入口
tests/         matlab.unittest 测试
scripts/       模型生成、对拍、代码生成脚本
codegen/       生成的 C 源码（入库，可脱离 MATLAB 编译）
c/             独立运行器、Makefile、参考序列
docs/          架构、验证、控制器基准
models/        生成的 .slx
figures/       演示图
```

## 控制器对比基准
同一条 3.6 m 阶跃、同一架四旋翼，换三种控制器各跑 14 s。
| 控制器 | 调节时间 s | 末段 RMS m | 最大倾角 ° | 饱和占比 % | 单次耗时 s |
| --- | --- | --- | --- | --- | --- |
| PID 级联 | 4.19 | 2.16e-3 | 30.45 | 1.64 | 1.60 |
| LQR | 4.47 | 1.25e-4 | 54.58 | 1.29 | 2.10 |
| MPC | 5.99 | 2.02e-3 | 75.03 | 1.07 | 7.48 |
![控制器轨迹与误差](figures/benchmark_step.png)
单电机 35% 降效后的恢复过程：故障电机转速被伪逆分配器抬高，位置误差回到毫米级。
![单电机降效恢复](figures/demo_fault.gif)
细节、MPC 走过的弯路、完全停转为什么不做，见 [docs/benchmark.md](docs/benchmark.md)。
## 约定

状态 13 维，顺序固定，两个构型共用：

```
x = [pn; pe; pd; u; v; w; qw; qx; qy; qz; p; q; r]
```

- 世界系 NED，机体系 FRD，四元数为 body 到 NED。
- 欧拉角按 ZYX 顺序解算，`quat2euler` 返回 `[phi; theta; psi]`。
- 四旋翼电机编号：1 右前、2 右后、3 左后、4 左前。1/3 逆时针，2/4 顺时针，`a = arm/sqrt(2)`。
- 四旋翼混控矩阵 `B` 把转速平方映射到 `[T; Mx; My; Mz]`，控制器用 `inv(B)`。
- 固定翼配平变量为迎角 `alpha`、升降舵 `de`、油门 `thr`，平飞时俯仰角等于迎角。
- 倾转四旋翼状态 21 维：13 维刚体 + 短舱角 + 4 转速 + 3 舵面，β=0 推力朝上、β=90° 朝前。

完整推导与符号见 [docs/architecture.md](docs/architecture.md)，固定翼单独一份 [docs/fixedwing.md](docs/fixedwing.md)，倾转四旋翼见 [docs/tiltrotor.md](docs/tiltrotor.md)，C 代码生成见 [docs/codegen.md](docs/codegen.md)，测试数值见 [docs/validation.md](docs/validation.md)。

## 设计取舍

**参数用函数返回 struct，不用脚本。** 旧版把 `quad_params.m` 当脚本执行，函数里静态解析看不到 `quad` 变量，会落到内置 `quad.m` 上，报“输入参数的数目不足”。现在 `uav.cfg.quad()` 返回结构体，调用处类型明确。

**控制器带状态 struct。** 积分项存在 `st.ip`、`st.ia`、`st.iV` 里，不用全局变量，函数保持 codegen 友好、可重复调用。

**两套实现互相对拍。** Simulink 图表代码由 `+uav/+model` 生成，参数写成字面量，模型自包含，不依赖 MATLAB 路径。方程改动后重跑生成器即可，避免两份代码手工同步。

**同一套积分顺序。** 脚本与模型都先更新执行机构、再在整步内视为常值做 RK4，记录也都在积分之前。四旋翼早期版本在积分后更新电机，与模型差 12 mm；统一顺序后降到 1e-15 量级。

**测试断言物理量。** 比如“向北有误差时应低头”“等转速不产生力矩”“电机一步后到 63.2%”“升降舵正偏产生低头力矩”，不是只检查函数能不能返回。

**生成的 C 入库。** `codegen/` 里是 `uav.codegen.*_sim` 生成的 C 源码（三个构型各一目录），`c/` 用 gcc 编译成独立运行器。目的不是“能生成”，是让别人 clone 下来不装 MATLAB 也能跑出同一条轨迹：CI 按构型矩阵用入库的参考序列比对，误差超过 1e-9 就红。生成物入库的代价是每次改方程要重跑 `scripts/codegen_c.m`，这一点由 `scripts/validate_c_runner.m` 保证——它先跑 C 再和脚本逐点比，三个构型分别差 5.9e-14、9.1e-13、8.4e-12。

## 已知限制

- 气动只有线性系数：四旋翼是机体轴阻力，固定翼是线性升阻与三轴力矩导数，没有失速、非定常、桨叶挥舞、地面效应、风场。
- 电机与舵机都是一阶惯性加限幅，没有电流环、没有间隙与死区。
- 固定翼控制器在 18 m/s 平飞点整定，大迎角或低速段未做增益调度。
- 倾转四旋翼只做通稳的过渡段：稳定走廊到 β=57°、空速 10 m/s，减速回悬停与 β=90° 全转换未打通，见 [docs/tiltrotor.md](docs/tiltrotor.md)。
- 单电机完全停转（`eta = 0`）时反扭矩缺口让机体自旋，位置环收敛不了，20 s 飞出去 2.3 km；只演示 35% 降效。
- 未做完整 SO(3) 大角度机动，姿态误差仍按欧拉角计算。

## English

A runnable UAV simulation framework with quadrotor, fixed-wing and tilt-rotor models: a MATLAB package for numerics and Simulink models for structural verification, written independently from the same equations and cross-checked against each other.

49 tests pass. The quadrotor position step settles within 3 mm. The fixed-wing mission climbs 30 m, turns 60 degrees and accelerates to 22 m/s, settling at 130.01 m, 60.0 degrees and 22.02 m/s. The tilt-rotor transitions from hover to 10 m/s with the nacelles at 57.3 degrees, holding altitude within 0.33 m. Simulink models agree with the MATLAB implementation to 1.1e-15 (quadrotor, 14 s), 1.3e-12 (fixed wing, 60 s) and 6.65e-12 (tilt rotor, 45 s, all 21 states).

The closed loops of all three configurations are also generated to C (`codegen/quad`, `codegen/fw`, `codegen/tilt`) and built without MATLAB: `cd c && make && make run`. The C trajectories match the MATLAB ones to 5.9e-14 (quadrotor, 1400 samples), 9.1e-13 (fixed wing, 6000) and 8.4e-12 (tilt rotor, 4500, all 21 states); CI compares each against a committed reference and fails above 1e-9.

```matlab
addpath('/path/to/drone-simulink');
uav.demo('step');
uav.demo('fw');
uav.demo('tilt');
runtests('tests');
```

Conventions: world frame NED, body frame FRD, quaternion body-to-NED, state vector `[pn pe pd u v w qw qx qy qz p q r]`. Motors 1-4 are front-right, rear-right, rear-left, front-left; 1 and 3 turn counter-clockwise.

## License

MIT，见 [LICENSE](LICENSE)。
