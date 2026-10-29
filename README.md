# 四旋翼 Simulink 仿真框架

一套可运行的无人机闭环仿真代码：MATLAB 包做数值实现，Simulink 模型做结构验证，两者独立编写同一组方程，互相对拍。

不是教程仓库，也不包含论文。代码能跑，测试能过，数值能复现。

## 当前状态

| 项目 | 结果 |
| --- | --- |
| 单元与闭环测试 | 25 项全部通过，耗时 3.9 s |
| 位置阶跃跟踪 | 末端误差 0.003 m，最大倾角 32.5°，饱和占比 2.6% |
| Simulink 对拍 | 14 s 位置轨迹最大偏差 12 mm，终点偏差 0.04 mm |
| Simulink 版本 | R2024a，无需额外工具箱 |

## 快速开始

```matlab
addpath('E:\path\to\drone-simulink');
uav.demo('step');          % 位置阶跃，出图到 figures/
uav.demo('hover');         % 定点悬停
uav.demo('yaw');           % 航向阶跃
```

跑测试：

```matlab
runtests('tests')
```

生成并校验 Simulink 模型：

```matlab
cd scripts
build_quad_model          % 生成 models/quad_closed_loop.slx
validate_model             % 与脚本实现对拍
```

## 目录

```
+uav/
  +cfg/        参数：机体、控制器、仿真设置
  +plant/      刚体六自由度、混控、电机、四元数工具
  +ctrl/       级联控制器、角度归一化
  +env/        悬停与阶跃参考
  +sim/        积分器、指标统计
  +model/      Simulink 图表源码生成器
  demo.m       演示入口
tests/         matlab.unittest 测试
scripts/       模型生成与对拍脚本
docs/          架构与验证说明
models/        生成的 .slx
figures/       演示图
```

## 约定

状态 13 维，顺序固定：

```
x = [pn; pe; pd; u; v; w; qw; qx; qy; qz; p; q; r]
```

- 世界系 NED，机体系 FRD，四元数为 body 到 NED。
- 欧拉角按 ZYX 顺序解算，`quat2euler` 返回 `[phi; theta; psi]`。
- 电机编号：1 右前、2 右后、3 左后、4 左前。1/3 逆时针，2/4 顺时针，`a = arm/sqrt(2)`。
- 混控矩阵 `B` 把转速平方映射到 `[T; Mx; My; Mz]`，控制器用 `inv(B)`。

完整推导与符号见 [docs/architecture.md](docs/architecture.md)，测试数值见 [docs/validation.md](docs/validation.md)。

## 设计取舍

**参数用函数返回 struct，不用脚本。** 旧版把 `quad_params.m` 当脚本执行，函数里静态解析看不到 `quad` 变量，会落到内置 `quad.m` 上，报“输入参数的数目不足”。现在 `uav.cfg.quad()` 返回结构体，调用处类型明确。

**控制器带状态 struct。** 积分项存在 `st.ip`、`st.ia` 里，不用全局变量，函数保持 codegen 友好、可重复调用。

**两套实现互相对拍。** Simulink 图表代码由 `+uav/+model` 生成，参数写成字面量，模型自包含，不依赖 MATLAB 路径。方程改动后重跑生成器即可，避免两份代码手工同步。

**测试断言物理量。** 比如“向北有误差时应低头”“等转速不产生力矩”“电机一步后到 63.2%”，不是只检查函数能不能返回。

## 已知限制

- 气动只有机体轴线性阻力，没有桨叶挥舞、地面效应、风场。
- 电机模型是一阶惯性加转速限幅，没有电流环。
- 姿态控制器在小角度下工作，未做完整 SO(3) 大角度机动。
- 脚本与 Simulink 的位置轨迹偏差约 1 cm，来源是离散控制器与 RK4 子步的执行顺序差异，不是方程不一致，详见验证文档。

## English

A runnable quadrotor simulation framework: a MATLAB package for numerics and a Simulink model for structural verification, written independently from the same equations and cross-checked against each other.

25 tests pass in 3.9 s. Position step tracking settles within 3 mm. The Simulink model agrees with the MATLAB implementation to 12 mm over a 14 s trajectory.

```matlab
addpath('/path/to/drone-simulink');
uav.demo('step');
runtests('tests');
```

Conventions: world frame NED, body frame FRD, quaternion body-to-NED, state vector `[pn pe pd u v w qw qx qy qz p q r]`. Motors 1-4 are front-right, rear-right, rear-left, front-left; 1 and 3 turn counter-clockwise.

## License

MIT，见 [LICENSE](LICENSE)。
