# C 代码生成与脱机运行器

这个目录做两件事：证明 MATLAB 侧的实现可以被代码生成器完整吃掉，以及提供一个不需要 MATLAB 就能跑的闭环仿真。

## 结构

```
+uav/+codegen/quad_sim.m   代码生成入口（闭环主循环）
codegen/                   生成结果：quad_sim.c/h 与运行时支撑
c/main.c                   独立运行器，链接生成的 C
c/tmwtypes.h               最小类型垫片（见下）
c/Makefile                 gcc 构建
c/ref/quad_step_ref.csv    MATLAB 侧参考序列，CI 比对用
c/compare_csv.py           CSV 比对
scripts/codegen_quad.m     重新生成
scripts/validate_c_runner.m 脚本与 C 逐点比对
```

## 入口函数

```matlab
X = uav.codegen.quad_sim(pTarget, psiTarget)
```

固定的 14 s 阶跃任务：t = 1 s 从 `[0;0;-5]` 跳到 `pTarget`，RK4 步长 1 ms，控制器 100 Hz，电机一阶模型精确离散，每 10 ms 记一个样本，返回 `1400 x 13` 的状态矩阵。与 `uav.sim.run` 的循环逐行对应，只是把 `ref`、`st`、`prm` 都换成了固定尺寸的局部量。

生成签名：

```c
void quad_sim(const double pTarget[3], double psiTarget, double X[18200]);
```

`X` 是列主序：第 `i` 维、第 `k` 个样本在 `X[i*1400 + k]`。这一点在 C 侧写错会得到一条看似合理的错误轨迹（每行都是别人的时间序列），排查时先看第一行是不是初值。

## 重新生成

```matlab
cd scripts
codegen_quad
```

只生成源码（`GenCodeOnly = true`），不需要本机装 C 编译器。目录里的 `buildInfo.mat`、`codedescriptor.dmr` 等中间产物脚本会删掉，不入库。

## 编译与运行

需要 gcc（Windows 上用 MinGW 的 `mingw32-make`）：

```
cd c
mingw32-make
build\uav_quad_runner            # 默认 3 2 -5 0.4，写 out/quad_step.csv
build\uav_quad_runner 5 0 -8 0.7 out\other.csv
```

## 对拍结果

`scripts/validate_c_runner.m` 跑同一个任务，逐点比较：

| 通道 | 最大偏差 |
| --- | --- |
| 位置、速度、四元数 | 4.4e-16 ~ 2.6e-15 |
| 角速度 p、q、r | 5.9e-14 |

两边都是 IEEE double，差别只来自 `exp`/`sin`/`atan2` 的末位不同，14 s 积分后放大到 1e-14。末端位置都是 `[2.999999, 2.000000, -4.999220]`。判据取 1e-6，给跨平台（glibc 与 MSVCRT 的 libm）留余量。

## tmwtypes.h 垫片

`rtwtypes.h` 会 `#include "tmwtypes.h"`，而后者属于 MATLAB 安装目录（`extern/include`），不在仓库里。为了 clone 下来只装 gcc 就能编译，`c/tmwtypes.h` 给出了生成代码实际用到的类型（`real_T`、`boolean_T`、`true/false` 等）。若以后引入定点或复数信号，需要补类型，或者编译时改成 `-I"$MATLABROOT/extern/include"`。

## CI

`.github/workflows/ci.yml` 在 ubuntu 上做三件事：`make` 编译、跑 `uav_quad_runner`、用 `compare_csv.py` 和入库的 `quad_step_ref.csv` 比对。不需要 MATLAB 许可证。

参考序列由 `scripts/validate_c_runner.m` 生成，只在 MATLAB 侧实现变化时更新。

## 一个移植性提示

生成的 `norm.c` 用了 SSE2 内联函数（`_mm_set1_pd` 等），这是 MATLAB Coder 对 x86 的默认向量化。在 x86_64 上 gcc 直接能编；换 ARM（Jetson、树莓派）需要关掉向量化重生成，否则链接阶段缺 `_mm_*`。真的要在机载端跑，建议在 codegen 配置里把 `cfg.SIMD` 或目标硬件设成对应平台再生成。
