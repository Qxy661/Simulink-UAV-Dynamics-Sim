# C 代码生成与脱机运行器

这个目录做两件事：证明 MATLAB 侧的实现可以被代码生成器完整吃掉，以及提供不需要 MATLAB 就能跑的三个构型闭环仿真。

## 结构

```
+uav/+codegen/quad_sim.m   四旋翼代码生成入口（闭环主循环）
+uav/+codegen/fw_sim.m     固定翼入口
+uav/+codegen/tilt_sim.m   倾转四旋翼入口
codegen/quad|fw|tilt/      生成结果：<构型>_sim.c/h 与运行时支撑
c/main.c                   独立运行器，一个文件配三个 -DMISSION_* 宏
c/tmwtypes.h               最小类型垫片（见下）
c/Makefile                 gcc 构建，一次编出三个可执行文件
c/ref/*.csv                MATLAB 侧参考序列，CI 比对用
c/compare_csv.py           CSV 比对
scripts/codegen_c.m        重新生成三个构型
scripts/validate_c_runner.m 脚本与 C 逐点比对，并写参考序列
```

## 入口函数

```matlab
X = uav.codegen.quad_sim(pTarget, psiTarget)   % 1400 x 13
X = uav.codegen.fw_sim()                        % 6000 x 13
X = uav.codegen.tilt_sim()                      % 4500 x 21
```

三者的时间跨度、步长、控制分频与记录间隔都取自对应脚本仿真的固定配置：

| 构型 | 任务 | 时长 | 样本 | 状态维数 |
| --- | --- | --- | --- | --- |
| quad | t=1 s 从 `[0;0;-5]` 跳到 `pTarget`，航向 `psiTarget` | 14 s | 1400 | 13 |
| fw | 100 m 巡航 → 爬 30 m → 转 60° → 加速 22 m/s | 60 s | 6000 | 13 |
| tilt | 20 m 悬停 → 加速到 10 m/s 转捩 → 巡航 | 45 s | 4500 | 21 |

它们与 `uav.sim.run` / `run_fw` / `run_tilt` 的循环逐行对应，只是把 `ref`、`st`、`prm` 换成了固定尺寸的局部量。生成签名：

```c
void quad_sim(const double pTarget[3], double psiTarget, double X[18200]);
void fw_sim(double X[78000]);
void tilt_sim(double X[94500]);
```

`X` 是列主序：第 `i` 维、第 `k` 个样本在 `X[i*N_SAMP + k]`。这一点在 C 侧写错会得到一条看似合理的错误轨迹（每行都是别人的时间序列），排查时先看第一行是不是初值。

## 重新生成

```matlab
cd scripts
codegen_c
```

只生成源码（`GenCodeOnly = true`），不需要本机装 C 编译器。三个目录各自自包含；`buildInfo.mat`、`codedescriptor.dmr` 等中间产物会被删掉，不入库。

## 编译与运行

需要 gcc（Windows 上用 MinGW 的 `mingw32-make`）：

```
cd c
mingw32-make                  # 编出 build/uav_{quad,fw,tilt}_runner
mingw32-make run              # 依次跑三个任务，写 out/{quad,fw,tilt}.csv
build\uav_fw_runner out\fw.csv
```

`main.c` 用 `-DMISSION_quad` / `-DMISSION_fw` / `-DMISSION_tilt` 切换构型，CSV 第二列起是全部状态。三个生成库里有同名文件（`rt_nonfinite.c`、`norm.c`、`rtGetInf.c`），Makefile 给目标文件加了构型前缀，否则链接时报重复符号。

## 对拍结果

`scripts/validate_c_runner.m` 依次跑三个运行器，与脚本实现逐点比较：

| 构型 | 样本 | 最大偏差 | 位置 | 出现位置 |
| --- | --- | --- | --- | --- |
| quad | 1400 x 13 | 5.86e-14 | 4.4e-16 | 第 118 样本，角速度 p |
| fw | 6000 x 13 | 9.10e-13 | 1.1e-15 | 第 3291 样本，位置 pn |
| tilt | 4500 x 21 | 1.42e-12 | 2.7e-16 | 第 1791 样本，第 15 维 om1 |

两边都是 IEEE double，差别只来自 `exp`/`sin`/`atan2` 的末位不同，长时间积分后放大到 1e-12。末端位置三构型逐位一致：quad `[2.999999, 2.000000, -4.999220]`、fw `[809.219198, 662.627464, -130.008134]`、tilt `[325.980899, 0.000000, -20.003875]`。判据取 1e-6，给跨平台（glibc 与 MSVCRT 的 libm）留余量。

## tmwtypes.h 垫片

`rtwtypes.h` 会 `#include "tmwtypes.h"`，而后者属于 MATLAB 安装目录（`extern/include`），不在仓库里。为了 clone 下来只装 gcc 就能编译，`c/tmwtypes.h` 给出了生成代码实际用到的类型（`real_T`、`boolean_T`、`true/false` 等）。若以后引入定点或复数信号，需要补类型，或者编译时改成 `-I"$MATLABROOT/extern/include"`。

## CI

`.github/workflows/ci.yml` 在 ubuntu 上按构型矩阵跑：`make` 编译、运行对应 runner、用 `compare_csv.py` 和入库的参考序列比对。三个构型互不阻塞（`fail-fast: false`），不需要 MATLAB 许可证。

参考序列由 `scripts/validate_c_runner.m` 生成，只在 MATLAB 侧实现变化时更新。

## 一个移植性提示

生成的 `norm.c` 用了 SSE2 内联函数（`_mm_set1_pd` 等），这是 MATLAB Coder 对 x86 的默认向量化。在 x86_64 上 gcc 直接能编；换 ARM（Jetson、树莓派）需要关掉向量化重生成，否则链接阶段缺 `_mm_*`。真的要在机载端跑，建议在 codegen 配置里把 `cfg.SIMD` 或目标硬件设成对应平台再生成。
