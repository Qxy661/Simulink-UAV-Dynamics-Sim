# r18：倾转悬停抗风——ESO 把边界从 2 m/s 推到 5 m/s

r17 留下一个问题：悬停姿态阻尼修好之后，2 m/s 侧风能稳住，3 m/s 漂 19.9 m、
4 m/s 漂 64.5 m。当时判断不是增益问题，而是控制器不知道风。这一轮把阵风研究里
的 ESO 接到倾转悬停的期望加速度上，看能推到多远。

## 做法

`+uav/+sim/run_tilt.m` 增加 opt-in 的 `s.useEso`、`s.woEso`、`s.esoNoise`：
每个控制拍用位置量测驱动三轴 ESO（z3 是扰动加速度），把 `dHat` 从期望加速度里
扣掉再交给级联：

```
[eso, dHat] = uav.ctrl.eso_step(eso, pMeas, aCmdRaw, dtCtrl);
rNow.a = rNow.a - dHat;
```

`pMeas` 默认取真值位置；`s.esoNoise = 0.05` 时叠 5 cm 噪声（模拟 GPS/视觉）。

## 第一版错在哪

第一版把**补偿后的**期望加速度喂回 ESO，等于让观测器把 `−dHat` 再估一遍，
形成双份补偿的静差：wo=8 时 2 m/s 侧风从 0.32 m 恶化到 2.44 m，倾角 2.2° → 14°。
改成喂补偿前的 `aCmdRaw` 后，同一个工况变成 0.121 m / 0.1°。这类"输入里已经
含了补偿量"的错很隐蔽，看数值只会觉得是参数没调好。

## 结果（30 s 悬停，北向恒定风，末段 10 s 统计）

| 侧风 | 无 ESO 误差 / 倾角 | ESO(wo=8) | ESO + 5 cm 噪声 |
| 2 m/s | 0.319 m / 2.2° | 0.121 m / 0.1° | 0.135 m / 0.5° |
| 3 m/s | 19.88 m / 180°（翻） | 0.255 m / 0.2° | 0.269 m / 0.5° |
| 4 m/s | 64.51 m / 6.6° | 0.411 m / 0.4° | 0.417 m / 0.6° |
| 5 m/s | 174.9 m / 173.7°（翻） | 0.568 m / 0.6° | 0.580 m / 1.0° |
| 6 m/s | 247.5 m / 174.9°（翻） | 245.8 m / 175.4°（翻） | 246.1 m / 175.6°（翻） |

带宽扫描（2/3/4 m/s）：wo=4 时 0.139/0.358/1.71 m，wo=8 与 wo=12 几乎一样
（0.121/0.255/0.411 m）。取 wo=8。

## 6 m/s 为什么还是翻

不是补偿不够，是**旋翼饱和**：t=8 s 时俯仰 −42.4°、高度爬到 27.5 m、四台转速
顶到 1100 rad/s（wMax）。饱和之后姿态环失去权限，一路翻过去。5 m/s 时峰值转速
871，还有余量。

饱和的来源是 ESO 在风变大时给出更大的 `dHat`，期望加速度抬高，推力被推满；
一旦推力见顶，位置环只能靠倾斜去追，倾斜又吃掉竖直分量，正反馈。

## 边界与后续

- 悬停抗风：无 ESO 到 2 m/s；加 ESO 到 5 m/s（2.5 倍），6 m/s 因旋翼饱和翻扣。
- 量测噪声 5 cm 对结果几乎无影响（0.135 vs 0.121 m），说明限制不在量测。
- 要走 6 m/s 以上，需要"补偿量限幅 + 抗饱和"（`dHat` 先过一层限幅，或按
  `wMax` 余量回退补偿），这轮没做，留作下一轮。
- 该路径只在脚本侧：Simulink 倾转模型与 C codegen 都还没有 ESO 通道。

## 复现

```matlab
cd scripts/research; r18_tilt_wind_eso
```

写 `research/results/csv/r18_tilt_wind_eso.csv` 与
`research/results/fig/r18_tilt_wind_eso.png`。

## 变更文件

- `+uav/+sim/run_tilt.m`：`s.useEso` / `s.woEso` / `s.esoNoise`，补偿走 `rNow.a`
- `scripts/research/r18_tilt_wind_eso.m`：三组对照 × 5 个风速
- `tests/tTilt.m`：新增 `esoExtendsTiltHoverWind`（5 m/s 末段 <1.5 m）
