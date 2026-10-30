# 文档索引

| 文件 | 内容 |
| --- | --- |
| [architecture.md](architecture.md) | 坐标系、13 维状态、刚体方程、四旋翼混控与电机、控制器结构、数据流、模块清单 |
| [fixedwing.md](fixedwing.md) | 固定翼气动系数表、配平推导、控制律与增益、任务结果、模型结构 |
| [tiltrotor.md](tiltrotor.md) | 短舱推力方向、虚拟量 U1..U4、逆混控与反解、调度与姿态混合、转捩包线与失稳原因 |
| [validation.md](validation.md) | 71 项测试逐条说明、三个构型实测数值、Simulink 对拍、C 对拍、踩过的方法论坑 |
| [codegen.md](codegen.md) | C 代码生成结构、三个入口、列主序陷阱、重新生成、编译运行、CI 矩阵 |
| [benchmark.md](benchmark.md) | PID / LQR / MPC 三种实现、线性化模型、MPC 走过的弯路、结果表、失效注入 |
| [estimation.md](estimation.md) | 传感器噪声与零偏、15 维 EKF、三个闭环约束、60 s 悬停精度、丢星与理想传感器对照 |
| [disturbance.md](disturbance.md) | Dryden 紊流模型、三阶 ESO、与位置积分的对比、噪声下的代价 |

教程在 `../tutorials/`，索引见 [tutorials/README.md](../tutorials/README.md)。
