# 教程

这套教程按"概念 → 建模 → 控制 → 仿真 → 实飞"排列，共 11 个部分。原来发布在单独的资料库，现在并进代码仓库，章节里的公式与推导可以直接对着 `+uav/` 里的实现读。

写作口径：正文讲原理，代码块以本仓库的 API 为准。三个构型的参数在 `+uav/+cfg/quad.m`、`fw.m`、`tilt.m`，控制器在 `+uav/+ctrl/`，被控对象在 `+uav/+plant/`。文中的 `models/*.slx` 指本仓库生成的三个闭环模型。

## 目录

| 部分 | 内容 | 对应代码 |
| --- | --- | --- |
| [00 导读与学习路线](00-导读与学习路线.md) | 学习顺序、每周任务、常见误区 | — |
| [01 基础概念](01-基础概念/) | 动力学是什么、Simulink 平台、MATLAB 协同、坐标系、构型分类 | `docs/architecture.md` |
| [02 动力学建模核心](02-动力学建模核心/) | 刚体六自由度、欧拉角与四元数、牛顿-欧拉推导、旋翼气动、电机、线性化 | `+uav/+plant/`、`+uav/+ctrl/hover_model.m` |
| [03 Simulink 建模实战](03-Simulink建模实战/) | 建模操作、第一个 6DOF、子系统封装、Simscape、Aerospace Blockset、S-Function | `scripts/build_*_model.m`、`+uav/+model/` |
| [04 控制系统设计](04-控制系统设计/) | PID、级联姿态、LQR/LQI、MPC、滑模、容错 | `+uav/+ctrl/cascade.m`、`lqr_ctrl.m`、`mpc_ctrl.m`、`tilt_alloc.m` |
| [05 状态估计与传感器](05-状态估计与传感器/) | IMU/GPS/气压计、EKF、VIO | 未实现，属于扩展点 |
| [06 高级仿真专题](06-高级仿真专题/) | 固定翼、VTOL、风扰、编队、eVTOL | `+uav/+plant/fw_aero.m`、`+uav/+ctrl/tilt_cascade.m` |
| [07 前沿技术与创新](07-前沿技术与创新/) | 强化学习辅助、分数阶 PID、数字孪生、AI 自适应、Simulink 与 ROS | 未实现，属于阅读材料 |
| [08 仿真与实飞衔接](08-仿真与实飞衔接/) | SIL、HIL、PX4 接口、验证流程 | `docs/codegen.md`、`c/` |
| [09 开源项目精读](09-开源项目精读/) | Quad-Sim、LADAC、MathWorks 官方模型、Flight-Dynamics-Control-UAVs、多机平台 | — |
| [10 论文导读](10-论文导读与前沿综述/) | 动力学、控制、状态估计、仿真平台的经典论文 | — |
| [11 PX4 飞控对接](11-PX4飞控系统对接/) | PX4 架构、控制链路、SITL、模块开发、日志调参 | — |
| [思维导图](mindmaps/) | 学习路线、动力学树、控制算法图、工具箱生态 | — |

## 和代码一起读的顺序

如果目标是把仓库跑通，先看 [README](../README.md) 的快速开始，再回来读 02 与 04 两章：02 解释 `rigid_body.m` 为什么这样写，04 解释 `cascade.m` 的姿态反解为什么用 `R_des` 而不是直接反三角函数。

如果目标是理解某个具体数值——比如悬停转速为什么是 495 rad/s、配平迎角为什么 2.2°——直接看 `docs/architecture.md` 与 `docs/fixedwing.md`，那里有逐步推导。

## 与原始资料库的差别

- 代码示例改成调用 `+uav` 包函数，不再依赖 `init_project` 之类的脚本注入。
- 模型文件名统一为 `models/quad_closed_loop.slx`、`fw_closed_loop.slx`、`tilt_closed_loop.slx`。
- 删掉了旧版仓库的目录截图与"复制到某文件夹"的说明，改为本仓库路径。
- 原资料库的 `references/`（ROS 包、launch 文件、论文清单）没有并入，需要时到原仓库取。
