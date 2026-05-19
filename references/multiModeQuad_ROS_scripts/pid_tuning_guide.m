%% pid_tuning_guide.m - multiModeQuad_ROS PID 参数整定指南
%
% 本脚本提供系统化的 PID 参数整定方法和自动化调参工具。
% 可在 MATLAB 中运行，或通过修改 multiModeQuad_ROS_data.cpp 中的值在 ROS 中生效。
%
% 级联PID架构:
%   速度指令 → [速度PID] → 姿态指令 → [角速率PID] → 混控器 → 电机 → 动态
%      外环 (10Hz)               内环 (200Hz)
%
% 整定原则:
%   1. 先内环后外环 (先锁定角速率环, 再整定速度环)
%   2. 先P后I再D (依次增加)
%   3. 每个环从低增益开始, 缓慢增加直到出现振荡, 然后回退30%

%% ==================== 1. 内环整定 (角速率环) ====================

% 角速率环是飞行稳定的关键。它直接控制电机混控输出。
%
% 默认参数 (从原项目提取):
fprintf('========== 角速率 PID (内环) ==========\n');
fprintf('Roll:  P=%.1f, I=%.3f, D=%.3f, N=%.0f\n', 0.3, 0.01, 0.05, 300);
fprintf('Pitch: P=%.1f, I=%.3f, D=%.3f, N=%.0f\n', 0.3, 0.01, 0.05, 300);
fprintf('Yaw:   P=%.1f, I=%.3f, D=%.3f, N=%.0f\n', 0.6, 0.01, 0.05, 300);
fprintf('\n');

% 整定步骤:
fprintf('整定步骤:\n');
fprintf('  1. 设置 P=0.1, I=0, D=0 (先只用比例)\n');
fprintf('  2. 逐渐增加 P 直到出现轻微角速率振荡\n');
fprintf('  3. 记录临界增益 P_crit, 取 P = 0.7 * P_crit\n');
fprintf('  4. 增加 I 以消除稳态误差 (从 0.5*P 开始)\n');
fprintf('  5. 如有过冲, 增加 D 抑制 (从 0.1*P 开始)\n');
fprintf('  6. Yaw 轴的 P 通常需要比 Roll/Pitch 大 1.5-2 倍\n');

% 推荐参数表:
fprintf('\n推荐参数 (经验值):\n');
fprintf('-----------------------------\n');
fprintf('| 轴    | P    | I    | D  |\n');
fprintf('|-------|------|------|----|\n');
fprintf('| Roll  | 0.4  | 0.02 | 0.08 |\n');
fprintf('| Pitch | 0.4  | 0.02 | 0.08 |\n');
fprintf('| Yaw   | 0.8  | 0.02 | 0.06 |\n');
fprintf('-----------------------------\n');

%% ==================== 2. 外环整定 (速度环) ====================

fprintf('\n========== 速度 PID (外环) ==========\n');
fprintf('Vel X: P=%.1f, I=%.3f, D=%.3f, N=%.0f\n', 10.0, 0.02, 0.01, 100);
fprintf('Vel Y: P=%.1f, I=%.3f, D=%.3f, N=%.0f\n', 10.0, 0.02, 0.01, 100);
fprintf('Vel Z: P=%.1f, I=%.3f, D=%.3f, N=%.0f\n', 3.0,  0.2,  2.0,  100);
fprintf('\n');

fprintf('整定步骤:\n');
fprintf('  1. 先确保角速率环已整定好\n');
fprintf('  2. 设置速度环 P=2, I=0, D=0\n');
fprintf('  3. 发送 1m/s 阶跃速度指令, 观察响应\n');
fprintf('  4. 逐渐增加 P 直到响应快速且无过冲\n');
fprintf('  5. 增加 I 以消除稳态误差\n');
fprintf('  6. 如果速度超调量大, 增加 D\n');
fprintf('\n');
fprintf('注意: Vel Z (高度) 的 P 值通常比水平通道小,');
fprintf(' 因为重力补偿已经承担了大部分负载\n');

%% ==================== 3. PID 参数影响速查 ====================

fprintf('\n========== PID 参数影响速查 ==========\n');
fprintf('参数 | 上升时间 | 超调量 | 稳态误差 | 稳定性\n');
fprintf('P ↑ |    ↓     |   ↑   |    ↓    |   ↓\n');
fprintf('I ↑ |    ↓     |   ↑   |    ↓↓   |   ↓\n');
fprintf('D ↑ |    -     |   ↓   |    -    |   ↑\n');
fprintf('N ↑ |    -     |   ↓   |    -    |   ↑ (抗噪↓)\n');

%% ==================== 4. 自动调参函数 ====================

% 以下函数可自动计算 Ziegler-Nichols 参数
fprintf('\n========== Ziegler-Nichols 整定 ==========\n');

function [Kp, Ki, Kd] = zn_pid(Ku, Tu, type)
    % Ziegler-Nichols 整定公式
    % 输入: Ku - 临界增益, Tu - 临界周期
    %       type - 控制器类型 ('P', 'PI', 'PID')
    switch lower(type)
        case 'p'
            Kp = 0.5 * Ku; Ki = 0; Kd = 0;
        case 'pi'
            Kp = 0.45 * Ku; Ki = 0.54 * Ku / Tu; Kd = 0;
        case 'pid'
            Kp = 0.6 * Ku; Ki = 1.2 * Ku / Tu; Kd = 0.075 * Ku * Tu;
        otherwise
            error('Unknown type: %s', type);
    end
end

fprintf('使用 ZN 公式:\n');
fprintf('  Kp = 0.6*Ku, Ki = 1.2*Ku/Tu, Kd = 0.075*Ku*Tu\n');
fprintf('  其中 Ku=临界增益, Tu=临界周期\n');

%% ==================== 5. PID 调试脚本 ====================

fprintf('\n========== 在线调试指令 (ROS) ==========\n');
fprintf('在 multiModeQuad_ROS_data.cpp 中修改对应参数:\n');
fprintf('  // 修改 Roll 角速率 P 增益\n');
fprintf('  // Mask Parameter: PIDangularroll_P\n');
fprintf('  // Referenced by: <S376>/Proportional Gain\n');
fprintf('  0.4,  // 原 0.3, 改为此值后重新编译\n');
fprintf('\n');
fprintf('动态调参 (如果启用了动态重配置):\n');
fprintf('  rosrun rqt_reconfigure rqt_reconfigure\n');
fprintf('\n验证:\n');
fprintf('  rostopic echo /quad_1/pose  # 查看位姿\n');
fprintf('  rostopic echo /quad_1/imuData  # 查看IMU数据\n');

%% ==================== 6. 仿真验证脚本 ====================

fprintf('\n========== 仿真测试命令 (MATLAB) ==========\n');
fprintf('1. 运行初始化:  quad_params\n');
fprintf('2. 打开模型:   open_system(''multiModeQuad_ROS'')\n');
fprintf('3. 修改PID参数: 双击对应 PID 模块修改\n');
fprintf('4. 运行仿真:   sim(''multiModeQuad_ROS'')\n');
fprintf('5. 查看结果:   scope 模块或 simlog 变量\n');
fprintf('==========================================\n');
