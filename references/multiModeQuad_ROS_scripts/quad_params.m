%% quad_params.m - multiModeQuad_ROS 物理参数与控制器参数
% 使用说明：在 MATLAB 中运行本脚本初始化工作空间，然后可运行 Simulink 模型或独立仿真
% 所有参数与 SLX 模型中的值保持完全一致
%
% 最后更新: 2026-05-17
% 对应模型: multiModeQuad_ROS.slx (R2024a)

clear; clc;

%% ===================== 1. 物理参数 =====================

% --- 四旋翼参数 ---
params.mass = 2.0;                  % 总质量 [kg]
params.g = 9.81;                    % 重力加速度 [m/s^2]
params.mg = params.mass * params.g; % 重力 [N]

% 惯量矩阵 [kg*m^2]
params.Jxx = 0.015;
params.Jyy = 0.015;
params.Jzz = 0.03;
params.I = [params.Jxx, 0, 0;
            0, params.Jyy, 0;
            0, 0, params.Jzz];

% --- 几何参数 ---
params.arm_length = 0.15;           % 机臂长度 [m]
params.motor_length = 0.05;         % 电机力臂 [m] (用于电机惯性计算)
params.servo_length = 0.02;         % 伺服机构长度 [m]

% --- 电机参数 ---
% 电机一阶传递函数: G(s) = K / (tau*s + 1)
% 从生成的代码: TransferFcn A = -24.3902 = -1/tau, C = 24.3902 = 1/tau
% 所以 tau = 1/24.3902 ≈ 0.041s, K = 1
params.motor_tau = 1/24.390243902439025;  % 电机时间常数 [s]
params.motor_K = 1;                        % 电机增益

% 电机一阶TF系数 (连续域)
% G(s) = params.motor_K / (params.motor_tau * s + 1)
params.motor_num = params.motor_K;
params.motor_den = [params.motor_tau, 1];

% --- 悬停参数 ---
params.hover_throttle = 3.13;           % 悬停油门值
params.hover_thrust = params.mg;         % 悬停所需推力
params.thrust_per_throttle = params.mg / params.hover_throttle;  % 推力/油门比

% --- 电机混控矩阵 (Standard X-configuration) ---
% 输入: [u1(roll), u2(pitch), u3(yaw), u4(thrust)]^T
% 输出: [Omega1, Omega2, Omega3, Omega4]^T (各电机转速指令)
%
% 电机布局:
%    T2(Front-Left)   T4(Front-Right)
%          \              /
%           \            /
%            -----------
%           /            \
%          /              \
%    T1(Rear-Left)    T3(Rear-Right)
%
% Mixer:
%   Omega1 = u4 - u1 - u2 - u3   (Rear-Left)
%   Omega2 = u4 - u1 + u2 + u3   (Front-Left)
%   Omega3 = u4 + u1 - u2 + u3   (Rear-Right)
%   Omega4 = u4 + u1 + u2 - u3   (Front-Right)
params.mixer_matrix = [ -1, -1, -1,  1;
                        -1,  1,  1,  1;
                         1, -1,  1,  1;
                         1,  1, -1,  1 ];

% 推力→机体力和力矩映射 (Chart 93)
% T_B = [0; 0; -(T1+T2+T3+T4)]   (NED: 正推力向下)
% tau_B = [(T3+T4-T1-T2)*arm_length;  % Roll
%          (T2+T4-T1-T3)*arm_length;  % Pitch
%          (T2+T3-T1-T4)*0.2*0.2];    % Yaw (drag coeff)
params.thrust_to_force = [0, 0, 0, 0;
                          0, 0, 0, 0;
                         -1,-1,-1,-1];
params.force_to_torque = [-1, -1,  1,  1;   % Roll arm mapping
                          -1,  1, -1,  1;   % Pitch arm mapping
                          -1,  1,  1, -1];  % Yaw direction mapping
params.arm_length_roll  = params.arm_length;   % Roll 力臂 [m]
params.arm_length_pitch = params.arm_length;   % Pitch 力臂 [m]
params.yaw_drag_coeff   = 0.2;                 % 偏航阻力系数

%% ===================== 2. 控制器参数 =====================

% --- 外环: 速度控制器 (PID) ---
% 参考: PIDVelocityx/y/z 块
params.vel_x = struct('P', 10.0, 'I', 0.02, 'D', 0.01, 'N', 100, ...
    'upper_limit', 50, 'lower_limit', -50);
params.vel_y = struct('P', 10.0, 'I', 0.02, 'D', 0.01, 'N', 100, ...
    'upper_limit', 50, 'lower_limit', -50);
params.vel_z = struct('P', 3.0,  'I', 0.2,  'D', 2.0,  'N', 100, ...
    'upper_limit', inf, 'lower_limit', -inf);

% --- 内环: 角速率控制器 (PID) ---
% 参考: PIDangularroll/pitch/yaw 块
params.rate_roll  = struct('P', 0.3, 'I', 0.01, 'D', 0.05, 'N', 300);
params.rate_pitch = struct('P', 0.3, 'I', 0.01, 'D', 0.05, 'N', 300);
params.rate_yaw   = struct('P', 0.6, 'I', 0.01, 'D', 0.05, 'N', 300);

% --- 角速率设定值限幅 ---
params.max_angular_rate = [20, 20, 20];  % [roll, pitch, yaw] [rad/s]

%% ===================== 3. 仿真设置 =====================

params.dt = 0.005;           % 仿真步长 [s] (与 SLX 一致: ODE4, 200Hz)
params.t_end = 20;           % 默认仿真时长 [s]

% 初始状态 [xe, ye, ze, ub, vb, wb, phi, theta, psi, p, q, r]
params.x0 = [0; 0; 0;        % NED 位置 [m]
             0; 0; 0;        % 体轴速度 [m/s]
             0; 0; 0;        % 欧拉角 [rad]
             0; 0; 0];       % 角速率 [rad/s]

% 坐标系统
params.coord_frame = 'NED';   % NED (North-East-Down)
params.velocity_frame = 'body'; % 速度指令在体轴系

%% ===================== 4. 输出参数 =====================

fprintf('========================================\n');
fprintf(' multiModeQuad_ROS 参数初始化完成\n');
fprintf('========================================\n');
fprintf(' 质量:        %.1f kg\n', params.mass);
fprintf(' 惯量:        [%.3f, %.3f, %.3f]\n', params.Jxx, params.Jyy, params.Jzz);
fprintf(' 机臂长:      %.2f m\n', params.arm_length);
fprintf(' 悬停油门:    %.2f\n', params.hover_throttle);
fprintf(' 仿真步长:    %.3f s (%.0f Hz)\n', params.dt, 1/params.dt);
fprintf(' 求解器:      ODE4 (4阶龙格-库塔)\n');
fprintf(' 坐标系:      %s\n', params.coord_frame);
fprintf(' 速度环 P:    [%.1f, %.1f, %.1f]\n', ...
    params.vel_x.P, params.vel_y.P, params.vel_z.P);
fprintf(' 角速率环 P:  [%.1f, %.1f, %.1f]\n', ...
    params.rate_roll.P, params.rate_pitch.P, params.rate_yaw.P);
fprintf('========================================\n');
