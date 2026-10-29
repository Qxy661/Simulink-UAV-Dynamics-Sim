function p = fw()
%FW  固定翼小型无人机参数。
%   5 kg 电动无人机，平飞 18 m/s。气动系数是量级合理的估计值，
%   没有风洞标定；用途是控制系统设计，不是性能预测。
%
%   坐标与四旋翼一致：世界 NED，机体 FRD，x 前 y 右 z 下。

p.mass   = 5.0;                 % kg
p.g      = 9.81;                % m/s^2
p.rho    = 1.225;               % kg/m^3，海平面

% 机翼
p.S      = 0.55;                % m^2，参考面积
p.b      = 2.50;                % m，翼展
p.c      = 0.22;                % m，平均气动弦长
p.I      = diag([0.45; 0.60; 0.85]);   % kg*m^2，x/y/z

% 升力：CL = CL0 + CLa*alpha + CLq*q*c/(2V) + CLde*de
p.CL0    = 0.25;
p.CLa    = 5.00;                % 1/rad
p.CLq    = 3.90;
p.CLde   = 0.40;

% 阻力：CD = CD0 + k*CL^2
p.CD0    = 0.025;
p.kind   = 0.045;

% 侧力：CY = CYb*beta + CYdr*dr
p.CYb    = -0.60;
p.CYdr   = 0.19;

% 俯仰力矩：Cm = Cm0 + Cma*alpha + Cmq*q*c/(2V) + Cmde*de
p.Cm0    = 0.05;
p.Cma    = -0.80;
p.Cmq    = -12.0;
p.Cmde   = -1.20;

% 滚转力矩：Cl = Clb*beta + Clp*p*b/(2V) + Clr*r*b/(2V) + Clda*da
p.Clb    = -0.090;
p.Clp    = -0.50;
p.Clr    = 0.080;
p.Clda   = 0.180;

% 偏航力矩：Cn = Cnb*beta + Cnp*p*b/(2V) + Cnr*r*b/(2V) + Cndr*dr
p.Cnb    = 0.060;
p.Cnp    = -0.030;
p.Cnr    = -0.120;
p.Cndr   = 0.090;

% 推进：推力 = throttle * Tmax，沿机体 x 轴
p.Tmax   = 40.0;                % N
p.tauT   = 0.15;                % s，电机+螺旋桨响应

% 舵面限幅与速率
p.deMax  = 25*pi/180;           % rad
p.daMax  = 20*pi/180;
p.drMax  = 25*pi/180;
p.tauA   = 0.05;                % s，舵机时间常数

% 平飞基准（供配平与控制器前馈用）
p.Vtrim  = 18.0;                % m/s
p.htrim  = 100.0;               % m
end
