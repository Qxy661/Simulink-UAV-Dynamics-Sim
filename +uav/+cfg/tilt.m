function p = tilt()
%TILT  倾转四旋翼（tilt-rotor）eVTOL 参数。
%   4 副旋翼装在机翼上，短舱绕机体 y 轴转动：beta=0 推力朝上，beta=90 度朝前。
%   8 kg 级、机翼 S=0.9 m^2。气动系数与固定翼同一套线性模型，量级合理但没有风洞标定。
%
%   坐标沿用全仓约定：世界 NED，机体 FRD，x 前 y 右 z 下。

p.mass   = 8.0;                 % kg
p.g      = 9.81;                % m/s^2
p.rho    = 1.225;               % kg/m^3，海平面

% 机翼
p.S      = 1.60;                % m^2，参考面积
p.b      = 3.60;                % m，翼展
p.c      = 0.34;                % m，平均气动弦长
p.I      = diag([0.90; 1.20; 1.60]);   % kg*m^2，x/y/z

% 升力：CL = CL0 + CLa*alpha + CLq*q*c/(2V) + CLde*de
p.CL0    = 0.45;
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

% 推进：倾转旋翼，机身 x 轴方向没有单独推力桨
p.Tmax   = 0.0;                 % N，保留字段以复用 fw_aero
p.tauT   = 0.15;                % s

% 舵面限幅与时间常数
p.deMax  = 25*pi/180;           % rad
p.daMax  = 20*pi/180;
p.drMax  = 25*pi/180;
p.tauA   = 0.05;                % s

% 旋翼：T = kt*w^2，反扭矩 Q = kd*w^2
p.kt     = 2.6e-5;              % N/(rad/s)^2
p.kd     = 5.0e-7;              % N*m/(rad/s)^2
p.kq     = p.kd/p.kt;           % m，反扭矩折算到推力的等效力臂
p.wMin   = 0.0;                 % rad/s
p.wMax   = 1100.0;              % rad/s
p.tauM   = 0.020;               % s，电机响应
p.hoverW = sqrt(p.mass*p.g/(4*p.kt));   % rad/s，悬停转速约 869

% 旋翼安装位置（机体系，z=0）：前右、后右、后左、前左
p.rx     = [ 0.40; -0.40; -0.40;  0.40];   % m
p.ry     = [ 0.90;  0.90; -0.90; -0.90];   % m
p.spin   = [ 1; -1; 1; -1];                % +1 逆时针（从上往下看）
p.aY     = 0.90;                % m，U2 的等效横向力臂
p.aX     = 0.40;                % m，U3 的等效纵向力臂

% 倾转短舱
p.betaMin = 0.0;                % rad，悬停时短舱竖直（5 度会给悬停带来 6.8 N 的前向分力）
p.betaMax = 90*pi/180;          % rad，巡航（纯前向推力，机翼承担全部升力）
p.tauBeta = 0.15;               % s，短舱一阶滞后
p.betaRate = 30*pi/180;         % rad/s，短舱速率限制

% 调度：空速 3 m/s 以下纯旋翼，14 m/s 以上纯机翼
% 14 是本配置的稳定上界，再高就掉高失速（见 docs/tiltrotor.md 包线表）
p.V1     = 3.0;                 % m/s
p.V2     = 14.0;                % m/s

% 基准
p.htrim  = 20.0;                % m，悬停高度
p.Vtrim  = 22.0;                % m/s，巡航速度
end
