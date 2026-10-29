function p = quad()
%QUAD  四旋翼机体与环境参数。
%   坐标系：世界 NED（x 北、y 东、z 地），机体 FRD（x 前、y 右、z 地）。
%   力臂按 X 布局投影 a = arm/sqrt(2)。
%   电机编号：1 右前、2 右后、3 左后、4 左前；1/3 逆时针，2/4 顺时针。

p.mass = 1.5;                       % kg
p.I    = diag([0.0035; 0.0035; 0.0065]);   % kg*m^2
p.g    = 9.81;                      % m/s^2
p.arm  = 0.225;                     % 电机到质心距离 m
p.a    = p.arm/sqrt(2);             % 力臂在 x/y 轴的投影 m

p.kt   = 1.5e-5;                    % 推力系数 N/(rad/s)^2
p.kd   = 2.0e-7;                    % 反扭矩系数 N*m/(rad/s)^2
p.kq   = p.kd/p.kt;                 % 反扭矩与推力之比 m

p.tauM = 0.02;                      % 电机一阶时间常数 s
p.wMin = 0;
p.wMax = 1200;                      % rad/s

p.drag = 0.08;                      % 线性阻力 N/(m/s)，仅用于低速段

% 混控矩阵 B：电机转速平方 -> [T; Mx; My; Mz]
kt = p.kt; kd = p.kd; a = p.a;
p.B = [ kt    kt    kt    kt;
       -kt*a -kt*a  kt*a  kt*a;
        kt*a -kt*a -kt*a  kt*a;
        kd   -kd    kd   -kd ];
p.Binv = inv(p.B);
p.hoverW = sqrt(p.mass*p.g/(4*p.kt));
end
