function [Ad, Bd, Cd, Dd] = hover_model(p, Ts)
%HOVER_MODEL  悬停点线性化模型，连续方程离散化到 Ts。
%   状态 12 维：[pn;pe;pd; u;v;w; phi;theta;psi; p_;q_;r_]
%   输入 4 维：[T; Mx; My; Mz]
%   角度为小角度，姿态角速度直接等于欧拉角导数；平飞时 T=mg。
%
%   推导：机体系 z 轴在 NED 下的方向约为 [sin(theta); -cos(theta)*sin(phi); cos(theta)*cos(phi)]，
%   推力沿 -z 轴，于是加速度为 T/m*[-sin(theta); sin(phi); -cos(theta)] + [0;0;g]。
%   小角度下 dVn = -g*theta、dVe = +g*phi：抬头（theta>0）推力后倾，向北减速。
%   对推力取 T = mg + dT，则 dVz = -dT/m。

A = zeros(12);
A(1:3, 4:6) = eye(3);
A(4:6, 7:9) = [0, -p.g, 0; p.g, 0, 0; 0, 0, 0];
A(7:9, 10:12) = eye(3);

B = zeros(12, 4);
B(6, 1) = -1/p.mass;
B(10:12, 2:4) = inv(p.I);

sys = ss(A, B, eye(12), zeros(12, 4));
sd  = c2d(sys, Ts, 'zoh');
Ad  = sd.A;
Bd  = sd.B;
Cd  = sd.C;
Dd  = sd.D;
end
