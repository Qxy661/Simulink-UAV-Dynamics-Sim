function t = fw_trim(p, V)
%FW_TRIM  定直平飞配平。
%   给定空速 V 与高度无关的海平面密度，解迎角 alpha、升降舵 de 与油门，
%   使机体轴上的合力与俯仰力矩为零。
%
%   平飞时航迹角为零、俯仰角等于迎角，机体 x 轴上还有重力的前向分量，
%   因此推力要抵消阻力前向分量、升力前向分量与重力分量三项。
%   升力需求与阻力耦合，用几轮不动点迭代即可收敛（阻力是小量）。
%
%   结果一次构造为结构体：代码生成要求先算完局部量再建结构体，
%   分步写 t.CD 再读它算 t.D 会触发字段锁定错误。

if nargin < 2
    V = p.Vtrim;
end

qbar = 0.5 * p.rho * V^2;
W    = p.mass * p.g;

CL = W / (qbar * p.S);          % 初值：升力等于重力
for it = 1:6
    CD = p.CD0 + p.kind*CL^2;
    D  = qbar * p.S * CD;
    % 解 [CLa CLde; Cma Cmde] * [alpha; de] = [CL - CL0; -Cm0]
    A = [p.CLa, p.CLde; p.Cma, p.Cmde];
    x = A \ [CL - p.CL0; -p.Cm0];
    a = x(1);
    % 由 z 轴力平衡更新升力需求：L*cos(a) + D*sin(a) = W*cos(a)
    CL = (W*cos(a) - D*sin(a)) / (qbar * p.S * cos(a));
end

CD  = p.CD0 + p.kind*CL^2;
D   = qbar * p.S * CD;
L   = qbar * p.S * CL;
thr = (D*cos(a) - L*sin(a) + W*sin(a)) / p.Tmax;

t = struct('alpha', a, 'de', x(2), 'CL', CL, 'CD', CD, 'D', D, 'L', L, ...
           'thr', thr, 'V', V, 'LD', CL/CD, 'qbar', qbar);
end
