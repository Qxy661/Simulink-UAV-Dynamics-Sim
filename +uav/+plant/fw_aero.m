function [Fb, Mb] = fw_aero(Vb, w, act, p)
%FW_AERO  固定翼气动力与力矩。
%   Vb  : 机体系速度 [u; v; w] (m/s)
%   w   : 机体系角速度 [p; q; r] (rad/s)
%   act : 执行机构 [throttle; de; da; dr]
%         throttle 0..1，舵面 rad，正 de 为抬头方向的定义见 fw_aero 文档
%   Fb  : 机体系力 (N)，包含推力、升力、阻力、侧力
%   Mb  : 机体力矩 (N*m)
%
%   气动角：alpha = atan2(w, u)，beta = asin(v/V)。
%   升力沿稳定性 z 轴负方向（机腹向上为负），阻力沿 -x，再按 alpha 转到机体轴。

u = Vb(1); v = Vb(2); wb = Vb(3);
p_ = w(1); q = w(2); r = w(3);
thr = act(1); de = act(2); da = act(3); dr = act(4);

V = norm(Vb);
Ve = max(V, 1.0);                       % 低速时避免除零，只在悬停/起飞段生效

alpha = atan2(wb, u);
beta  = asin(max(-1, min(1, v / Ve)));
qbar  = 0.5 * p.rho * V^2;

CL = p.CL0 + p.CLa*alpha + p.CLq*q*p.c/(2*Ve) + p.CLde*de;
CD = p.CD0 + p.kind*CL^2;
CY = p.CYb*beta + p.CYdr*dr;

D = qbar * p.S * CD;
L = qbar * p.S * CL;
Y = qbar * p.S * CY;

% 稳定性轴到机体轴
Xa = -D*cos(alpha) + L*sin(alpha);
Za = -D*sin(alpha) - L*cos(alpha);

T = thr * p.Tmax;

Fb = [Xa + T; Y; Za];

Mb = [ qbar*p.S*p.b*(p.Clb*beta + p.Clp*p_*p.b/(2*Ve) + p.Clr*r*p.b/(2*Ve) + p.Clda*da);
       qbar*p.S*p.c*(p.Cm0 + p.Cma*alpha + p.Cmq*q*p.c/(2*Ve) + p.Cmde*de);
       qbar*p.S*p.b*(p.Cnb*beta + p.Cnp*p_*p.b/(2*Ve) + p.Cnr*r*p.b/(2*Ve) + p.Cndr*dr) ];
end
