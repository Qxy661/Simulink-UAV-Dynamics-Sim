function r = tilt_ref_energy(t, Vmax, h0, eta)
%TILT_REF_ENERGY  倾转四旋翼的能量一致往返任务
%
%   R = uav.env.tilt_ref_energy(T, VMAX, H0, ETA) 在时刻 T 返回参考
%   结构体 R，字段 p/v/a/psi/h/V。任务前段与 uav.env.tilt_ref 相同：
%   悬停 5 s，第 5..20 s 余弦加速到 VMAX，之后巡航 10 s。区别在减速段：
%
%   平飞减速时减速度只能来自阻力与抬头，抬头的升力必须有地方去，
%   否则机体爬升（见 research/REPORT_r3_energy.md）。这里让高度跟着
%   能量走：
%
%       h(t) = H0 + ETA*(VMAX^2 - V(t)^2)/(2*g)
%
%   即减速过程把动能按效率 ETA 换成高度，减速完成后悬停在更高的
%   高度上。ETA=0.7 与包线实测吻合。
%
%   参数
%     T     时间 (s)
%     VMAX  巡航速度 (m/s)
%     H0    起始高度 (m, 正高度)
%     ETA   动能转高度的效率 (0..1)，默认 0.7
%
%   输出 R.p 为 NED 位置（下为正），R.h 为正高度。

g = 9.81;
if nargin < 4 || isempty(eta)
    eta = 0.7;
end

tHover = 5;
tRamp = 15;
tCruise = 10;
t3 = tHover + tRamp + tCruise;   % 减速开始时刻
Td = 20;                          % 减速时长，保证减速度 <= 1.5 m/s^2

hEnd = h0 + eta * Vmax^2 / (2*g);

if t <= tHover
    V = 0; a = 0; x = 0; h = h0; hdot = 0;
elseif t <= t3
    u = (t - tHover) / tRamp;
    V = Vmax * (1 - cos(pi*u)) / 2;
    a = Vmax * pi / (2*tRamp) * sin(pi*u);
    x = Vmax * (t - tHover)/2 - Vmax * tRamp/(2*pi) * sin(pi*u);
    h = h0; hdot = 0;
elseif t <= t3 + Td
    u = (t - t3) / Td;
    V = Vmax * (1 + cos(pi*u)) / 2;
    a = -Vmax * pi / (2*Td) * sin(pi*u);
    xRamp = Vmax * tRamp / 2;
    xCruise = Vmax * (t3 - tHover - tRamp);
    xDecel = Vmax * (Td/2 * u + Td/(2*pi) * sin(pi*u));
    x = xRamp + xCruise + xDecel;
    h = h0 + eta * (Vmax^2 - V^2) / (2*g);
    hdot = -eta * V * a / g;
else
    V = 0; a = 0;
    xRamp = Vmax * tRamp / 2;
    xCruise = Vmax * (t3 - tHover - tRamp);
    x = xRamp + xCruise + Vmax * Td / 2;
    h = hEnd; hdot = 0;
end

r.p = [x; 0; -h];
r.v = [V; 0; -hdot];
r.a = [a; 0; eta * V * a / g];   % 由 hdot 求导得到
r.psi = 0;
r.h = h;
r.V = V;
end
