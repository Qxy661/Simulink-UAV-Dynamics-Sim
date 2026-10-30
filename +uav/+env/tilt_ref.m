function r = tilt_ref(t)
%TILT_REF  倾转四旋翼的任务参考（解析式，便于在图表里内联）。
%   0–5 s    20 m 悬停
%   5–20 s   沿正北加速到 10 m/s，高度保持 20 m
%   20–45 s  巡航
%   45–60 s  减速到悬停
%   60 s 后  20 m 悬停
%
%   高度在过渡段保持不变。早期版本让加速与爬升同时进行（20→40 m），
%   纵向需求把推力方向拉得几乎竖直，机体被迫低头不够，实测卡在 32°
%   爬升、机体反而抬头 7.6°，速度停在 4 m/s。过渡段先做平飞加速，
%   把纵向与横向耦合分开，等机翼接管后再做高度机动。
%
%   返回 struct：p（NED 位置）、v、a、psi、h、V。
%   加速度用余弦包络的一阶导数，速度与位置用其解析积分，避免数值积分误差。

Vmax = 22.0;                    % m/s，巡航速度；β 到 90°（见 docs/tiltrotor.md）
Vmin = 4.0;                     % m/s，减速后保留的低速巡航
Ta   = 15.0;                    % s，加/减速时长
t1   = 5.0;                     % 加速开始
t2   = t1 + Ta;                 % 20
t3   = 45.0;                    % 减速开始
t4   = t3 + Ta;                 % 60
h0   = 20.0;                    % m，全程高度

pAcc = Vmax*Ta/2;               % 加速段位移 132 m
pCru = pAcc + Vmax*(t3 - t2);   % 巡航结束位置

% ---- 速度与加速度 ----
if t <= t1
    V = 0; aN = 0; pN = 0;
elseif t <= t2
    tau = t - t1;
    V  = Vmax/2*(1 - cos(pi*tau/Ta));
    aN = Vmax/2*(pi/Ta)*sin(pi*tau/Ta);
    pN = Vmax/2*(tau - (Ta/pi)*sin(pi*tau/Ta));
else
    % 加速段结束后保持 10 m/s 巡航。任务只覆盖"悬停 → 转捩 → 巡航"，
    % 不做减速回悬停：减速段的姿态混合会削弱倾角需求，实测位置滞后
    % 25 m 且 β 收回到 8° 后机体继续漂移，见 docs/tiltrotor.md 未解决项。
    V = Vmax; aN = 0; pN = pAcc + Vmax*(t - t2);
end

r.p   = [pN; 0; -h0];
r.v   = [V; 0; 0];
r.a   = [aN; 0; 0];
r.psi = 0;
r.h   = h0;
r.V   = V;
end
