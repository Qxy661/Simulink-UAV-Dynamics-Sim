function r = tilt_ref_bt(t, Vmax, hTop, hBot, Tdec)
%TILT_REF_BT  带高度下沉的往返任务参考（减速段允许掉高换速度）。
%   r = uav.env.tilt_ref_bt(t, Vmax, hTop, hBot)
%
%   0–5 s      hTop 悬停
%   5–20 s     加速到 Vmax，保持 hTop
%   20–40 s    hTop 巡航
%   40–60 s    减速到 0，同时从 hTop 平滑降到 hBot
%   60 s 之后  hBot 悬停
%
%   与 tilt_ref_v 的区别只在减速段：定高减速时，减速只能靠抬头，抬头必然
%   多出升力（本机翼 13 m/s、14° 迎角就有 310 N），多出的升力没有去处，
%   于是被顶高十几米（见 research/REPORT_r2_backtransition.md）。这里让参考
%   高度同步下降，把多余的升力变成掉高，而不是变成爬升。
%
%   输入
%     t     时间 (s)，可为向量
%     Vmax  巡航速度 (m/s)，默认 22
%     hTop  巡航高度 (m)，默认 20
%     hBot  减速结束高度 (m)，默认 8
%
%   输出 struct：p(3x1, NED)、v、a、psi、h、V

if nargin < 2 || isempty(Vmax), Vmax = 22; end
if nargin < 3 || isempty(hTop), hTop = 20; end
if nargin < 4 || isempty(hBot), hBot = 8; end
if nargin < 5 || isempty(Tdec), Tdec = 15.0; end

Ta  = 15.0;      % 加速时长
Td  = Tdec;      % 减速时长（越大减速度越小）
t1  = 5.0;
t2  = t1 + Ta;
t3  = 40.0;
t4  = t3 + Td;
dh  = hTop - hBot;

pAcc = Vmax*Ta/2;
pCru = pAcc + Vmax*(t3 - t2);

n = numel(t);
r.p = zeros(3, n); r.v = zeros(3, n); r.a = zeros(3, n);
r.psi = zeros(1, n); r.h = zeros(1, n); r.V = zeros(1, n);

for k = 1:n
    tk = t(k);
    if tk <= t1
        V = 0; aN = 0; pN = 0; h = hTop; hd = 0; ha = 0;
    elseif tk <= t2
        tau = tk - t1;
        V  = Vmax/2*(1 - cos(pi*tau/Ta));
        aN = Vmax/2*(pi/Ta)*sin(pi*tau/Ta);
        pN = Vmax/2*(tau - (Ta/pi)*sin(pi*tau/Ta));
        h = hTop; hd = 0; ha = 0;
    elseif tk <= t3
        V = Vmax; aN = 0; pN = pAcc + Vmax*(tk - t2);
        h = hTop; hd = 0; ha = 0;
    elseif tk <= t4
        tau = tk - t3;
        V  = Vmax/2*(1 + cos(pi*tau/Td));
        aN = -Vmax/2*(pi/Td)*sin(pi*tau/Td);
        pN = pAcc + Vmax*(t3 - t2) + Vmax/2*(tau + (Td/pi)*sin(pi*tau/Td));
        s      = (1 - cos(pi*tau/Td))/2;          % 0->1 平滑
        ds     = (pi/(2*Td))*sin(pi*tau/Td);
        dds    = (pi/(2*Td))^2*cos(pi*tau/Td);
        h  = hTop - dh*s;
        hd = -dh*ds;
        ha = -dh*dds;
    else
        V = 0; aN = 0; pN = pCru + Vmax*Td/2;
        h = hBot; hd = 0; ha = 0;
    end
    r.p(:, k) = [pN; 0; -h];
    r.v(:, k) = [V; 0; -hd];       % NED：向上为 -z
    r.a(:, k) = [aN; 0; -ha];
    r.psi(k)  = 0;
    r.h(k)    = h;
    r.V(k)    = V;
end
end
