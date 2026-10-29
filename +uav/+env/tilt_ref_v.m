function r = tilt_ref_v(t, Vmax)
%TILT_REF_V  参数化版本的倾转四旋翼任务参考，用于转捩包线扫描。
%   r = uav.env.tilt_ref_v(t, Vmax)
%   与 uav.env.tilt_ref 相同的形状，只是把巡航速度做成参数：
%   0–5 s 20 m 悬停，5–20 s 加速到 Vmax，20–45 s 巡航，45–60 s 减速回悬停。

Ta = 15.0;
t1 = 5.0;
t2 = t1 + Ta;
t3 = 45.0;
t4 = t3 + Ta;
h0 = 20.0;

pAcc = Vmax*Ta/2;
pCru = pAcc + Vmax*(t3 - t2);

if t <= t1
    V = 0; aN = 0; pN = 0;
elseif t <= t2
    tau = t - t1;
    V  = Vmax/2*(1 - cos(pi*tau/Ta));
    aN = Vmax/2*(pi/Ta)*sin(pi*tau/Ta);
    pN = Vmax/2*(tau - (Ta/pi)*sin(pi*tau/Ta));
elseif t <= t3
    V = Vmax; aN = 0; pN = pAcc + Vmax*(t - t2);
elseif t <= t4
    tau = t - t3;
    V  = Vmax/2*(1 + cos(pi*tau/Ta));
    aN = -Vmax/2*(pi/Ta)*sin(pi*tau/Ta);
    pN = pAcc + Vmax*(t3 - t2) + Vmax/2*(tau + (Ta/pi)*sin(pi*tau/Ta));
else
    V = 0; aN = 0; pN = pCru + Vmax*Ta/2;
end

r.p   = [pN; 0; -h0];
r.v   = [V; 0; 0];
r.a   = [aN; 0; 0];
r.psi = 0;
r.h   = h0;
r.V   = V;
end
