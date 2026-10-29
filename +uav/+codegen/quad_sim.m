function X = quad_sim(pTarget, psiTarget)
%QUAD_SIM  四旋翼闭环仿真主循环（C 代码生成入口）。
%   X = uav.codegen.quad_sim(pTarget, psiTarget)
%   与 uav.sim.run 的数值完全一致：RK4 @1 ms、控制器 100 Hz、
%   电机一阶模型精确离散、日志记录积分前的状态。
%   pTarget 为 3x1 NED 位置目标（t = 1 s 阶跃），psiTarget 为航向。
%   返回 1401x13 状态序列，采样间隔 10 ms，总时长 14 s。
%
%#codegen

p  = uav.cfg.quad();
c  = uav.cfg.ctrl();
p0 = [0; 0; -5];

dt      = 1e-3;
ctrlDiv = 10;
logDiv  = 10;
nStep   = 14000;
nLog    = nStep/logDiv;              % 0..13.99 s，每 10 ms 一个样本

X = zeros(nLog, 13);
x = [p0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
w = [p.hoverW; p.hoverW; p.hoverW; p.hoverW];
st = struct('ip', zeros(3, 1), 'ia', zeros(3, 1));
wcmd = w;
Tcmd = 0;
Mcmd = zeros(3, 1);
j    = 0;

for k = 1:nStep
    t = (k - 1)*dt;

    if mod(k - 1, ctrlDiv) == 0
        r = uav.env.step(t, 1.0, p0, pTarget, psiTarget);
        [Tcmd, Mcmd, st] = uav.ctrl.cascade(x, r, p, c, st, dt*ctrlDiv);
        [wcmd, sat] = uav.plant.mixer([Tcmd; Mcmd], p);   %#ok<ASGLU>
    end

    w = uav.plant.motor_step(w, wcmd, p, dt);

    k1 = deriv(x, w, p);
    k2 = deriv(x + 0.5*dt*k1, w, p);
    k3 = deriv(x + 0.5*dt*k2, w, p);
    k4 = deriv(x + dt*k3, w, p);

    if mod(k - 1, logDiv) == 0
        j = j + 1;
        X(j, :) = x';
    end

    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(7:10) = x(7:10)/norm(x(7:10));
end
end

function xd = deriv(x, w, p)
Vb = x(4:6);
[Fb, Mb] = uav.plant.quad_wrench(w, Vb, p);
xd = uav.plant.rigid_body(x, Fb, Mb, p);
end
