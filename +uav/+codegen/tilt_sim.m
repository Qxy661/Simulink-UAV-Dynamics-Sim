function X = tilt_sim()
%TILT_SIM  倾转四旋翼闭环仿真（C 代码生成入口）。
%   X = uav.codegen.tilt_sim()
%   与 uav.sim.run_tilt 的数值完全一致：RK4 @1 ms、控制器 100 Hz、
%   短舱/转速/舵面一阶滞后在导数里精确积分、记录积分前的状态。
%   任务：20 m 悬停 5 s，随后沿正北加速到 10 m/s 并进入过渡段。
%   返回 4500 x 21 状态序列，采样间隔 10 ms，总时长 45 s。
%
%#codegen

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();

dt      = 1e-3;
ctrlDiv = 10;
logDiv  = 10;
nStep   = 45000;
nLog    = nStep/logDiv;

h0 = 20;

X   = zeros(nLog, 21);
x   = [0; 0; -h0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0; ...
       p.betaMin; p.hoverW; p.hoverW; p.hoverW; p.hoverW; 0; 0; 0];
cmd = [p.hoverW; p.hoverW; p.hoverW; p.hoverW; 0; 0; 0; p.betaMin];
st  = struct('ip', zeros(3, 1), 'ia', zeros(3, 1), 'iH', 0);
j   = 0;

for k = 1:nStep
    t = (k - 1)*dt;

    if mod(k - 1, ctrlDiv) == 0
        r = uav.env.tilt_ref(t);
        [cmd, st] = uav.ctrl.tilt_cascade(x, r, st, p, c, dt*ctrlDiv);
    end

    if mod(k - 1, logDiv) == 0
        j = j + 1;
        X(j, :) = x';
    end

    k1 = deriv(x, cmd, p);
    k2 = deriv(x + 0.5*dt*k1, cmd, p);
    k3 = deriv(x + 0.5*dt*k2, cmd, p);
    k4 = deriv(x + dt*k3, cmd, p);

    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(7:10) = x(7:10)/norm(x(7:10));
    x(15:18) = max(0, min(p.wMax, x(15:18)));
end
end

function xd = deriv(x, cmd, p)
om   = x(15:18);
act  = x(19:21);
beta = x(14);

[Fb, Mb] = uav.plant.tilt_wrench(x, om, act, beta, p);
xd13 = uav.plant.rigid_body(x(1:13), Fb, Mb, p);

betaDot = (cmd(8) - beta)/p.tauBeta;
betaDot = max(-p.betaRate, min(p.betaRate, betaDot));

omDot  = (cmd(1:4) - om)/p.tauM;
actDot = (cmd(5:7) - act)/p.tauA;

xd = [xd13; betaDot; omDot; actDot];
end
