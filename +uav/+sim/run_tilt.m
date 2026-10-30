function out = run_tilt(p, c, s, ref, x0)
%RUN_TILT  倾转四旋翼闭环仿真。
%   out = uav.sim.run_tilt(p, c, s, ref, x0)
%   ref 为结构体或 @(t) 句柄；省略 x0 时从 ref(0) 的悬停状态起飞。
%
%   状态 x（21 维）：
%     1:13  刚体 [pn;pe;pd; u;v;w; qw;qx;qy;qz; q;p;r]
%     14    短舱倾角 beta
%     15:18 旋翼转速
%     19:21 舵面 [de; da; dr]
%   控制器按 ctrlDiv 分频，短舱/电机/舵面在 plant 内部做一阶滞后。

if nargin < 4
    error('run_tilt:ref', '需要参考信号');
end

r0 = eval_ref(ref, 0);
if nargin < 5 || isempty(x0)
    x0 = [r0.p(:); 0; 0; 0; 1; 0; 0; 0; 0; 0; 0; ...
          p.betaMin; p.hoverW*ones(4,1); 0; 0; 0];
end

dt   = s.dt;
N    = round(s.tEnd/dt);
nLog = floor(N/s.logDiv) + 1;

tLog = zeros(nLog, 1);
xLog = zeros(nLog, 21);
pLog = zeros(nLog, 3);
vLog = zeros(nLog, 3);
eLog = zeros(nLog, 3);
wLog = zeros(nLog, 3);
cLog = zeros(nLog, 8);
rLog = zeros(nLog, 6);

x   = x0(:);
cmd = [p.hoverW*ones(4,1); 0; 0; 0; p.betaMin];
st  = struct('ip', zeros(3,1), 'ia', zeros(3,1), 'iH', 0);

j = 0;
for kk = 1:N
    t = (kk-1)*dt;

    if mod(kk-1, s.ctrlDiv) == 0
        rNow = eval_ref(ref, t);
        [cmd, st] = uav.ctrl.tilt_cascade(x, rNow, st, p, c, s.dt*s.ctrlDiv);
        cmd = cmd(:);
    end

    % 记录 t 时刻状态（积分之前），与 Simulink 积分器输出对齐
    if mod(kk-1, s.logDiv) == 0
        j = j + 1;
        tLog(j) = t;
        xLog(j, :) = x';
        pLog(j, :) = x(1:3)';
        vLog(j, :) = x(4:6)';
        eLog(j, :) = uav.plant.quat2euler(x(7:10))';
        wLog(j, :) = x(11:13)';
        cLog(j, :) = cmd';
        rLog(j, :) = [rNow.p(:); rNow.psi; rNow.h; rNow.V]';
    end

    k1 = deriv(x, cmd, p);
    k2 = deriv(x + 0.5*dt*k1, cmd, p);
    k3 = deriv(x + 0.5*dt*k2, cmd, p);
    k4 = deriv(x + dt*k3, cmd, p);
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(7:10) = x(7:10)/norm(x(7:10));
    x(15:18) = max(0, min(p.wMax, x(15:18)));
end

out.t    = tLog(1:j);
out.x    = xLog(1:j, :);
out.p    = pLog(1:j, :);
out.v    = vLog(1:j, :);
out.eul  = eLog(1:j, :);
out.w    = wLog(1:j, :);
out.cmd  = cLog(1:j, :);
out.ref  = rLog(1:j, :);
out.beta = xLog(1:j, 14);
out.om   = xLog(1:j, 15:18);
out.prm  = p;
end

function xd = deriv(x, cmd, p)
om   = x(15:18);
act  = x(19:21);
beta = x(14);

[Fb, Mb] = uav.plant.tilt_wrench(x, om, act, beta, p);
xd13 = uav.plant.rigid_body(x(1:13), Fb, Mb, p);

betaDot = (cmd(8) - beta) / p.tauBeta;
betaDot = max(-p.betaRate, min(p.betaRate, betaDot));

omDot  = (cmd(1:4) - om) / p.tauM;
actDot = (cmd(5:7) - act) / p.tauA;

xd = [xd13; betaDot; omDot; actDot];
end

function r = eval_ref(ref, t)
if isa(ref, 'function_handle')
    r = ref(t);
else
    r = ref;
end
end
