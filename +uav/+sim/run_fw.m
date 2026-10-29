function out = run_fw(p, c, s, ref, x0)
%RUN_FW  固定翼闭环仿真。
%   out = uav.sim.run_fw(p, c, s, ref, x0)
%   ref 为参考结构体（h/psi/V）或 @(t) 返回该结构体的句柄。
%   x0 省略时用配平点：高度取 ref.h，姿态俯仰等于配平迎角。
%   舵机与油门按一阶滞后在每一步精确离散更新，RK4 内部视为常值。

r0 = eval_ref(ref, 0);

if nargin < 5 || isempty(x0)
    tr = uav.plant.fw_trim(p, r0.V);
    q  = uav.plant.euler2quat(0, tr.alpha, r0.psi);
    Vb = r0.V * [cos(tr.alpha); 0; sin(tr.alpha)];
    x0 = [0; 0; -r0.h; Vb; q; 0; 0; 0];
end

dt   = s.dt;
N    = round(s.tEnd/dt);
nLog = floor(N/s.logDiv) + 1;

tLog  = zeros(nLog, 1);
xLog  = zeros(nLog, 13);
pLog  = zeros(nLog, 3);
vLog  = zeros(nLog, 3);
eLog  = zeros(nLog, 3);
aLog  = zeros(nLog, 4);
cLog  = zeros(nLog, 4);
rLog  = zeros(nLog, 3);

x  = x0(:);
tr = uav.plant.fw_trim(p, r0.V);
act = [tr.thr; tr.de; 0; 0];
cmd = act;
st  = struct('iV', 0);

j = 0;
for k = 1:N
    t = (k-1)*dt;

    if mod(k-1, s.ctrlDiv) == 0
        rNow = eval_ref(ref, t);
        [cmd, st] = uav.ctrl.fw_cascade(x, rNow, st, p, c, s.dt*s.ctrlDiv);
    end

    act = uav.plant.actuator_step(cmd, act, p, dt);

    % 记录时刻 t 的状态（积分之前），与 Simulink 积分器输出的对齐方式一致
    if mod(k-1, s.logDiv) == 0
        j = j + 1;
        tLog(j) = t;
        xLog(j, :) = x';
        pLog(j, :) = x(1:3)';
        vLog(j, :) = x(4:6)';
        eLog(j, :) = uav.plant.quat2euler(x(7:10))';
        aLog(j, :) = act';
        cLog(j, :) = cmd';
        rLog(j, :) = [rNow.h; rNow.psi; rNow.V]';
    end

    k1 = deriv(x, act, p);
    k2 = deriv(x + 0.5*dt*k1, act, p);
    k3 = deriv(x + 0.5*dt*k2, act, p);
    k4 = deriv(x + dt*k3, act, p);
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(7:10) = x(7:10)/norm(x(7:10));
end

out.t   = tLog(1:j);
out.x   = xLog(1:j, :);
out.p   = pLog(1:j, :);
out.v   = vLog(1:j, :);
out.eul = eLog(1:j, :);
out.act = aLog(1:j, :);
out.cmd = cLog(1:j, :);
out.ref = rLog(1:j, :);
out.prm = p;
end

function xd = deriv(x, act, p)
Vb = x(4:6);
w  = x(11:13);
[Fb, Mb] = uav.plant.fw_aero(Vb, w, act, p);
xd = uav.plant.rigid_body(x, Fb, Mb, p);
end

function r = eval_ref(ref, t)
if isa(ref, 'function_handle')
    r = ref(t);
else
    r = ref;
end
end
