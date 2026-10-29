function out = run(p, c, s, ref, x0)
%RUN  四旋翼闭环仿真。
%   out = uav.sim.run(p, c, s, ref, x0)
%   ref 为参考结构体或 @(t) 返回参考结构体的函数句柄。
%   x0 省略时按参考初始位置悬停起飞。
%   积分用 RK4（dt），控制器每 ctrlDiv 步更新一次。

if nargin < 5 || isempty(x0)
    r0 = eval_ref(ref, 0);
    x0 = [r0.p; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
end

dt   = s.dt;
N    = round(s.tEnd/dt);
nLog = floor(N/s.logDiv) + 1;

tLog = zeros(nLog, 1);
xLog = zeros(nLog, 13);
pLog = zeros(nLog, 3);
vLog = zeros(nLog, 3);
eLog = zeros(nLog, 3);
wLog = zeros(nLog, 4);
uLog = zeros(nLog, 4);
rLog = zeros(nLog, 3);
satLog = false(nLog, 1);

x = x0(:);
w = repmat(p.hoverW, 4, 1);
st = struct('ip', [0; 0; 0], 'ia', [0; 0; 0]);
wcmd = w;
Tcmd = 0;
Mcmd = [0; 0; 0];
sat  = false;
rNow = eval_ref(ref, 0);

% 控制器选择：'pid'（默认级联）、'lqr'、'mpc'
if isfield(s, 'ctrl')
    ctrlName = s.ctrl;
else
    ctrlName = 'pid';
end
if isfield(s, 'b')
    b = s.b;            % 基准对比时允许覆盖线性控制器权重
else
    b = uav.cfg.benchctrl();
end
dtc = s.dt*s.ctrlDiv;

% 电机失效/降效：s.fault = struct('t', 5, 'k', 1, 'eta', 0.35)
%   eta = 0 表示完全停转；0 < eta < 1 表示该电机推力与反扭矩按比例下降。
fault = [];
if isfield(s, 'fault'), fault = s.fault; end

j = 0;
for k = 1:N
    t = (k-1)*dt;
    act = ones(4, 1);
    if ~isempty(fault) && t >= fault.t
        if isfield(fault, 'eta')
            act(fault.k) = fault.eta;
        else
            act(fault.k) = 0;
        end
    end

    if mod(k-1, s.ctrlDiv) == 0
        rNow = eval_ref(ref, t);
        if any(act == 0)
            % 有电机完全停转时偏航不可控：航向锁在当前值，偏航力矩交给伪逆
            eNow = uav.plant.quat2euler(x(7:10));
            rNow.psi = eNow(3);
        end
        switch ctrlName
            case 'lqr'
                [Tcmd, Mcmd, st] = uav.ctrl.lqr_ctrl(x, rNow, p, c, b, st, dtc);
            case 'mpc'
                [Tcmd, Mcmd, st] = uav.ctrl.mpc_ctrl(x, rNow, p, c, b, st, dtc);
            otherwise
                [Tcmd, Mcmd, st] = uav.ctrl.cascade(x, rNow, p, c, st, dtc);
        end
        if any(act == 0)
            Mcmd(3) = 0;            % 反扭矩缺口配对不了，偏航力矩只能放弃
        end
        [wcmd, sat] = uav.plant.mixer_fault([Tcmd; Mcmd], p, act);
    end

    % 电机先按本步指令更新，再在整步内视为常值积分
    w = uav.plant.motor_step(w, wcmd, p, dt);
    wEff = sqrt(act) .* w;          % 降效电机：转速照旧，推力与反扭矩按系数缩

    k1 = deriv(x, wEff, p);
    k2 = deriv(x + 0.5*dt*k1, wEff, p);
    k3 = deriv(x + 0.5*dt*k2, wEff, p);
    k4 = deriv(x + dt*k3, wEff, p);

    % 记录时刻 t 的状态（积分之前），与 Simulink 积分器输出的对齐方式一致
    if mod(k-1, s.logDiv) == 0
        j = j + 1;
        tLog(j) = t;
        xLog(j, :) = x';
        pLog(j, :) = x(1:3)';
        vLog(j, :) = x(4:6)';
        eLog(j, :) = uav.plant.quat2euler(x(7:10))';
        wLog(j, :) = w';
        uLog(j, :) = [Tcmd, Mcmd'];
        rLog(j, :) = rNow.p';
        satLog(j) = sat;
    end

    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(7:10) = x(7:10)/norm(x(7:10));
end

out.t    = tLog(1:j);
out.x    = xLog(1:j, :);
out.p    = pLog(1:j, :);
out.v    = vLog(1:j, :);
out.eul  = eLog(1:j, :);
out.w    = wLog(1:j, :);
out.u    = uLog(1:j, :);
out.ref  = rLog(1:j, :);
out.sat  = satLog(1:j);
out.prm  = p;
end

function xd = deriv(x, w, p)
Vb = x(4:6);
[Fb, Mb] = uav.plant.quad_wrench(w, Vb, p);
xd = uav.plant.rigid_body(x, Fb, Mb, p);
end

function r = eval_ref(ref, t)
if isa(ref, 'function_handle')
    r = ref(t);
else
    r = ref;
end
end
