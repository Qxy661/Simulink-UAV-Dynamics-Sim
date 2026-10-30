function out = run_wind(p, c, s, ref, gust, x0)
%RUN_WIND  带 Dryden 阵风与可选 ESO 补偿的四旋翼闭环仿真。
%   out = uav.sim.run_wind(p, c, s, ref, gust, x0)
%   gust 为 uav.env.wind_dryden 返回的结构体，省略时不加风。
%   s.useEso = true 时启用三轴线性 ESO，把估计出的扰动加速度从
%   参考加速度里扣掉（前馈补偿），控制器本体不变。
%   其余约定与 uav.sim.run 一致：RK4 积分、控制器每 ctrlDiv 步更新、
%   执行机构先更新后积分、记录都在积分之前。

if nargin < 6 || isempty(x0)
    r0 = eval_ref(ref, 0);
    x0 = [r0.p; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
end
if nargin < 5 || isempty(gust)
    gust = struct('t', [0; 1], 'W', [0 0 0; 0 0 0]);
end

dt    = s.dt;
N     = round(s.tEnd/dt);
nLog  = floor(N/s.logDiv) + 1;
dtc   = s.dt*s.ctrlDiv;

useEso = isfield(s, 'useEso') && s.useEso;
if useEso
    if isfield(s, 'woEso')
        wo = s.woEso;
    else
        wo = 8;                 % 观测器带宽，约为位置环带宽的 5 倍
    end
    eso = uav.ctrl.eso_init(wo);
    if isfield(s, 'esoNoise')
        esoNoise = s.esoNoise;
    else
        esoNoise = 0;
    end
    dHat = [0; 0; 0];
end

tLog  = zeros(nLog, 1);
xLog  = zeros(nLog, 13);
pLog  = zeros(nLog, 3);
vLog  = zeros(nLog, 3);
eLog  = zeros(nLog, 3);
wLog  = zeros(nLog, 4);
uLog  = zeros(nLog, 4);
rLog  = zeros(nLog, 3);
satLog = false(nLog, 1);
dLog  = zeros(nLog, 3);

x = x0(:);
w = repmat(p.hoverW, 4, 1);
st = struct('ip', [0; 0; 0], 'ia', [0; 0; 0]);
wcmd = w;
Tcmd = 0;
Mcmd = [0; 0; 0];
sat  = false;
rNow = eval_ref(ref, 0);

j = 0;
for k = 1:N
    t = (k - 1) * dt;

    if mod(k - 1, s.ctrlDiv) == 0
        rNow = eval_ref(ref, t);
        rEff = rNow;
        if useEso
            rEff.a = rNow.a - dHat;      % 扰动前馈：等价于抵消常值扰动
        end
        [Tcmd, Mcmd, st] = uav.ctrl.cascade(x, rEff, p, c, st, dtc);
        [wcmd, sat] = uav.plant.mixer([Tcmd; Mcmd], p);

        if useEso
            % 控制器这一拍实际给出的加速度：推力沿机体 -z，再加回重力
            R = uav.plant.q2dcm(x(7:10));
            aCmd = -Tcmd/p.mass * R(:, 3) + [0; 0; p.g];
            [eso, dHat] = uav.ctrl.eso_step(eso, x(1:3) + esoNoise*randn(3,1), aCmd, dtc);
        end
    end

    w = uav.plant.motor_step(w, wcmd, p, dt);

    k1 = deriv(x, w, p, windNed(gust, t));
    k2 = deriv(x + 0.5*dt*k1, w, p, windNed(gust, t + 0.5*dt));
    k3 = deriv(x + 0.5*dt*k2, w, p, windNed(gust, t + 0.5*dt));
    k4 = deriv(x + dt*k3, w, p, windNed(gust, t + dt));

    if mod(k - 1, s.logDiv) == 0
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
        if useEso
            dLog(j, :) = dHat';
        end
    end

    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(7:10) = x(7:10)/norm(x(7:10));
end

out.t   = tLog(1:j);
out.x   = xLog(1:j, :);
out.p   = pLog(1:j, :);
out.v   = vLog(1:j, :);
out.eul = eLog(1:j, :);
out.w   = wLog(1:j, :);
out.u   = uLog(1:j, :);
out.ref = rLog(1:j, :);
out.sat = satLog(1:j);
out.dHat = dLog(1:j, :);
out.prm = p;
end

function xd = deriv(x, w, p, Wned)
R = uav.plant.q2dcm(x(7:10));
Wb = R' * Wned;
[Fb, Mb] = uav.plant.quad_wrench(w, x(4:6), p, Wb);
xd = uav.plant.rigid_body(x, Fb, Mb, p);
end

function W = windNed(gust, t)
if isempty(gust) || ~isfield(gust, 'W')
    W = [0; 0; 0];
    return;
end
n = size(gust.W, 1);
dtg = gust.t(2) - gust.t(1);
k = min(max(floor(t/dtg) + 1, 1), n);
W = gust.W(k, :)';
end

function r = eval_ref(ref, t)
if isa(ref, 'function_handle')
    r = ref(t);
else
    r = ref;
end
end
