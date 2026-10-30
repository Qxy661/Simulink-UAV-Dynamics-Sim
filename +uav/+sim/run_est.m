function out = run_est(p, c, s, ref)
%RUN_EST 带传感器与 EKF 的闭环仿真。
%   被控对象与控制律与 uav.sim.run 相同，区别有三处：
%     1) IMU 200 Hz、GPS 5 Hz、气压 20 Hz、磁 50 Hz，读数含常值零偏与白噪声；
%     2) 控制器只拿到 EKF 的估计状态，拿不到真值；
%     3) 输出同时留真值与估计，便于统计估计误差与零偏收敛。
%
%   out = uav.sim.run_est(p, c, s, ref)
%   s.sens 可选字段（括号为默认）：
%     bg [0.02;-0.015;0.01] 陀螺常值零偏 rad/s
%     ba [0.10;-0.08;0.06]  加计常值零偏 m/s^2
%     sigmaG 0.004          陀螺噪声 rad/s（单样本，200 Hz）
%     sigmaA 0.08           加计噪声 m/s^2（单样本，200 Hz）
%     sigmaGps [1;1;2]      GPS 位置噪声 m
%     sigmaBaro 0.15        气压高度噪声 m
%     sigmaMag 0.03         磁航向噪声 rad
%     gpsDrop []            丢星区间 [t0 t1]
%     seed 7
%
%   见 docs/estimation.md。

if nargin < 4 || isempty(ref)
    ref = @(t) uav.env.hover(0, 0, -5, 0);
end
if ~isfield(s, 'dt'), s.dt = 1e-3; end
if ~isfield(s, 'ctrlDiv'), s.ctrlDiv = 10; end
if ~isfield(s, 'tEnd'), s.tEnd = 20; end
if ~isfield(s, 'logDiv'), s.logDiv = 10; end
if ~isfield(s, 'sens'), s.sens = struct(); end

ss = s.sens;
if ~isfield(ss, 'bg'), ss.bg = [0.02; -0.015; 0.01]; end
if ~isfield(ss, 'ba'), ss.ba = [0.10; -0.08; 0.06]; end
if ~isfield(ss, 'sigmaG'), ss.sigmaG = 0.004; end
if ~isfield(ss, 'sigmaA'), ss.sigmaA = 0.08; end
if ~isfield(ss, 'sigmaGps'), ss.sigmaGps = [1; 1; 2]; end
if ~isfield(ss, 'sigmaBaro'), ss.sigmaBaro = 0.15; end
if ~isfield(ss, 'sigmaGpsVel'), ss.sigmaGpsVel = 0.15; end
if ~isfield(ss, 'sigmaMag'), ss.sigmaMag = 0.03; end
if ~isfield(ss, 'gpsDrop'), ss.gpsDrop = []; end
if ~isfield(ss, 'seed'), ss.seed = 7; end
if ~isfield(ss, 'mRef'), ss.mRef = [0.42; 0; 0.91]; end
if ~isfield(ss, 'useAccel'), ss.useAccel = true; end
if ~isfield(ss, 'useMag'), ss.useMag = true; end
if ~isfield(ss, 'usePos'), ss.usePos = true; end
if ~isfield(ss, 'useFlow'), ss.useFlow = true; end
if ~isfield(ss, 'sigmaFlow'), ss.sigmaFlow = 0.05; end
if ~isfield(ss, 'flowScale'), ss.flowScale = 1.02; end

rng(ss.seed);

dt = s.dt;
N = round(s.tEnd / dt);
nLog = floor(N / s.logDiv) + 1;

imuDiv = 5;      % 200 Hz
gpsDiv = 200;    % 5 Hz
baroDiv = 50;    % 20 Hz
magDiv = 20;     % 50 Hz
flowDiv = 20;    % 50 Hz

r0 = eval_ref(ref, 0);
x = [r0.p(:); zeros(3, 1); uav.plant.euler2quat(0, 0, r0.psi); zeros(3, 1)];
e = uav.est.ekf_init(x(1:3), x(7:10));
st = struct('ip', [0; 0; 0], 'ia', [0; 0; 0]);
w = repmat(p.hoverW, 4, 1);
wcmd = w;
Tcmd = 0;
Mcmd = [0; 0; 0];
sat = false;
rNow = r0;
gyroMeas = x(11:13);
pF = e.p; vF = e.v;
if ~isfield(ss, 'estLp'), ss.estLp = 5; end
alphaF = exp(-2*pi*ss.estLp*dt*s.ctrlDiv);

if ~isfield(ss, 'qA'), ss.qA = ss.sigmaA; end
if ~isfield(ss, 'qG'), ss.qG = ss.sigmaG; end
nz = struct('sigmaA', ss.qA, 'sigmaG', ss.qG, ...
            'sigmaBg', 1e-4, 'sigmaBa', 1e-3);

tLog = zeros(nLog, 1);
xLog = zeros(nLog, 13);
pLog = zeros(nLog, 3);
vLog = zeros(nLog, 3);
eLog = zeros(nLog, 3);
wLog = zeros(nLog, 4);
uLog = zeros(nLog, 4);
rLog = zeros(nLog, 3);
satLog = false(nLog, 1);
estLog = zeros(nLog, 3);
estVLog = zeros(nLog, 3);
bgLog = zeros(nLog, 3);
baLog = zeros(nLog, 3);
thLog = zeros(nLog, 4);

j = 0;
for k = 1:N
    t = (k - 1) * dt;
    r = eval_ref(ref, t);

    if mod(k - 1, s.ctrlDiv) == 0
        rNow = r;
        if isfield(ss, 'useTruth') && ss.useTruth
            xhat = x;
        elseif isfield(ss, 'useTruthPV') && ss.useTruthPV
            xhat = [x(1:6); e.q; gyroMeas - e.bg];
        else
            pF = alphaF*pF + (1-alphaF)*e.p;
            vF = alphaF*vF + (1-alphaF)*e.v;
            if isfield(ss, 'rawRate') && ss.rawRate
                xhat = [pF; vF; e.q; gyroMeas];
            else
                xhat = [pF; vF; e.q; gyroMeas - e.bg];
            end
        end
        [Tcmd, Mcmd, st] = uav.ctrl.cascade(xhat, rNow, p, c, st, dt * s.ctrlDiv);
        [wcmd, sat] = uav.plant.mixer([Tcmd; Mcmd], p);
    end

    % 电机先按本步指令更新，再在整步内视为常值积分
    w = uav.plant.motor_step(w, wcmd, p, dt);

    [Fb, Mb] = uav.plant.quad_wrench(w, x(4:6), p);

    % IMU：比力取机体系合力除以质量
    if mod(k - 1, imuDiv) == 0
        z = uav.sens.imu_sample(x(11:13), Fb / p.mass, ss.bg, ss.ba, ss.sigmaG, ss.sigmaA);
        gyroMeas = z.gyro;
        e = uav.est.ekf_predict(e, z.gyro, z.accel, dt * imuDiv, nz);
        
    end

    if mod(k - 1, gpsDiv) == 0
        gpsOn = true;
        if ~isempty(ss.gpsDrop)
            gpsOn = (t < ss.gpsDrop(1)) || (t > ss.gpsDrop(2));
        end
        if gpsOn && ss.usePos
            [zp, zv] = uav.sens.gps_sample(x(1:3), ss.sigmaGps(1), ss.sigmaGps(3), x(4:6), ss.sigmaGpsVel);
            e = uav.est.ekf_update_gps(e, zp, ss.sigmaGps);
            e = uav.est.ekf_update_vel(e, zv, ss.sigmaGpsVel);
        end
    end
    if mod(k - 1, baroDiv) == 0
        zb = uav.sens.baro_sample(-x(3), ss.sigmaBaro);
        e = uav.est.ekf_update_baro(e, zb, ss.sigmaBaro);
    end
    eul = uav.plant.quat2euler(x(7:10));
    if ss.useAccel && mod(k - 1, magDiv) == 0
        e = uav.est.ekf_update_accel(e, z.accel, 0.5, 0.3, gyroMeas - e.bg, 0.3);
    end
    if ss.useFlow && mod(k - 1, flowDiv) == 0
        zf = uav.sens.flow_sample(x(7:10), x(4:6), ss.sigmaFlow, ss.flowScale);
        e = uav.est.ekf_update_flow(e, zf, ss.sigmaFlow);
    end
    if ss.useMag && mod(k - 1, magDiv) == 0
        zm = uav.sens.mag_sample(x(7:10), ss.mRef, ss.sigmaMag);
        e = uav.est.ekf_update_mag(e, zm, ss.sigmaMag, ss.mRef);
    end

    % 记录积分前的状态，与 Simulink 积分器输出的对齐方式一致
    if mod(k - 1, s.logDiv) == 0
        j = j + 1;
        tLog(j) = t;
        xLog(j, :) = x';
        pLog(j, :) = x(1:3)';
        vLog(j, :) = x(4:6)';
        eLog(j, :) = eul';
        wLog(j, :) = w';
        uLog(j, :) = [Tcmd, Mcmd'];
        rLog(j, :) = rNow.p';
        satLog(j) = sat;
        estLog(j, :) = e.p';
        estVLog(j, :) = e.v';
        bgLog(j, :) = e.bg';
        baLog(j, :) = e.ba';
        thLog(j, :) = e.q';
    end

    k1 = deriv(x, w, p);
    k2 = deriv(x + 0.5 * dt * k1, w, p);
    k3 = deriv(x + 0.5 * dt * k2, w, p);
    k4 = deriv(x + dt * k3, w, p);
    x = x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
    x(7:10) = x(7:10) / norm(x(7:10));
end

out.t = tLog(1:j);
out.x = xLog(1:j, :);
out.p = pLog(1:j, :);
out.v = vLog(1:j, :);
out.eul = eLog(1:j, :);
out.w = wLog(1:j, :);
out.u = uLog(1:j, :);
out.ref = rLog(1:j, :);
out.sat = satLog(1:j);
out.est = estLog(1:j, :);
out.estV = estVLog(1:j, :);
out.bg = bgLog(1:j, :);
out.ba = baLog(1:j, :);
out.qEst = thLog(1:j, :);
out.prm = p;
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
