function r4_backtransition(tag)
%R4_BACKTRANSITION 减速回悬停的逐拍诊断（auto-research 第 6 轮）。
%
%   r4_backtransition         基线：现有调度 + 现有增益
%   r4_backtransition('name') 给这一轮起名，产物写 research/results/ 下
%
%   任务：tilt_ref_v(t, 22) —— 0-5 s 悬停、5-20 s 加速、20-45 s 巡航、
%   45-60 s 减速到 0、60 s 后悬停。跑 75 s，打印减速段的
%   速度 / 短舱 / 俯仰 / 高度偏差 / 推力与力矩饱和度，定位失控环节。

if nargin < 1 || isempty(tag)
    tag = 'baseline';
end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 75;
s.logDiv = 10;

ref = @(t) uav.env.tilt_ref_v(t, 22);
out = uav.sim.run_tilt(p, c, s, ref);

V   = vecnorm(out.v, 2, 2);
h   = -out.p(:, 3);
dh  = h - p.htrim;
bt  = rad2deg(out.beta);
th  = rad2deg(out.eul(:, 2));
ep  = vecnorm(out.p - out.ref(:, 1:3), 2, 2);

% 短舱与前向推力：U1 = sum(kt*w^2)，前向分量 U1*sin(beta)
kt = p.kt;
U1 = kt * sum(out.om.^2, 2);
Ffwd = U1 .* sin(out.beta);
Tmax = 4 * kt * p.wMax^2;

fprintf('\n== back-transition（%s）==\n', tag);
fprintf('末端：高度 %.2f m，空速 %.2f m/s，短舱 %.1f deg，俯仰 %.1f deg\n', ...
    h(end), V(end), bt(end), th(end));
fprintf('峰值：|dh| %.2f m，俯仰 %.1f deg，水平误差 %.2f m\n', ...
    max(abs(dh)), max(abs(th)), max(ep));

tShow = [45 48 51 54 57 60 63 66 70 75];
fprintf('\n%6s %7s %7s %7s %8s %8s %8s %6s\n', ...
    't', 'V', 'beta', 'theta', 'dh', 'U1/U1max', 'Ffwd', 'k');
for tt = tShow
    [~, j] = min(abs(out.t - tt));
    k = uav.ctrl.tilt_schedule(ref(out.t(j)).V, p).k;
    fprintf('%6.1f %7.2f %7.1f %7.1f %8.2f %8.2f %8.1f %6.2f\n', ...
        out.t(j), V(j), bt(j), th(j), dh(j), U1(j)/Tmax, Ffwd(j), k);
end

% 减速段控制权限：机翼能提供多少升力、旋翼竖直分量剩多少
fprintf('\n减速段竖直力预算（每 3 s）：\n');
fprintf('%6s %8s %8s %8s %8s\n', 't', 'WingL', 'RotV', 'Weight', '缺口');
for tt = 45:3:60
    [~, j] = min(abs(out.t - tt));
    Vn = norm(out.v(j, :));
    qbar = 0.5 * p.rho * Vn^2;
    alpha = out.eul(j, 2) - uav.plant.fw_trim(p, max(Vn, 8)).alpha;
    CL = p.CL0 + p.CLa * alpha;
    Lw = qbar * p.S * CL;
    Rv = U1(j) * cos(out.beta(j));
    fprintf('%6.1f %8.1f %8.1f %8.1f %8.1f\n', out.t(j), Lw, Rv, p.mass*p.g, ...
        Lw + Rv - p.mass*p.g);
end

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
csvDir = fullfile(root, 'research', 'results', 'csv');
figDir = fullfile(root, 'research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end

T = table(out.t, V, bt, th, dh, U1/Tmax, Ffwd, ...
    'VariableNames', {'t_s', 'V_mps', 'beta_deg', 'theta_deg', 'dh_m', 'U1_frac', 'Ffwd_N'});
writetable(T, fullfile(csvDir, ['r4_' tag '.csv']));

f = figure('Color', 'w', 'Position', [100 100 900 520]);
subplot(2, 1, 1);
plot(out.t, V, 'b', out.t, out.ref(:, 6), 'r--'); grid on; ylabel('空速 (m/s)');
legend('实际', '参考', 'Location', 'best'); title('减速段：速度与短舱');
subplot(2, 1, 2);
yyaxis left; plot(out.t, bt, 'b'); ylabel('\beta (deg)');
yyaxis right; plot(out.t, dh, 'r'); ylabel('高度偏差 (m)');
grid on; xlabel('t (s)');
exportgraphics(f, fullfile(figDir, ['r4_' tag '.png']));
fprintf('\n写入 research/results/csv/r4_%s.csv\n', tag);
end
