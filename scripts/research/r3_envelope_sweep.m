function r3_envelope_sweep(Vlist)
%R3_ENVELOPE_SWEEP 当前代码下的倾转包线扫描（复现 r4/r5 的结论）。
%
%   r3_envelope_sweep          默认 8:2:24 m/s
%   r3_envelope_sweep([10 22]) 指定巡航速度
%
%   每个速度跑 45 s 的 uav.env.tilt_ref_v 任务，记录峰值/稳态高度偏差、
%   最大俯仰、最大短舱角、有无饱和，写 research/results/csv/r6_summary.csv
%   与 research/results/fig/r6_budget.png。

if nargin < 1 || isempty(Vlist)
    Vlist = 8:2:24;
end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 45;
s.logDiv = 50;

n = numel(Vlist);
peak = zeros(n, 1);
steady = zeros(n, 1);
tiltMax = zeros(n, 1);
betaMax = zeros(n, 1);
satMax = zeros(n, 1);
betaEnd = zeros(n, 1);
Vend = zeros(n, 1);

for i = 1:n
    V = Vlist(i);
    ref = @(t) uav.env.tilt_ref_v(t, V);
    out = uav.sim.run_tilt(p, c, s, ref);
    h = -out.p(:, 3);
    dh = h - p.htrim;
    peak(i) = max(abs(dh));
    steady(i) = max(abs(dh(out.t > 35)));
    tiltMax(i) = max(abs(rad2deg(out.eul(:, 2))));
    betaMax(i) = max(rad2deg(out.beta));
    betaEnd(i) = rad2deg(out.beta(end));
    Vend(i) = norm(out.v(end, :));
    % run_tilt 不返回饱和标志，用转速贴顶近似
    satMax(i) = double(max(out.om(:)) > 0.999 * p.wMax);
    fprintf('V=%2d m/s  peak %5.2f m  steady %4.2f m  pitch %4.1f deg  beta %5.1f deg  sat %d\n', ...
        V, peak(i), steady(i), tiltMax(i), betaMax(i), satMax(i) > 0);
end

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
csvDir = fullfile(root, 'research', 'results', 'csv');
figDir = fullfile(root, 'research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end

T = table(Vlist(:), peak, steady, tiltMax, betaMax, betaEnd, Vend, satMax, ...
    'VariableNames', {'Vmax_mps', 'peak_dh_m', 'steady_dh_m', 'max_pitch_deg', ...
    'max_beta_deg', 'beta_end_deg', 'V_end_mps', 'sat'});
writetable(T, fullfile(csvDir, 'r6_summary.csv'));

f = figure('Color', 'w', 'Position', [100 100 720 420]);
subplot(1, 2, 1);
plot(Vlist, peak, 'o-', Vlist, steady, 's--'); grid on
xlabel('巡航速度 (m/s)'); ylabel('高度偏差 (m)'); title('加速段峰值与稳态');
legend('峰值', '35 s 后', 'Location', 'northwest');

subplot(1, 2, 2);
plot(Vlist, betaMax, 'o-'); grid on
xlabel('巡航速度 (m/s)'); ylabel('\beta_{max} (deg)'); title('短舱最大倾角');
exportgraphics(f, fullfile(figDir, 'r6_budget.png'));
fprintf('写入 %s\n', fullfile(csvDir, 'r6_summary.csv'));
end
