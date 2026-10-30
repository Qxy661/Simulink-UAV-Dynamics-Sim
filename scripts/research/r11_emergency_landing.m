% r11_emergency_landing  悬停断桨后的应急着陆包线
%
%   20 m 悬停，t = 8 s 断 1 号电机，进入应急模式（放弃水平定点、按固定
%   速率下降、航向锁当前）。扫下降速率，看能不能落到地面、接地时姿态
%   与垂直速度多大。推力裕度只有 1.60 的机体，这是"摔得轻一点"的边界。
%
%   结果写 research/results/csv/r11_emergency_landing.csv，
%   图写 research/results/fig/r11_emergency_landing.png。

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(pwd));

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
ref = @(t) struct('p', [0; 0; -20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                  'psi', 0, 'h', 20, 'V', 0);

rates = [0.8 1.2 1.6 2.0];
rows = {};
curves = {};

for rate = rates
    s = uav.cfg.simcfg();
    s.tEnd = 45; s.logDiv = 10;
    s.alloc = 'rankrpi';
    s.faultAware = true; s.faultHard = true;
    s.fault = struct('t', 8, 'k', 1, 'eta', 0);
    s.emergency = true; s.emergencyRate = rate;

    out = uav.sim.run_tilt(p, c, s, ref);
    h = -out.p(:, 3);
    k = find(h <= 0.05, 1);
    if isempty(k)
        tTouch = NaN; vz = NaN; tilt = NaN; drift = NaN;
        fprintf('rate %.1f m/s: 未接地，45 s 后高度 %5.2f m\n', rate, h(end));
    else
        tTouch = out.t(k);
        vz     = -out.v(k, 3);                 % 向下为正
        tilt   = max(abs(rad2deg(out.eul(k, 1:2))));
        drift  = norm(out.p(k, 1:2));
        fprintf('rate %.1f m/s: 接地 t=%5.2f s  vz=%5.2f m/s  tilt=%5.1f deg  drift=%5.1f m\n', ...
                rate, tTouch, vz, tilt, drift);
    end
    rows(end+1, :) = {rate, tTouch, vz, tilt, drift}; %#ok<SAGROW>
    curves{end+1} = [out.t, h, rad2deg(vecnorm(out.eul(:,1:2), 2, 2))]; %#ok<SAGROW>
end

csvDir = fullfile('research', 'results', 'csv');
if ~isfolder(csvDir), mkdir(csvDir); end
T = cell2table(rows, 'VariableNames', ...
    {'emergencyRate', 'tTouch', 'vzTouch', 'tiltTouch', 'driftTouch'});
writetable(T, fullfile(csvDir, 'r11_emergency_landing.csv'));

figDir = fullfile('research', 'results', 'fig');
if ~isfolder(figDir), mkdir(figDir); end
f = figure('Visible', 'off', 'Position', [100 100 900 360], 'Color', 'w');
subplot(1,2,1); hold on
for k = 1:numel(curves), plot(curves{k}(:,1), curves{k}(:,2), 'LineWidth', 1.2); end
yline(0, 'k--'); hold off; grid on
xlabel('时间 (s)'); ylabel('高度 (m)'); title('应急下降：高度');
legend(arrayfun(@(r) sprintf('%.1f m/s', r), rates, 'UniformOutput', false), 'Location', 'southwest');
subplot(1,2,2); hold on
for k = 1:numel(curves), plot(curves{k}(:,1), curves{k}(:,3), 'LineWidth', 1.2); end
hold off; grid on
xlabel('时间 (s)'); ylabel('倾角 (deg)'); title('应急下降：姿态偏差');
exportgraphics(f, fullfile(figDir, 'r11_emergency_landing.png'), 'Resolution', 150);
close(f);

fprintf('\nreference: research/REPORT_r7_emergency.md\n');
