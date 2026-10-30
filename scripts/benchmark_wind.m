% 阵风下的扰动抑制对比：PID、PID + 水平积分、PID + ESO。
% 同一阵风种子、同一任务，只换控制律；指标是 20–40 s 的水平位置误差。

cd(fileparts(fileparts(mfilename('fullpath'))));
addpath(pwd);

p = uav.cfg.quad();
base = uav.cfg.ctrl();
cfg = struct('dt', 1e-3, 'ctrlDiv', 10, 'tEnd', 40, 'logDiv', 10);
ref = uav.env.hover(0, 0, -5, 0);
gust = uav.env.wind_dryden(cfg.tEnd, cfg.dt, 10, 2.0, 11, [3; 2; 0]);

cInt = base;
cInt.Ki = [0.3; 0.3; 1.2];
cInt.iMax = [0.3; 0.3; 1.0];

names = {'PID', 'PID + 水平积分', 'PID + ESO', 'PID + ESO（5 cm 量测噪声）'};
ctrls = {base, cInt, base, base};
esos  = [false, false, true, true];
noise = [0, 0, 0, 0.05];

rmsV = zeros(1, 4); maxV = zeros(1, 4); meanV = zeros(2, 4); tiltV = zeros(1, 4);
errs = cell(1, 4);
rng(3, 'twister');
for i = 1:4
    cfgI = cfg;
    cfgI.useEso = esos(i);
    cfgI.esoNoise = noise(i);
    out = uav.sim.run_wind(p, ctrls{i}, cfgI, ref, gust);
    e = out.p - out.ref;
    tail = out.t > 20;
    errs{i} = e;
    rmsV(i) = sqrt(mean(sum(e(tail, 1:2).^2, 2)));
    maxV(i) = max(vecnorm(e(tail, 1:2), 2, 2));
    meanV(:, i) = mean(e(tail, 1:2))';
    tiltV(i) = rad2deg(max(abs(out.eul(:, 1:2)), [], 'all'));
end

fprintf('阵风：均值 [%.1f %.1f] m/s，湍流强度 %.1f m/s，%d s\n', gust.Wmean(1), gust.Wmean(2), gust.sigma, cfg.tEnd);
fprintf('%-26s | 水平 RMS | 最大 | 平均偏移 | 最大倾角\n', '控制器');
for i = 1:4
    fprintf('%-28s | %.3f m | %.3f m | [%.2f %.2f] | %.1f°\n', names{i}, rmsV(i), maxV(i), meanV(:, i), tiltV(i));
end

fig = figure('Visible', 'off', 'Position', [100, 100, 900, 380]);
subplot(1, 2, 1);
hold on; grid on;
for i = 1:4
    plot(errs{i}(:, 2), errs{i}(:, 1), 'LineWidth', 1.2);
end
xlabel('东向误差 (m)'); ylabel('北向误差 (m)');
title('阵风下的水平轨迹');
legend(names, 'Location', 'northeast');

subplot(1, 2, 2);
hold on; grid on;
for i = 1:4
    plot(out.t, sqrt(errs{i}(:, 1).^2 + errs{i}(:, 2).^2), 'LineWidth', 1.2);
end
xlabel('时间 (s)'); ylabel('水平误差 (m)');
title('水平误差随时间');
legend(names, 'Location', 'northeast');

figDir = fullfile(pwd, 'figures');
if ~isfolder(figDir), mkdir(figDir); end
saveas(fig, fullfile(figDir, 'benchmark_wind.png'));
close(fig);
fprintf('图已存 figures/benchmark_wind.png\n');
