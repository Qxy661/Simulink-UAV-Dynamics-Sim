function T = benchmark_controllers()
%BENCHMARK_CONTROLLERS  三种控制器在同一位置阶跃上的对比。
%   任务：t=1 s 从 [0;0;-5] 阶跃到 [3;2;-5]，航向 0.4 rad，14 s。
%   PID 为解析级联（uav.ctrl.cascade），LQR 与 MPC 用悬停点线性模型。
%   输出：控制台表格 + figures/benchmark_step.png + docs 表格用数据。

p = uav.cfg.quad();
c = uav.cfg.ctrl();
s = uav.cfg.simcfg();
s.tEnd   = 14;
s.logDiv = 10;

p0  = [0; 0; -5];
p1  = [3; 2; -5];
psi = 0.4;
ref = @(t) uav.env.step(t, 1, p0, p1, psi);

names = {'pid', 'lqr', 'mpc'};
label = {'PID 级联', 'LQR', 'MPC'};
T = table('Size', [numel(names), 7], ...
    'VariableTypes', {'string','double','double','double','double','double','double'}, ...
    'VariableNames', {'ctrl','settle_s','peak_m','rms_m','tilt_deg','sat_pct','cpu_s'});

out = cell(numel(names), 1);
for i = 1:numel(names)
    s.ctrl = names{i};
    tic;
    out{i} = uav.sim.run(p, c, s, ref);
    el = toc;

    t   = out{i}.t;
    err = vecnorm(out{i}.p - p1', 2, 2);
    % 进入 5 cm 并保持
    setT = NaN;
    idx  = find(t > 1 & err < 0.05, 1, 'first');
    if ~isempty(idx) && all(err(idx:end) < 0.05)
        setT = t(idx) - 1;
    end
    peak = max(vecnorm(out{i}.p(t > 1, :) - p1', 2, 2));
    tail = t > 10;
    T.ctrl(i)     = string(label{i});
    T.settle_s(i) = setT;
    T.peak_m(i)   = peak;
    T.rms_m(i)    = sqrt(mean(err(tail).^2));
    T.tilt_deg(i) = max(vecnorm(out{i}.eul(:, 1:2), 2, 2))*180/pi;
    T.sat_pct(i)  = 100*mean(out{i}.sat);
    T.cpu_s(i)    = el;
end

disp(T);

% 对比图：俯视轨迹 + 误差
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 900 400]);
subplot(1, 2, 1);
cols = lines(numel(names));
hold on;
for i = 1:numel(names)
    plot(out{i}.p(:, 2), out{i}.p(:, 1), 'Color', cols(i, :), 'LineWidth', 1.4);
end
plot(p1(2), p1(1), 'k+', 'MarkerSize', 10);
hold off; grid on; axis equal;
xlabel('东向 pe (m)'); ylabel('北向 pn (m)');
legend(label, 'Location', 'southeast');
title('俯视轨迹');

subplot(1, 2, 2);
hold on;
for i = 1:numel(names)
    plot(out{i}.t, vecnorm(out{i}.p - p1', 2, 2), 'Color', cols(i, :), 'LineWidth', 1.2);
end
hold off; grid on;
xlabel('时间 (s)'); ylabel('位置误差 (m)');
set(gca, 'YScale', 'log', 'XLim', [0 14]);
legend(label, 'Location', 'northeast');
title('误差收敛');

root = fileparts(fileparts(mfilename('fullpath')));
figDir = fullfile(root, 'figures');
if ~isfolder(figDir), mkdir(figDir); end
saveas(f, fullfile(figDir, 'benchmark_step.png'));
close(f);
end
