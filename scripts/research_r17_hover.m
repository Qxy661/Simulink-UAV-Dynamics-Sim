function T = research_r17_hover()
%RESEARCH_R17_HOVER  悬停鲁棒性：初始偏差与侧风。
%   两组扫描，各写一行 CSV，并出一张双栏图：
%     左：不同初始位置偏差下的收敛过程（水平误差 vs 时间）
%     右：不同侧风速度下的末段水平误差
%   结论见 research/REPORT_r13_hover_robustness.md。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
cd(root);

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 30;
s.logDiv = 10;

refHover = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                       'psi', 0, 'h', 20, 'V', 0);

offs = [0.5 1 2];
winds = [0 1 2 3 4 6];

rows = {};
fig = figure('Visible', 'off', 'Position', [100 100 900 360]);

subplot(1,2,1); hold on; grid on;
for off = offs
    x0 = [off; 0; -20; zeros(3,1); 1;0;0;0; zeros(3,1); ...
          p.betaMin; p.hoverW*ones(4,1); zeros(3,1)];
    o = uav.sim.run_tilt(p, c, s, refHover, x0);
    e = vecnorm(o.p(:,1:2), 2, 2);
    plot(o.t, e, 'LineWidth', 1.2);
    m = o.t > 15;
    rows{end+1} = sprintf('offset,%.2f,%.3f,%.2f,%.2f', ...
        off, max(e(m)), max(abs(rad2deg(o.eul(m,2)))), ...
        max(abs(rad2deg(o.eul(m,1))))); %#ok<AGROW>
end
xlabel('t (s)'); ylabel('水平误差 (m)');
legend(arrayfun(@(v) sprintf('%.1f m 偏差', v), offs, 'UniformOutput', false), ...
       'Location', 'northeast');
title('初始位置偏差');

subplot(1,2,2); hold on; grid on;
errT = zeros(size(winds));
for i = 1:numel(winds)
    s2 = s; s2.windFn = @(t) [winds(i); 0; 0];
    o = uav.sim.run_tilt(p, c, s2, refHover);
    m = o.t > 5;
    errT(i) = max(vecnorm(o.p(m,1:2), 2, 2));
    rows{end+1} = sprintf('wind,%.1f,%.3f,%.2f,%.2f', ...
        winds(i), errT(i), max(abs(rad2deg(o.eul(m,2)))), ...
        max(abs(rad2deg(o.eul(m,1))))); %#ok<AGROW>
end
semilogy(winds, max(errT, 1e-3), 'o-', 'LineWidth', 1.2);
xlabel('侧风 (m/s)'); ylabel('末段水平误差 (m)');
title('恒定侧风');

csvDir = fullfile(root, 'research', 'results', 'csv');
figDir = fullfile(root, 'research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end

fid = fopen(fullfile(csvDir, 'r17_hover.csv'), 'w');
fprintf(fid, 'kind,value,errTail_m,pitchMax_deg,rollMax_deg\n');
fprintf(fid, '%s\n', rows{:});
fclose(fid);

exportgraphics(fig, fullfile(figDir, 'r17_hover.png'), 'Resolution', 140);
close(fig);

T = rows;
fprintf('写好 research/results/csv/r17_hover.csv 与 fig/r17_hover.png\n');
end
