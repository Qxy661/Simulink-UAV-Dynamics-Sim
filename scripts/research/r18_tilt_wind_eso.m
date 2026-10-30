function T = r18_tilt_wind_eso()
%R18_TILT_WIND_ESO  倾转悬停抗风：ESO 扰动补偿能不能把 2 m/s 的边界往外推。
%   对照 3 组：不开 ESO、ESO(wo=8)、ESO(wo=8)+5 cm 量测噪声。
%   侧风 2/3/4/6 m/s，30 s 悬停，统计末段 10 s 的水平误差与倾角。
%   结论见 research/REPORT_r14_tilt_wind_eso.md。

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root); cd(root);

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 30;
s.logDiv = 10;

refHover = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                       'psi', 0, 'h', 20, 'V', 0);

winds = [2 3 4 5 6];
modes = {'off', 'eso', 'esoNoise'};

T = table();
fig = figure('Visible', 'off', 'Position', [100 100 960 380]);

for mi = 1:numel(modes)
    errs = zeros(size(winds));
    tilts = zeros(size(winds));
    for wi = 1:numel(winds)
        s2 = s;
        s2.windFn = @(tt) [winds(wi); 0; 0];
        switch modes{mi}
            case 'eso'
                s2.useEso = true; s2.woEso = 8;
            case 'esoNoise'
                s2.useEso = true; s2.woEso = 8; s2.esoNoise = 0.05;
        end
        o = uav.sim.run_tilt(p, c, s2, refHover);
        tail = o.t > o.t(end) - 10;
        errs(wi) = max(vecnorm(o.p(tail,1:2), 2, 2));
        tilts(wi) = max(max(abs(rad2deg(o.eul(tail,1:2)))));
        T = [T; table({modes{mi}}, winds(wi), errs(wi), tilts(wi), ...
            'VariableNames', {'mode','wind','errTail','tiltMax'})]; %#ok<AGROW>
    end
    subplot(1,2,1); hold on; grid on;
    semilogy(winds, max(errs, 1e-3), 'o-', 'LineWidth', 1.2);
    subplot(1,2,2); hold on; grid on;
    plot(winds, tilts, 'o-', 'LineWidth', 1.2);
end

subplot(1,2,1);
xlabel('侧风 (m/s)'); ylabel('末段 10 s 水平误差 (m)');
legend({'无 ESO','ESO','ESO + 5 cm 噪声'}, 'Location', 'northwest');
title('位置保持');
subplot(1,2,2);
xlabel('侧风 (m/s)'); ylabel('末段 10 s 最大倾角 (deg)');
legend({'无 ESO','ESO','ESO + 5 cm 噪声'}, 'Location', 'northwest');
title('姿态摆动');

csvDir = fullfile(root, 'research', 'results', 'csv');
figDir = fullfile(root, 'research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end
writetable(T, fullfile(csvDir, 'r18_tilt_wind_eso.csv'));
exportgraphics(fig, fullfile(figDir, 'r18_tilt_wind_eso.png'), 'Resolution', 140);
close(fig);

disp(T);
end
