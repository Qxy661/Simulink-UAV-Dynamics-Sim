% r10_hover_fault  悬停断桨：分配方式与推力裕度的对照实验
%
%   20 m 悬停，t = 8 s 让 1 号电机完全停转，跑 30 s，记录最低高度、
%   水平漂移、滚转峰值。分两部分：
%     1) 三种分配方式（pinv / rank / rankrpi）——验证"最小范数会折中总推力"
%     2) 三种 wMax（1100 / 1250 / 1300）——验证推力裕度判据
%
%   结果写 research/results/csv/r10_hover_fault.csv，图写
%   research/results/fig/r10_hover_fault.png。

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(pwd));

c   = uav.cfg.tiltctrl();
ref = @(t) struct('p', [0; 0; -20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                  'psi', 0, 'h', 20, 'V', 0);

rows = {};
hAlts = {};

%% 1) 分配方式
for alloc = {'pinv', 'rank', 'rankrpi'}
    p = uav.cfg.tilt();
    s = baseSim();
    s.alloc = alloc{1};
    out = uav.sim.run_tilt(p, c, s, ref);
    m = score(out);
    fprintf('alloc %-8s hMin %7.2f m  drift %6.1f m  roll %6.1f deg\n', ...
            alloc{1}, m.hMin, m.drift, m.roll);
    rows(end+1, :) = {alloc{1}, p.wMax, p.kt*p.wMax^2/(p.mass*p.g/4), ...
                      m.hMin, m.drift, m.roll}; %#ok<SAGROW>
end

%% 2) 推力裕度
for wMax = [1100 1250 1300]
    p = uav.cfg.tilt();
    p.wMax = wMax;
    s = baseSim();
    s.alloc = 'rankrpi';
    out = uav.sim.run_tilt(p, c, s, ref);
    m = score(out);
    fprintf('wMax %4d (margin %.2f) hMin %6.2f m  drift %6.1f m  roll %6.1f deg\n', ...
            wMax, uav.cfg.thrust_margin(p), m.hMin, m.drift, m.roll);
    rows(end+1, :) = {sprintf('rankrpi_w%d', wMax), wMax, ...
                      uav.cfg.thrust_margin(p), m.hMin, m.drift, m.roll}; %#ok<SAGROW>
    hAlts{end+1} = [-out.p(:,3), out.t]; %#ok<SAGROW>
end

%% 落盘
csvDir = fullfile('research', 'results', 'csv');
if ~isfolder(csvDir), mkdir(csvDir); end
T = cell2table(rows, 'VariableNames', ...
    {'config', 'wMax', 'margin', 'hMin', 'drift', 'rollMax'});
writetable(T, fullfile(csvDir, 'r10_hover_fault.csv'));

figDir = fullfile('research', 'results', 'fig');
if ~isfolder(figDir), mkdir(figDir); end
f = figure('Visible', 'off', 'Position', [100 100 760 380], 'Color', 'w');
hold on
for k = 1:numel(hAlts)
    plot(hAlts{k}(:,2), hAlts{k}(:,1), 'LineWidth', 1.4);
end
yline(0, 'k--');
hold off
grid on
xlabel('时间 (s)');
ylabel('高度 (m)');
title('悬停断桨：三段推力裕度下的高度响应');
legend({'wMax 1100（裕度 1.60）', 'wMax 1250（2.07）', 'wMax 1300（2.24）'}, ...
       'Location', 'southwest');
exportgraphics(f, fullfile(figDir, 'r10_hover_fault.png'), 'Resolution', 150);
close(f);

fprintf('\nreference: research/REPORT_r6_hover_fault.md\n');


function s = baseSim()
s = uav.cfg.simcfg();
s.tEnd     = 30;
s.logDiv   = 10;
s.fault    = struct('t', 8, 'k', 1, 'eta', 0);
s.faultAware = true;
s.faultHard  = true;
end

function m = score(out)
post = out.t > 8;
m.hMin  = min(-out.p(post, 3));
m.drift = max(vecnorm(out.p(post, 1:2), 2, 2));
m.roll  = max(abs(rad2deg(out.eul(post, 1))));
end
