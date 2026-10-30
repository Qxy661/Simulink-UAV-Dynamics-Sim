% r17_tilt_wind  倾转构型悬停抗风：水平积分顶不顶得住？
%
%   四旋翼上用 ESO 把阵风下的悬停误差从 0.145 m 压到 0.006 m。
%   倾转构型的悬停由旋翼撑，但机翼面积大（S=1.6），侧风会把整机推走。
%   这里先问一个更便宜的问题：给水平通道加积分项够不够？
%
%   结果写 research/results/csv/r17_tilt_wind.csv，
%   图写 research/results/fig/r17_tilt_wind.png。

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(pwd));

p = uav.cfg.tilt();
c0 = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 40; s.logDiv = 10;

% 阵风：均值 6 m/s 吹向北、3 m/s 吹向东，湍流 2 m/s
gust = uav.env.wind_dryden(s.tEnd, s.dt, 6, 2, 21, [6; 3; 0]);
s.windFn = @(t) interp1(gust.t, gust.W, t, 'linear', 'extrap')';

ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                  'psi', 0, 'h', 20, 'V', 0);

cases = {'无积分', 0, 0; 'Ki_h 0.15', 0.15, 0.6; 'Ki_h 0.4', 0.40, 1.2};
rows = {};
for i = 1:size(cases,1)
    c = c0;
    c.Ki(1:2) = cases{i,2};
    c.iMax(1:2) = cases{i,3};
    out = uav.sim.run_tilt(p, c, s, ref);
    m = out.t > 5;
    err = vecnorm(out.p(m,1:2), 2, 2);
    rows(end+1,:) = {cases{i,1}, sqrt(mean(err.^2)), max(err), ...
                     max(abs(rad2deg(out.eul(m,1)))), max(abs(rad2deg(out.eul(m,2))))}; %#ok<SAGROW>
    fprintf('%-10s 水平 RMS %.3f m  峰值 %.3f m  最大滚转 %.2f°  最大俯仰 %.2f°\n', ...
            cases{i,1}, rows{end,2}, rows{end,3}, rows{end,4}, rows{end,5});
end

csvDir = fullfile('research','results','csv');
if ~isfolder(csvDir), mkdir(csvDir); end
T = cell2table(rows, 'VariableNames', {'case','posRms','posMax','rollMaxDeg','pitchMaxDeg'});
writetable(T, fullfile(csvDir, 'r17_tilt_wind.csv'));

figDir = fullfile('research','results','fig');
if ~isfolder(figDir), mkdir(figDir); end
f = figure('Visible','off','Position',[100 100 760 330],'Color','w');
bar(cell2mat(rows(:,2))); grid on
set(gca, 'XTickLabel', cases(:,1));
ylabel('水平位置 RMS (m)'); title('倾转悬停抗风：水平积分的作用')
exportgraphics(f, fullfile(figDir,'r17_tilt_wind.png'), 'Resolution', 150);
close(f);

fprintf('\nreference: research/REPORT_r13_tilt_wind.md\n');
