% r15_yawtrim  差动短舱补偏航：能否用四台短舱角的差异做偏航？
%
%   单桨失效后偏航通道秩亏损，机体自转，推力方向被航向带着转。
%   这里让四台短舱角不再一致（beta_i = beta + trim_i），trim 由三约束小方程解：
%       Σ T_i t_i = 0、Σ rx_i T_i cosβ t_i = 0、-Σ ry_i T_i cosβ t_i = uYaw
%   结论见 research/REPORT_r11_yawtrim.md：自转被压住，但位置漂移变大。
%
%   结果写 research/results/csv/r15_yawtrim.csv，
%   图写 research/results/fig/r15_yawtrim.png。

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(pwd));

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();

ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                  'psi', 0, 'h', 20, 'V', 0);

base = uav.cfg.simcfg();
base.tEnd = 25; base.logDiv = 20;
base.alloc = 'rankrpi'; base.faultAware = true; base.faultHard = true;
base.fault = struct('t', 8, 'k', 1, 'eta', 0);

cases = {'off', NaN; 'kp0.4', 0.4; 'kp0.8', 0.8; 'kp1.2', 1.2};
rows = {};
for i = 1:size(cases, 1)
    s = base;
    if isfinite(cases{i, 2})
        s.yawTrim = struct('enable', true, 'kp', cases{i,2}, 'kd', 0.25, ...
                           'maxTrim', 12*pi/180);
    end
    out = uav.sim.run_tilt(p, c, s, ref);
    post = out.t > 8;
    roll  = max(abs(rad2deg(out.eul(post, 1))));
    drift = max(vecnorm(out.p(post, 1:2), 2, 2));
    hMin  = min(-out.p(post, 3));
    rmsR  = sqrt(mean(out.w(post, 3).^2));
    psiEnd = rad2deg(out.eul(end, 3));
    rows(end+1, :) = {cases{i,1}, roll, drift, hMin, rmsR, psiEnd}; %#ok<SAGROW>
    fprintf('%-6s 滚转 %5.1f°  漂移 %6.1f m  最低高度 %5.2f m  偏航率RMS %.3f  末端航向 %6.1f°\n', ...
            cases{i,1}, roll, drift, hMin, rmsR, psiEnd);
end

csvDir = fullfile('research', 'results', 'csv');
if ~isfolder(csvDir), mkdir(csvDir); end
T = cell2table(rows, 'VariableNames', {'case','rollMax','driftMax','hMin','yawRateRms','psiEnd'});
writetable(T, fullfile(csvDir, 'r15_yawtrim.csv'));

figDir = fullfile('research', 'results', 'fig');
if ~isfolder(figDir), mkdir(figDir); end
f = figure('Visible', 'off', 'Position', [100 100 820 330], 'Color', 'w');
subplot(1,2,1); hold on
plot(cell2mat(rows(:,4)), cell2mat(rows(:,2))*-1, 'o'); hold off; grid on
xlabel('最低高度 (m)'); ylabel('最大滚转 (°)'); title('姿态与高度')
subplot(1,2,2); hold on
plot(cell2mat(rows(:,5)), cell2mat(rows(:,3)), 's'); hold off; grid on
xlabel('偏航角速度 RMS (rad/s)'); ylabel('漂移峰值 (m)'); title('自转越小，漂移越大')
exportgraphics(f, fullfile(figDir, 'r15_yawtrim.png'), 'Resolution', 150);
close(f);

fprintf('\nreference: research/REPORT_r11_yawtrim.md\n');
