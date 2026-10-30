function r9_detect_fault()
%R9_DETECT_FAULT 失效检测：不告诉控制器 η，只看转速残差，能不能追上 oracle。
%
%   任务 tilt_ref_v(22)，45 s，t=25 s 让 1 号电机降效到 0.35。
%   三种策略：oracle（已知 η，硬判死）/ detected（残差检测）/ soft（按真实 η 分配）。
%   输出 research/results/csv/r9_detect_fault.csv，并画轨迹对比图。
%
%   结论见 research/REPORT_r5_detection.md：检测延迟 40 ms，漂移与 oracle 相同 2.16 m。

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
csvDir = fullfile(root, 'research', 'results', 'csv');
figDir = fullfile(root, 'research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end

p  = uav.cfg.tilt();
c  = uav.cfg.tiltctrl();
s0 = uav.cfg.simcfg();
s0.tEnd    = 45;
s0.logDiv  = 10;
s0.fault   = struct('t', 25, 'k', 1, 'eta', 0.35);
s0.faultAware = true;

ref    = @(t) uav.env.tilt_ref_v(t, 22);
labels = {'oracle', 'detected', 'soft'};
rows   = zeros(3, 4);
outs   = cell(1, 3);

for mode = 1:3
    s = s0;
    s.faultHard = (mode == 1);
    s.detect    = (mode == 2);
    s.logFault  = (mode == 2);
    out = uav.sim.run_tilt(p, c, s, ref);
    outs{mode} = out;

    post = out.t > 25;
    ep   = vecnorm(out.p(post,1:2) - out.ref(post,1:2), 2, 2);
    rows(mode, :) = [max(ep), max(abs(rad2deg(out.w(post,3)))), ...
                     max(abs(-out.p(post,3) - out.ref(post,5))), ...
                     max(abs(rad2deg(out.eul(post,1))))];
    fprintf('%-9s drift %5.2f m  yawrate %5.2f deg/s  dh %4.2f m  roll %4.2f deg\n', ...
        labels{mode}, rows(mode,1), rows(mode,2), rows(mode,3), rows(mode,4));
end

T = table(labels(:), rows(:,1), rows(:,2), rows(:,3), rows(:,4), ...
    'VariableNames', {'strategy', 'drift_m', 'yawrate_dps', 'dh_m', 'roll_deg'});
writetable(T, fullfile(csvDir, 'r9_detect_fault.csv'));

f = figure('Color','w','Position',[90 90 900 620]);
tiledlayout(2,1,'Padding','compact');
nexttile
hold on
for k = 1:3
    plot(outs{k}.p(:,2), outs{k}.p(:,1), 'LineWidth', 1.1);
end
plot(outs{1}.ref(:,2), outs{1}.ref(:,1), 'k--');
grid on; axis equal; xlabel('东 (m)'); ylabel('北 (m)');
legend([labels, {'参考'}], 'Location', 'best');
title('η=0.35，t=25 s 故障：三种策略的水平轨迹');
nexttile
hold on
for k = 1:3
    plot(outs{k}.t, vecnorm(outs{k}.p(:,1:2) - outs{k}.ref(:,1:2), 2, 2), 'LineWidth', 1.1);
end
grid on; xline(25, 'k:'); xlabel('t (s)'); ylabel('水平位置误差 (m)');
legend(labels, 'Location', 'best');
exportgraphics(f, fullfile(figDir, 'r9_detect_fault.png'), 'Resolution', 150);
close(f);
end
