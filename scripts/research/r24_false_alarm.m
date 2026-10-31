function T = r24_false_alarm(tag)
% 失效检测在阵风下的误报率
%
% r9 的检测器靠"指令转速 - 实测转速"残差判死（阈值 60 rad/s、连续 3 拍）。
% 阵风里控制器为了抗风会顶转速，残差同样会变大——如果误判，飞控会把一台
% 好电机当死机处理，比故障本身更危险。这一轮量误报率。
%
% 用法：T = r24_false_alarm('hover')

if nargin < 1, tag = 'hover'; end
if ~strcmp(tag, 'hover'), error('目前只做了 hover'); end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
windList = [0 2 4 6 8];       % 常值北向风（m/s）
rows = cell(numel(windList), 4);

ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                  'psi', 0, 'h', 20, 'V', 0);

fprintf('%-8s %10s %10s %12s\n', 'wind', 'falseAlarm', 'drift', 'tiltMax');
for i = 1:numel(windList)
    w = windList(i);
    s = struct('dt', 1e-3, 'ctrlDiv', 1, 'tEnd', 20, 'logDiv', 20);
    s.useEso = true;
    s.detect = true;
    s.windFn = @(t) [w; 0; 0];
    out = uav.sim.run_tilt(p, c, s, ref);
    nFA = numel(out.det.t);          % 全程无故障，任何检测都是误报
    tail = out.t > 14;
    drift = max(vecnorm(out.p(tail,1:2), 2, 2));
    tiltMax = max(vecnorm(rad2deg(out.eul(:,1:2)), 2, 2));
    rows(i,:) = {tag, w, nFA, drift};
    fprintf('%-8.1f %10d %10.3f %12.2f\n', w, nFA, drift, tiltMax);
    drawnow limitrate
end

T = rows;
if ~isfolder('research/results/csv'), mkdir('research/results/csv'); end
fid = fopen(fullfile('research/results/csv', ['r24_' tag '.csv']), 'w');
fprintf(fid, 'tag,wind,falseAlarms,drift\n');
for i = 1:size(rows,1)
    fprintf(fid, '%s,%.1f,%d,%.6g\n', rows{i,1}, rows{i,2}, rows{i,3}, rows{i,4});
end
fclose(fid);
end
