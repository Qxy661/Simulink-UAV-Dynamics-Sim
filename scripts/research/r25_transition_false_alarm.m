function T = r25_transition_false_alarm()
% 转捩段的检测误报：短舱过渡时转速从 500 拉到 1100，残差会不会撞阈值
%
% 悬停段（r21）零误报，但转捩段完全是另一回事：指令转速在 15 s 内翻倍，
% 电机一阶滞后本身就会让"指令 - 实测"差出上百 rad/s。检测器只看残差，
% 分不清"追不上指令"和"电机死了"。这一轮量一下到底误不误报。

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
windList = [0 2 4];
rows = cell(numel(windList), 4);

fprintf('%-8s %10s %12s %12s\n', 'wind', 'events', 'firstT', 'finalV');
for i = 1:numel(windList)
    w = windList(i);
    s = struct('dt', 1e-3, 'ctrlDiv', 1, 'tEnd', 45, 'logDiv', 20);
    s.detect = true;
    s.windFn = @(t) [w; 0; 0];
    out = uav.sim.run_tilt(p, c, s, @uav.env.tilt_ref);
    nDet = numel(out.det.t);
    if nDet > 0
        firstT = out.det.t(1);
    else
        firstT = NaN;
    end
    finalV = norm(out.v(end, :));
    rows(i,:) = {'transition', w, nDet, firstT};
    fprintf('%-8.1f %10d %12.3f %12.3f\n', w, nDet, firstT, finalV);
    drawnow limitrate
end

T = rows;
if ~isfolder('research/results/csv'), mkdir('research/results/csv'); end
fid = fopen('research/results/csv/r25_transition.csv', 'w');
fprintf(fid, 'tag,wind,events,firstT\n');
for i = 1:size(rows,1)
    fprintf(fid, '%s,%.1f,%d,%.6g\n', rows{i,1}, rows{i,2}, rows{i,3}, rows{i,4});
end
fclose(fid);
end
