function T = r23_lag_budget(tag)
% 姿态量测延迟的容忍度：估计器相位滞后能吃掉多少裕度
%
% r19 的结论是"精度不是瓶颈，相位才是"，这一轮把它量出来：
% 给控制器喂滞后 attDelayTicks 个控制拍的四元数（1 kHz 下 1 拍 = 1 ms），
% 看悬停漂移与转捩指标在哪个延迟上崩。
%
% 用法：
%   T = r23_lag_budget('hover')
%   T = r23_lag_budget('transition')

if nargin < 1, tag = 'hover'; end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();

lagList = [0 2 5 10 15 20 30];   % 控制拍（1 ms）
x0 = [];
rows = cell(numel(lagList), 4);

switch tag
    case 'hover'
        s = struct('dt', 1e-3, 'ctrlDiv', 1, 'tEnd', 20, 'logDiv', 20);
        ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                          'psi', 0, 'h', 20, 'V', 0);
        % 从 2 m 偏差起飞：精确平衡点上延迟没有任何东西可放大，
        % 必须先偏离平衡点才能看见相位裕度的差别。
        x0 = [2; 0; -20; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0; ...
              p.betaMin; p.hoverW*ones(4,1); 0; 0; 0];
    case 'transition'
        s = struct('dt', 1e-3, 'ctrlDiv', 1, 'tEnd', 45, 'logDiv', 20);
        ref = @uav.env.tilt_ref;
    otherwise
        error('tag 只能是 hover 或 transition');
end

fprintf('%-8s %12s %12s %10s\n', 'lag(ms)', 'metric1', 'metric2', 'tiltMax');
for i = 1:numel(lagList)
    lag = lagList(i);
    si = s;
    si.pert = struct('attDelayTicks', lag, 'attBias', zeros(3,1), ...
                     'posNoise', 0, 'posBias', zeros(3,1));
    try
        out = uav.sim.run_tilt(p, c, si, ref, x0);
        tail = out.t > 0.7*s.tEnd;
        if strcmp(tag, 'hover')
            m1 = max(vecnorm(out.p(tail,1:2), 2, 2));
            m2 = max(abs(out.p(tail,3) - ref(0).p(3)));
        else
            idx = out.t >= 5 & out.t <= 40;
            m1 = max(abs(out.p(idx,3) - out.ref(idx,3)));
            m2 = abs(out.p(end,3) - out.ref(end,3));
        end
        tiltMax = max(vecnorm(rad2deg(out.eul(:,1:2)), 2, 2));
        rows(i,:) = {tag, lag, m1, m2};
        fprintf('%-8d %12.3f %12.3f %10.2f\n', lag, m1, m2, tiltMax);
    catch err
        rows(i,:) = {tag, lag, NaN, NaN};
        fprintf('%-8d %12s  %s\n', lag, 'FAILED', err.message);
    end
    drawnow limitrate
end

T = rows;
if ~isfolder('research/results/csv'), mkdir('research/results/csv'); end
fid = fopen(fullfile('research/results/csv', ['r23_' tag '.csv']), 'w');
fprintf(fid, 'tag,lagMs,metric1,metric2\n');
for i = 1:size(rows,1)
    fprintf(fid, '%s,%d,%.6g,%.6g\n', rows{i,1}, rows{i,2}, rows{i,3}, rows{i,4});
end
fclose(fid);
end
