function T = r22_ctrl_sensitivity(tag)
% 控制输入摄动敏感度：估计器差多少会出事
%
% 前面的轮次（r13、r18）测的都是"真值状态直接进控制器"。真实飞控里
% 控制器吃的是估计值，所以要知道姿态偏置与位置噪声各能容忍多少。
% 这里只摄动控制器看到的量（run_tilt 的 s.pert），不动机体真值：
%   attBias  估计姿态的小角偏置（弧度），三个工况：滚转/俯仰各给一半
%   posNoise 位置白噪标准差（米）
%
% 用法：
%   T = r22_ctrl_sensitivity('hover')      % 30 s 悬停
%   T = r22_ctrl_sensitivity('transition') % 22 m/s 全任务

if nargin < 1, tag = 'hover'; end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = struct('dt', 1e-3, 'ctrlDiv', 1, 'tEnd', 30, 'logDiv', 20);

switch tag
    case 'hover'
        % tilt 构型的参考必须带 V/h 两个字段（tilt_cascade 的调度与高度环要用），
        % uav.env.hover 只给 p/v/a/psi，直接传会报"无法识别字段名称 V"。
        ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                          'psi', 0, 'h', 20, 'V', 0); %#ok<NASGU>
        % 悬停时 20 m 高度、零速度，指标看末段漂移与姿态抖动
        metric = @(out) [max(vecnorm(out.p(:,1:2) - out.ref(:,1:2), 2, 2)); ...
                         max(vecnorm(rad2deg(out.eul(:,1:2)), 2, 2))];
    case 'transition'
        s.tEnd = 45;
        ref = @(t) uav.env.tilt_ref(t);
        % 转捩看减速段之前的高度峰值、末端高度与末端倾角
        idx = @(out) out.t >= 5 & out.t <= 40;
        metric = @(out) [max(abs(out.p(idx(out),3) - out.ref(idx(out),3))); ...
                         abs(out.p(end,3) - out.ref(end,3)); ...
                         max(vecnorm(rad2deg(out.eul(:,1:2)), 2, 2))];
    otherwise
        error('tag 只能是 hover 或 transition');
end

attList = [0 0.5 1 2 5] * pi/180;
noiseList = [0 0.05 0.20];

rows = {};
fprintf('%-6s %-8s %10s %10s\n', 'att°', 'posNoise', 'metric1', 'metric2');
for a = attList
    for n = noiseList
        s.pert = struct('attBias', [a; a*0.5; 0], 'posNoise', n, 'posBias', zeros(3,1));
        out = uav.sim.run_tilt(p, c, s, ref);
        m = metric(out);
        rows(end+1, :) = {tag, a*180/pi, n, m}; %#ok<AGROW>
        fprintf('%-6.1f %-8.2f %10.3f %10.3f\n', a*180/pi, n, m(1), m(2));
        drawnow limitrate
    end
end

T = rows;
if ~isfolder('research/results/csv'), mkdir('research/results/csv'); end
fid = fopen(fullfile('research/results/csv', ['r22_' tag '.csv']), 'w');
fprintf(fid, 'tag,attDeg,posNoise,metric1,metric2\n');
for i = 1:size(rows,1)
    m = rows{i,4};
    fprintf(fid, '%s,%.3f,%.3f,%.6g,%.6g\n', rows{i,1}, rows{i,2}, rows{i,3}, m(1), m(2));
end
fclose(fid);
end
