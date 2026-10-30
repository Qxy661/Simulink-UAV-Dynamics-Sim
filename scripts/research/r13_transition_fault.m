% r13_transition_fault  过渡段（β≈45°）失效的存活带
%
%   REPORT_r8 只测了悬停（β=0）与巡航（β=90）两个端点，明确留了个洞：
%   过渡段里旋翼同时出竖直与前进分量，失效后既不能像悬停那样靠旋翼硬撑，
%   也不能像巡航那样靠机翼接管。这里扫过渡段失效，回答"什么时候断最危险"。
%
%   结果写 research/results/csv/r13_transition_fault.csv，
%   图写 research/results/fig/r13_transition_fault.png。

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(pwd));

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s0 = uav.cfg.simcfg();

% 任务：0-5 悬停，5-20 加速到 22，之后巡航。β 在 t≈11 s 到 45°、t≈14 s 到 90°。
cases = { ...
    % name,             kDead,  tFault
    'transition_none',   [],     11; ...
    'transition_one',    1,      11; ...
    'transition_adj',    [1 2],  11; ...
    'transition_diag',   [1 3],  11; ...
    'trans_early_one',   1,      8; ...
    'trans_late_one',    1,      14};

rows = {};
for i = 1:size(cases, 1)
    [name, kDead, tFault] = cases{i, :};

    s = s0;
    s.tEnd = 45; s.logDiv = 10;
    s.alloc = 'rankrpi';
    s.faultAware = true; s.faultHard = true;
    s.detect = false;
    if ~isempty(kDead)
        s.fault = struct('t', tFault, 'k', kDead, 'eta', zeros(numel(kDead), 1));
    end

    ref = @(t) uav.env.tilt_ref_v(t, 22);
    out = uav.sim.run_tilt(p, c, s, ref);

    post = out.t > tFault;
    errH = vecnorm(out.p(:, 1:2) - out.ref(:, 1:2), 2, 2);
    dh   = -out.p(:, 3) - out.ref(:, 5);
    tilt = max(abs(rad2deg(out.eul(:, 1:2))), [], 2);
    beta = rad2deg(out.beta);

    % 失效瞬间的短舱角，用来标定"断在哪个转化点上"
    kB = find(out.t >= tFault, 1);
    rows(end+1, :) = {name, tFault, beta(kB), max(errH(post)), max(abs(dh(post))), ...
                      max(tilt(post)), errH(end), dh(end), norm(out.v(end,:))}; %#ok<SAGROW>

    fprintf('%-16s beta@fault %5.1f  errH %7.1f  dh %7.2f  tilt %6.1f  VEnd %5.1f\n', ...
            name, beta(kB), max(errH(post)), max(abs(dh(post))), max(tilt(post)), ...
            norm(out.v(end,:)));
end

csvDir = fullfile('research', 'results', 'csv');
if ~isfolder(csvDir), mkdir(csvDir); end
T = cell2table(rows, 'VariableNames', {'caseName','tFault','betaAtFault','errHMax','dhMax', ...
    'tiltMax','errHEnd','dhEnd','vEnd'});
writetable(T, fullfile(csvDir, 'r13_transition_fault.csv'));

figDir = fullfile('research', 'results', 'fig');
if ~isfolder(figDir), mkdir(figDir); end
f = figure('Visible', 'off', 'Position', [100 100 900 380], 'Color', 'w');
subplot(1,2,1)
bar(cell2mat(rows(:, 5))); set(gca, 'XTick', 1:size(rows,1), 'XTickLabel', rows(:,1), ...
    'XTickLabelRotation', 35); grid on
ylabel('高度偏差峰值 (m)'); title('过渡段失效：高度')
subplot(1,2,2)
bar(cell2mat(rows(:, 6))); set(gca, 'XTick', 1:size(rows,1), 'XTickLabel', rows(:,1), ...
    'XTickLabelRotation', 35); grid on
ylabel('最大倾角 (deg)'); title('过渡段失效：姿态')
exportgraphics(f, fullfile(figDir, 'r13_transition_fault.png'), 'Resolution', 150);
close(f);

fprintf('\nreference: research/REPORT_r9_transition_fault.md\n');
