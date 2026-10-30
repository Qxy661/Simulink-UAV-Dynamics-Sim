% r12_multi_fault  双桨失效的存活边界
%
%   单桨失效的结论（REPORT_r6）：悬停需要推力裕度 >= 2，巡航靠机翼与舵面
%   能撑住。双桨失效还剩什么？分两种几何：相邻（同一侧的滚转力矩无法配平）
%   与对角（只剩一对同向推力）。分别在悬停（t=8 s）与 22 m/s 巡航（t=30 s）
%   切断，指标是相对参考的偏差，不是飞了多远。
%
%   结果写 research/results/csv/r12_multi_fault.csv，
%   图写 research/results/fig/r12_multi_fault.png。

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(pwd));

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s0 = uav.cfg.simcfg();

cases = { ...
    % name,              Vmax, kDead, tFault, tEnd
    'hover_none',        0,    [],    0,     25; ...
    'hover_one',         0,    1,     8,     25; ...
    'hover_adjacent',    0,    [1 2], 8,     25; ...
    'hover_diagonal',    0,    [1 3], 8,     25; ...
    'cruise_none',       22,   [],    30,    45; ...
    'cruise_adjacent',   22,   [1 2], 30,    45; ...
    'cruise_diagonal',   22,   [1 3], 30,    45};

rows = {};
for i = 1:size(cases, 1)
    [name, Vmax, kDead, tFault, tEnd] = cases{i, :};

    s = s0;
    s.tEnd = tEnd; s.logDiv = 10;
    s.alloc = 'rankrpi';
    s.faultAware = true; s.faultHard = true;
    s.detect = false;
    if ~isempty(kDead)
        s.fault = struct('t', tFault, 'k', kDead, 'eta', zeros(numel(kDead), 1));
    end

    if Vmax == 0
        h0 = 20;
        ref = @(t) struct('p', [0;0;-h0], 'v', zeros(3,1), 'a', zeros(3,1), ...
                          'psi', 0, 'h', h0, 'V', 0);
    else
        ref = @(t) uav.env.tilt_ref_v(t, Vmax);
    end

    out = uav.sim.run_tilt(p, c, s, ref);

    % 相对参考的位置偏差：ref 6 列 [p(3) psi h V]
    errH = vecnorm(out.p(:, 1:2) - out.ref(:, 1:2), 2, 2);
    dh   = -out.p(:, 3) - out.ref(:, 5);
    tilt = max(abs(rad2deg(out.eul(:, 1:2))), [], 2);

    if isempty(kDead), tOn = tEnd; else, tOn = tFault; end
    post = out.t > max(tOn, 1);

    rows(end+1, :) = {name, Vmax, tFault, ...
                      max(errH(post)), max(abs(dh(post))), max(tilt(post)), ...
                      errH(end), dh(end), norm(out.v(end, :)), ...
                      max(abs(rad2deg(out.w(post, 3))))}; %#ok<SAGROW>

    fprintf('%-16s errHMax %7.1f  dhMax %6.2f  tiltMax %6.1f  errHEnd %6.1f  dhEnd %6.2f  VEnd %5.1f\n', ...
            name, max(errH(post)), max(abs(dh(post))), max(tilt(post)), ...
            errH(end), dh(end), norm(out.v(end, :)));
end

csvDir = fullfile('research', 'results', 'csv');
if ~isfolder(csvDir), mkdir(csvDir); end
T = cell2table(rows, 'VariableNames', {'caseName','Vmax','tFault','errHMax','dhMax', ...
    'tiltMax','errHEnd','dhEnd','vEnd','yawRateMax'});
writetable(T, fullfile(csvDir, 'r12_multi_fault.csv'));

figDir = fullfile('research', 'results', 'fig');
if ~isfolder(figDir), mkdir(figDir); end
f = figure('Visible', 'off', 'Position', [100 100 900 380], 'Color', 'w');
subplot(1,2,1)
bar(cell2mat(rows(:, 4))); set(gca, 'XTick', 1:size(rows,1), 'XTickLabel', rows(:,1), ...
    'XTickLabelRotation', 35); grid on
ylabel('水平偏差峰值 (m)'); title('双桨失效：水平偏差')
subplot(1,2,2)
bar(cell2mat(rows(:, 5))); set(gca, 'XTick', 1:size(rows,1), 'XTickLabel', rows(:,1), ...
    'XTickLabelRotation', 35); grid on
ylabel('高度偏差峰值 (m)'); title('双桨失效：高度偏差')
exportgraphics(f, fullfile(figDir, 'r12_multi_fault.png'), 'Resolution', 150);
close(f);

fprintf('\nreference: research/REPORT_r8_multifault.md\n');
