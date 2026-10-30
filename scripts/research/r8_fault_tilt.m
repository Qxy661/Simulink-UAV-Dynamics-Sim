function r8_fault_tilt(tag)
%R8_FAULT_TILT 倾转构型在巡航段单电机降效/失效后的重构能力。
%
%   r8_fault_tilt           默认跑 eta=[1 0.6 0.35 0] 四档
%   r8_fault_tilt('eta35')  只跑一档，用于调试
%
%   任务：tilt_ref_v(22)，45 s。t=25 s 时 1 号电机（右前）降效到 eta。
%   指标：故障后水平偏离、高度损失、滚转/俯仰漂移、偏航角速度、饱和占比、
%   是否存活到 45 s。eta=1 是基线。
%
%   分配用 uav.plant.tilt_mix_fault：B*diag(act) 的伪逆，秩 3，
%   偏航通道不可控，靠姿态环残余权限压。

if nargin < 1 || isempty(tag), tag = 'sweep'; end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s0 = uav.cfg.simcfg();
s0.tEnd = 45;
s0.logDiv = 10;
s0.ctrlDiv = 10;

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
csvDir = fullfile(root, 'research', 'results', 'csv');
figDir = fullfile(root, 'research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end

if strcmp(tag, 'eta35')
    etas = 0.35;
else
    etas = [1 0.6 0.35 0];
end
hardFlags = [false true];

ref = @(t) uav.env.tilt_ref_v(t, 22);
fprintf('\n%6s %6s %9s %9s %9s %9s %9s %8s\n', 'eta', 'hard', 'dhMax', 'driftMax', 'rollMax', 'pitchMax', 'yawRate', 'survive');
res = struct('eta', {}, 'hard', {}, 'dhMax', {}, 'driftMax', {}, 'rollMax', {}, ...
    'pitchMax', {}, 'yawRate', {}, 'survive', {});

for i = 1:numel(etas)
  for j = 1:numel(hardFlags)
    s = s0;
    s.fault = struct('t', 25, 'k', 1, 'eta', etas(i));
    s.faultAware = true;
    s.faultHard  = hardFlags(j);
    out = uav.sim.run_tilt(p, c, s, ref);

    t  = out.t;
    post = t > 25;
    h  = -out.p(:, 3);
    hR = out.ref(:, 5);
    dh = h - hR;
    ep = vecnorm(out.p(post, 1:2) - out.ref(post, 1:2), 2, 2);
    rollMax  = max(abs(rad2deg(out.eul(post, 1))));
    pitchMax = max(abs(rad2deg(out.eul(post, 2))));
    yawRate  = max(abs(out.w(post, 3)));
    survive  = all(isfinite(out.x(end, :))) && abs(dh(end)) < 5 && out.p(end, 1) > 0;

    fprintf('%6.2f %6d %9.2f %9.2f %9.2f %9.2f %9.2f %8d\n', ...
        etas(i), hardFlags(j), max(abs(dh(post))), max(ep), rollMax, pitchMax, ...
        rad2deg(yawRate), survive);

    res(end+1) = struct('eta', etas(i), 'hard', hardFlags(j), ...
        'dhMax', max(abs(dh(post))), 'driftMax', max(ep), 'rollMax', rollMax, ...
        'pitchMax', pitchMax, 'yawRate', rad2deg(yawRate), 'survive', survive); %#ok<AGROW>

    if numel(etas) == 1 && j == numel(hardFlags)
        f = figure('Color','w','Position',[90 90 950 640]);
        subplot(3,1,1); plot(t, h, t, hR, '--'); grid on; ylabel('高度 (m)'); legend('实际','参考');
        subplot(3,1,2); plot(t, out.p(:,1), t, out.ref(:,1), '--'); grid on; ylabel('北向位置 (m)');
        subplot(3,1,3);
        yyaxis left; plot(t, rad2deg(out.eul(:,1))); ylabel('滚转 (deg)');
        yyaxis right; plot(t, out.om(:,1)); ylabel('\omega_1 (rad/s)'); grid on; xlabel('t (s)');
        exportgraphics(f, fullfile(figDir, ['r8_fault_tilt_' tag '.png']));
    end
  end
end

T = struct2table(res);
writetable(T, fullfile(csvDir, ['r8_fault_tilt_' tag '.csv']));
fprintf('\n写入 research/results/csv/r8_fault_tilt_%s.csv\n', tag);
end
