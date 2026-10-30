function T = r21_transition_wind(tag)
%R21_TRANSITION_WIND  转捩段带侧风：ESO 是否还有用
%
%   T = r21_transition_wind(TAG)
%
%   第 17 轮。悬停段接 ESO 后抗风边界从 2 m/s 推到 5 m/s（r14），但转捩段
%   有气动升力参与，扰动来源变了：悬停时扰动主要是阻力，转捩时还有机翼
%   升力随迎角的变化。这一轮只量加速与巡航段（t=5..45 s），减速段本身
%   已由能量账单独处理（r3、r16），不混在一起看。
%
%   输入 TAG 用于命名输出，默认 'w2'.

if nargin < 1 || isempty(tag)
    tag = 'w2';
end

Wmag = str2double(regexprep(tag, '[^0-9.]', ''));
if isnan(Wmag), Wmag = 2; end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
ref = @(t) uav.env.tilt_ref(t);

cases = { ...
    'noeso', false; ...
    'eso',   true};

T = struct();
T.tag = tag;
T.wind = Wmag;
T.rows = zeros(numel(cases), 5);

for i = 1:size(cases, 1)
    s = struct('dt', 1e-3, 'ctrlDiv', 10, 'tEnd', 60, 'logDiv', 10);
    s.useEso = cases{i, 2};
    s.windFn = @(t) [Wmag; 0; 0];
    out = uav.sim.run_tilt(p, c, s, ref);

    idx = out.t >= 5 & out.t <= 45;
    pRef = zeros(nnz(idx), 3);
    k = 0;
    for j = find(idx).'
        k = k + 1;
        pRef(k, :) = ref(out.t(j)).p.';
    end
    dp = out.p(idx, :) - pRef;
    dh = -dp(:, 3);
    dxy = vecnorm(dp(:, 1:2), 2, 2);
    T.rows(i, :) = [max(dxy), max(abs(dh)), max(abs(rad2deg(out.eul(idx, 1)))), ...
                    max(abs(rad2deg(out.eul(idx, 2)))), max(out.beta(idx))];
    if i == 1, outNo = out; else, outEso = out; end
end

fprintf('\n== 转捩段侧风 %.1f m/s（t=5..45 s） ==\n', Wmag);
fprintf('  %-6s %10s %10s %10s %10s %10s\n', '控制器', '水平峰值', '高度峰值', '滚转', '俯仰', '短舱');
for i = 1:size(cases, 1)
    r = T.rows(i, :);
    fprintf('  %-6s %10.3f %10.3f %10.2f %10.2f %10.2f\n', cases{i, 1}, r(1), r(2), r(3), r(4), r(5));
end

csvDir = fullfile('research', 'results', 'csv');
figDir = fullfile('research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end

fid = fopen(fullfile(csvDir, ['r21_' tag '.csv']), 'w');
fprintf(fid, 'controller,hMax,dhMax,rollMax,pitchMax,betaMax\n');
for i = 1:size(cases, 1)
    r = T.rows(i, :);
    fprintf(fid, '%s,%.4f,%.4f,%.4f,%.4f,%.4f\n', cases{i, 1}, r(1), r(2), r(3), r(4), r(5));
end
fclose(fid);

fig = figure('Visible', 'off', 'Position', [100 100 900 400]);
subplot(1,2,1);
plot(outNo.p(:, 2), outNo.p(:, 1), 'b-', outEso.p(:, 2), outEso.p(:, 1), 'r-');
xlabel('东 (m)'); ylabel('北 (m)'); axis equal; grid on;
legend('无 ESO', 'ESO', 'Location', 'southwest'); title('俯视轨迹');
subplot(1,2,2);
plot(outNo.t, -outNo.p(:, 3), 'b-', outEso.t, -outEso.p(:, 3), 'r-');
xlabel('t (s)'); ylabel('高度 (m)'); grid on; title('高度');
exportgraphics(fig, fullfile(figDir, ['r21_' tag '.png']), 'Resolution', 130);
close(fig);
fprintf('  CSV/图已写：research/results/{csv,fig}/r21_%s.*\n', tag);
end
