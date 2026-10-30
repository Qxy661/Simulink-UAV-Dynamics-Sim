function r5_decelsag(tag, hBot)
%R5_DECELSAG 减速段允许掉高时的往返测试（auto-research 第 7 轮）。
%
%   r5_decelsag                 hBot = 8 m（从 20 m 掉 12 m）
%   r5_decelsag('h12', 12)      只掉 8 m
%
%   指标（skill 纪律：稳态/瞬态分开报、单种子要说明是否随机）
%     - 高度偏差：全程峰值、减速段峰值、末端
%     - 空速：末端、减速段实际最大减速度 vs 参考最大减速度
%     - 姿态：最大俯仰/滚转
%     - 执行器：推力占比峰值、舵面峰值、是否饱和
%
%   本仿真确定性（无随机种子），同一配置逐位可重放，所以不做多种子。

if nargin < 1 || isempty(tag), tag = 'sag'; end
if nargin < 2 || isempty(hBot), hBot = 8; end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 80;
s.logDiv = 10;

ref = @(t) uav.env.tilt_ref_bt(t, 22, 20, hBot);
out = uav.sim.run_tilt(p, c, s, ref);

t   = out.t;
V   = vecnorm(out.v, 2, 2);
h   = -out.p(:, 3);
hR  = out.ref(:, 5);           % 参考高度（out.ref = [p(3) psi h V]）
dh  = h - hR;
bt  = rad2deg(out.beta);
th  = rad2deg(out.eul(:, 2));
ph  = rad2deg(out.eul(:, 1));
U1  = p.kt * sum(out.om.^2, 2);
U1f = U1 / (4*p.kt*p.wMax^2);

% 减速段窗口
w = t >= 40 & t <= 60;
% 参考减速度（对空速参考求数值导数）
Vref = out.ref(:, 6);
aRef = gradient(Vref, mean(diff(t)));
aAct = gradient(V, mean(diff(t)));

fprintf('\n== 带掉高的往返（%s，hBot=%.1f m）==\n', tag, hBot);
fprintf('末端：高度 %.2f m（参考 %.2f），空速 %.2f m/s，短舱 %.1f deg\n', ...
    h(end), hR(end), V(end), bt(end));
fprintf('高度偏差：全程峰值 %.2f m，减速段峰值 %.2f m，末端 %+.2f m\n', ...
    max(abs(dh)), max(abs(dh(w))), dh(end));
fprintf('减速度：参考峰值 %.2f m/s^2，实际峰值 %.2f m/s^2\n', ...
    max(abs(aRef(w))), max(abs(aAct(w))));
fprintf('姿态峰值：俯仰 %.1f deg，滚转 %.1f deg\n', max(abs(th)), max(abs(ph)));
fprintf('推力占比峰值 %.2f\n', max(U1f));

fprintf('\n%6s %7s %7s %8s %8s %7s %7s\n', ...
    't', 'V', 'Vref', 'h', 'hRef', 'beta', 'theta');
tt = [38 42 46 50 54 58 62 70 80];
for k = tt
    [~, j] = min(abs(t - k));
    fprintf('%6.1f %7.2f %7.2f %8.2f %8.2f %7.1f %7.1f\n', ...
        t(j), V(j), Vref(j), h(j), hR(j), bt(j), th(j));
end

% ---- 产物 ----
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
csvDir = fullfile(root, 'research', 'results', 'csv');
figDir = fullfile(root, 'research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end

T = table(t, V, Vref, h, hR, dh, bt, th, ph, U1f, ...
    'VariableNames', {'t_s','V_mps','Vref_mps','h_m','hRef_m','dh_m', ...
                      'beta_deg','theta_deg','phi_deg','U1_frac'});
writetable(T, fullfile(csvDir, ['r5_' tag '.csv']));

f = figure('Color', 'w', 'Position', [100 100 980 560]);
subplot(2,1,1);
plot(t, h, 'b', t, hR, 'r--'); grid on; ylabel('高度 (m)');
legend('实际', '参考', 'Location', 'best');
title(sprintf('减速段掉高 %.0f m：高度与空速', 20-hBot));
subplot(2,1,2);
yyaxis left; plot(t, V, 'b', t, Vref, 'r--'); ylabel('空速 (m/s)');
yyaxis right; plot(t, bt, 'k'); ylabel('\beta (deg)');
grid on; xlabel('t (s)');
exportgraphics(f, fullfile(figDir, ['r5_' tag '.png']));
fprintf('\n写入 research/results/csv/r5_%s.csv\n', tag);
end
