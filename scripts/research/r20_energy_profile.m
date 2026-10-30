function T = r20_energy_profile(tag)
%R20_ENERGY_PROFILE  减速段沿能量轨迹的可行性
%
%   T = r20_energy_profile(TAG)
%
%   第 20 轮：前几轮证明"定高减速"在物理上不成立——22 m/s 平飞时
%   阻力只有 12 N，要把速度降下来必然抬头，抬头产生的升力只能把
%   机体顶高（research/REPORT_r3_energy.md：峰值高度偏差 ≈ 0.7*V^2/2g）。
%
%   这一轮换一个问法：如果不要求定高，而是让高度按能量守恒走
%
%       h = h0 + eta*(Vmax^2 - V^2)/(2g)
%
%   控制器能不能贴着这条参考飞完减速段？如果能，减速段的问题就从
%   "控制失败"变成"任务设计不允许"。
%
%   输入 TAG 仅用于命名输出文件，默认 'energy'.

if nargin < 1 || isempty(tag)
    tag = 'energy';
end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
Vmax = 22;
h0 = 20;
eta = 0.7;

s = struct('dt', 1e-3, 'ctrlDiv', 10, 'tEnd', 60, 'logDiv', 10);
s.useEso = false;

ref = @(t) uav.env.tilt_ref_energy(t, Vmax, h0, eta);
out = uav.sim.run_tilt(p, c, s, ref);

% 参考量与观测量
hRef = arrayfun(@(k) ref(out.t(k)).h, 1:numel(out.t)).';
h = -out.p(:, 3);
V = sqrt(sum(out.v.^2, 2));

t3 = 5 + 15 + 10;          % 减速开始
t4 = t3 + 20;              % 减速结束
idec = out.t >= t3 & out.t <= t4;
ihov = out.t > t4 + 2;

dh = h - hRef;
T = struct();
T.tag = tag;
T.Vmax = Vmax;
T.eta = eta;
T.h0 = h0;
T.hEnd_ref = h0 + eta * Vmax^2 / (2*9.81);
T.dhDecelMax = max(abs(dh(idec)));
T.dhFinal = max(abs(dh(ihov)));
T.hFinal = h(end);
T.vFinal = V(end);
T.tiltMax = max(abs(rad2deg(out.eul(:, 1))));
T.betaMax = max(out.beta);
T.peakErrConstantH = 23.81;   % r5 定高减速的实测峰值，用作对照

fprintf('\n== 沿能量轨迹减速（eta=%.2f） ==\n', eta);
fprintf('  减速段高度偏差峰值   %.2f m（定高方案 23.81 m）\n', T.dhDecelMax);
fprintf('  末段悬停高度偏差     %.2f m\n', T.dhFinal);
fprintf('  末端高度             %.2f m（参考 %.2f）\n', T.hFinal, T.hEnd_ref);
fprintf('  末端速度             %.2f m/s\n', T.vFinal);
fprintf('  最大滚转             %.2f deg\n', T.tiltMax);
fprintf('  最大短舱角           %.1f deg\n', T.betaMax);

% 落盘
csvDir = fullfile('research', 'results', 'csv');
figDir = fullfile('research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end

csvFile = fullfile(csvDir, ['r20_' tag '.csv']);
fid = fopen(csvFile, 'w');
fprintf(fid, 't,h,hRef,dh,V,Vref,beta,roll,pitch\n');
for k = 1:numel(out.t)
    rk = ref(out.t(k));
    fprintf(fid, '%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
        out.t(k), h(k), hRef(k), dh(k), V(k), rk.V, out.beta(k), ...
        rad2deg(out.eul(k,1)), rad2deg(out.eul(k,2)));
end
fclose(fid);

fig = figure('Visible', 'off', 'Position', [100 100 900 620]);

subplot(2,2,1);
plot(out.t, hRef, 'k--', out.t, h, 'b-');
ylabel('高度 (m)'); xlabel('t (s)'); grid on;
legend('参考（能量轨迹）', '实际', 'Location', 'northwest');
title('高度跟随');

subplot(2,2,2);
plot(out.t, dh, 'b-'); ylim([-15 15]);
ylabel('高度偏差 (m)'); xlabel('t (s)'); grid on;
yline(0, 'k:'); title('高度偏差');

subplot(2,2,3);
plot(out.t, V, 'b-', out.t, arrayfun(@(k) ref(out.t(k)).V, 1:numel(out.t)), 'k--');
ylabel('空速 (m/s)'); xlabel('t (s)'); grid on; title('空速');

subplot(2,2,4);
plot(out.t, rad2deg(out.beta), 'b-');
ylabel('短舱角 (deg)'); xlabel('t (s)'); grid on; title('短舱角');

figFile = fullfile(figDir, ['r20_' tag '.png']);
exportgraphics(fig, figFile, 'Resolution', 130);
close(fig);

fprintf('  CSV: %s\n', csvFile);
fprintf('  图 : %s\n', figFile);
end
