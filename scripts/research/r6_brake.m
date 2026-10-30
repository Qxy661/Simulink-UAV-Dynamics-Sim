function r6_brake(mode, Vmax)
%R6_BRAKE 减速模态（uav.ctrl.tilt_brake）的往返测试。
%
%   r6_brake('level')          定高减速（20 m 保持）
%   r6_brake('sag')            减速段允许掉到 8 m
%   r6_brake(mode, Vmax)       改巡航速度
%
%   指标：末端位置/高度/空速、高度偏差峰值（全程/减速段）、实际最大减速度、
%   姿态峰值、短舱轨迹、推力占比峰值。

if nargin < 1 || isempty(mode), mode = 'sag'; end
if nargin < 2 || isempty(Vmax), Vmax = 22; end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 85;
s.logDiv = 10;
s.brake = true;

switch mode
    case 'level'
        ref = @(t) uav.env.tilt_ref_v(t, Vmax);
    otherwise
        ref = @(t) uav.env.tilt_ref_bt(t, Vmax, 20, 8);
end

out = uav.sim.run_tilt(p, c, s, ref);
t  = out.t;
V  = vecnorm(out.v, 2, 2);
h  = -out.p(:, 3);
hR = out.ref(:, 5);
Vr = out.ref(:, 6);
dh = h - hR;
bt = rad2deg(out.beta);
th = rad2deg(out.eul(:, 2));
U1 = p.kt*sum(out.om.^2, 2);
U1f = U1/(4*p.kt*p.wMax^2);

% 减速窗口：参考速度开始下降的那一段
iDec = find(Vr < Vmax*0.99 & t > 10, 1);
if isempty(iDec), iDec = 1; end
wDec = t >= t(iDec) & t <= t(iDec) + 25;
aRef = gradient(Vr, mean(diff(t)));
aAct = gradient(V,  mean(diff(t)));

fprintf('\n== 减速模态（%s, Vmax=%g）==\n', mode, Vmax);
fprintf('末端：位置 [%.2f %.2f %.2f]，高度误差 %+.2f m，空速 %.2f m/s，β %.1f°\n', ...
    out.p(end,1), out.p(end,2), out.p(end,3), dh(end), V(end), bt(end));
fprintf('高度偏差：全程峰值 %.2f m，减速段峰值 %.2f m\n', max(abs(dh)), max(abs(dh(wDec))));
fprintf('减速度：参考峰值 %.2f，实际峰值 %.2f m/s^2\n', max(abs(aRef(wDec))), max(abs(aAct(wDec))));
fprintf('姿态峰值：俯仰 %.1f°，滚转 %.1f°；推力占比峰值 %.2f\n', ...
    max(abs(th)), max(abs(rad2deg(out.eul(:,1)))), max(U1f));
fprintf('末端空速误差 %.2f m/s，末端高度 %.2f m（参考 %.2f）\n', V(end)-Vr(end), h(end), hR(end));

fprintf('\n%6s %7s %7s %8s %8s %7s %7s\n', 't','V','Vref','h','hRef','beta','theta');
for kk = [30 38 44 48 52 56 60 66 74 85]
    [~, j] = min(abs(t - kk));
    fprintf('%6.1f %7.2f %7.2f %8.2f %8.2f %7.1f %7.1f\n', ...
        t(j), V(j), Vr(j), h(j), hR(j), bt(j), th(j));
end

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
csvDir = fullfile(root, 'research', 'results', 'csv');
figDir = fullfile(root, 'research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end
tag = sprintf('brake_%s_%g', mode, Vmax);
T = table(t, V, Vr, h, hR, dh, bt, th, U1f, ...
    'VariableNames', {'t_s','V_mps','Vref_mps','h_m','hRef_m','dh_m','beta_deg','theta_deg','U1_frac'});
writetable(T, fullfile(csvDir, ['r6_' tag '.csv']));

f = figure('Color','w','Position',[100 100 980 620]);
subplot(3,1,1); plot(t,h,'b',t,hR,'r--'); grid on; ylabel('高度 (m)'); legend('实际','参考','Location','best');
subplot(3,1,2); plot(t,V,'b',t,Vr,'r--'); grid on; ylabel('空速 (m/s)');
subplot(3,1,3); yyaxis left; plot(t,bt,'b'); ylabel('\beta (deg)');
yyaxis right; plot(t,th,'r'); ylabel('\theta (deg)'); grid on; xlabel('t (s)');
exportgraphics(f, fullfile(figDir, ['r6_' tag '.png']));
fprintf('\n写入 research/results/csv/r6_%s.csv\n', tag);
end
