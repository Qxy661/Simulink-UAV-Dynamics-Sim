function T = r2_full_roundtrip()
%R2_FULL_ROUNDTRIP  全包线往返：悬停—加速到 22 m/s（β=90°）—减速回悬停。
%
%   验证两件事：
%     1. 短舱能从 0° 一路转到 90°，不再像 v1.0 那样在 57° 卡住；
%     2. 减速段还能把机体拉回悬停，而不只是"单向过渡"。
%
%   参考速度用余弦包络：0–15 s 加速到 22 m/s，15–20 s 保持，20–45 s 减速到 0，
%   之后悬停。高度全程 20 m。

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cd(root); addpath(root);

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();

t1 = 15; t2 = 20; t3 = 45; Vc = 22;
ref = @(t) local_ref(t, p.htrim, Vc, t1, t2, t3);

s = struct('dt', 1e-3, 'ctrlDiv', 1, 'tEnd', 70, 'logDiv', 10);
out = uav.sim.run_tilt(p, c, s, ref);

n = numel(out.t);
h = -out.x(:, 3);
V = vecnorm(out.x(:, 4:6), 2, 2);
beta = out.x(:, 14);
th = out.eul(:, 2);
dh = h - p.htrim;

T_ = table(out.t, h, dh, V, th, beta, ...
           out.x(:,15), out.x(:,16), out.x(:,17), out.x(:,18), ...
           out.cmd(:,5), out.cmd(:,6), out.cmd(:,7));
T_.Properties.VariableNames = {'t','h','dh','V','theta','beta','om1','om2','om3','om4','de','da','dr'};
writetable(T_, fullfile(root, 'research', 'results', 'csv', 'r2_roundtrip.csv'));

tail = out.t > 65;
fprintf('末端：h %.2f m（偏差 %.2f m）、V %.2f m/s、beta %.1f deg、theta %.1f deg\n', ...
        h(end), dh(end), V(end), rad2deg(beta(end)), rad2deg(th(end)));
fprintf('悬停段（>65 s）：高度偏差均值 %.3f m、水平速度 %.3f m/s\n', ...
        mean(dh(tail)), mean(V(tail)));
fprintf('峰值：|dh| %.2f m、|theta| %.1f deg、beta %.1f deg、om %.0f rad/s\n', ...
        max(abs(dh)), rad2deg(max(abs(th))), rad2deg(max(beta)), max(out.x(:, 15:18), [], 'all'));

f = figure('Visible','off','Position',[100 100 900 700]);
tiledlayout(3,1);
nexttile; plot(out.t, V, 'LineWidth', 1.3); hold on; plot(out.t, local_v(out.t, Vc, t1, t2, t3), 'k--');
ylabel('空速 (m/s)'); grid on; legend('实际','参考','Location','best');
nexttile; plot(out.t, rad2deg(beta), 'LineWidth', 1.3); ylabel('短舱角 (deg)'); grid on; ylim([-5 95]);
nexttile; plot(out.t, dh, 'LineWidth', 1.3); ylabel('高度偏差 (m)'); xlabel('t (s)'); grid on;
exportgraphics(f, fullfile(root,'research','results','fig','r2_roundtrip.png'), 'Resolution', 150);
close(f);

T = T_;
end

function r = local_ref(t, h0, Vc, t1, t2, t3)
v  = local_v(t, Vc, t1, t2, t3);
[~, sPos] = local_s(t, Vc, t1, t2, t3);
r = struct('p', [sPos; 0; -h0], 'v', [v; 0; 0], 'a', [0; 0; 0], ...
           'psi', 0, 'h', h0, 'V', v);
end

function v = local_v(t, Vc, t1, t2, t3)
v = 0;
if t >= t1 && t < t2
    v = Vc * 0.5 * (1 - cos(pi*(t - t1)/(t2 - t1)));
elseif t >= t2 && t < t3
    v = Vc;
elseif t >= t3
    v = Vc * 0.5 * (1 + cos(pi*min(t - t3, 10)/10));
end
end

function [v, s] = local_s(t, Vc, t1, t2, t3)
% 速度曲线是余弦包络，位置用它的解析积分；避免在 1 kHz 循环里做数值积分
T1 = t2 - t1; T3 = 10;
s  = 0; v = 0;
if t <= t1
    return;
elseif t < t2
    u = t - t1;
    v = Vc*0.5*(1 - cos(pi*u/T1));
    s = Vc*(u/2 - T1/(2*pi)*sin(pi*u/T1));
elseif t < t3
    v = Vc;
    s = Vc*(T1/2) + Vc*(t - t2);
else
    u  = min(t - t3, T3);
    v  = Vc*0.5*(1 + cos(pi*u/T3));
    s  = Vc*(T1/2) + Vc*(t3 - t2) + Vc*(u/2 + T3/(2*pi)*sin(pi*u/T3));
end
end
