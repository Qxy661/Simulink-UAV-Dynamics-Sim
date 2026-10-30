function out = demo(caseName)
%DEMO  运行内置演示并出图。
%   uav.demo('hover')   四旋翼定点悬停
%   uav.demo('step')    四旋翼位置阶跃（默认）
%   uav.demo('yaw')     四旋翼航向阶跃
%   uav.demo('fw')      固定翼爬升-转弯-加速任务
%   uav.demo('tilt')    倾转四旋翼悬停-转捩-巡航
%   uav.demo('wind')    阵风下的 PID 与 ESO 对比

if nargin < 1
    caseName = 'step';
end

if strcmpi(caseName, 'fw')
    out = fw_demo();
    return
end

if strcmpi(caseName, 'tilt')
    out = tilt_demo();
    return
end

if strcmpi(caseName, 'wind')
    out = wind_demo();
    return
end

p = uav.cfg.quad();
c = uav.cfg.ctrl();
s = uav.cfg.simcfg();

switch lower(caseName)
    case 'hover'
        s.tEnd = 12;
        ref = uav.env.hover(0, 0, -5, 0);
        x0 = [0;0;-4; 0;0;0; 1;0;0;0; 0;0;0];
    case 'step'
        s.tEnd = 14;
        ref = @(t) uav.env.step(t, 1, [0;0;-5], [3;2;-5], 0.4);
        x0 = [];
    case 'yaw'
        s.tEnd = 10;
        ref = uav.env.hover(0, 0, -5, 0.5);
        x0 = [];
    otherwise
        error('uav:demo:unknownCase', '未知演示：%s', caseName);
end

out = uav.sim.run(p, c, s, ref, x0);
m = uav.sim.metrics(out);

fprintf('\n== %s ==\n', caseName);
fprintf('末端位置  [%.3f %.3f %.3f] m\n', out.p(end,:));
fprintf('后段误差  max %.3f m, rms %.3f m\n', m.errMax, m.errRms);
fprintf('最大倾角  %.1f deg\n', rad2deg(m.tiltMax));
fprintf('饱和比例  %.1f%%\n', 100*m.satRatio);

root = fileparts(fileparts(mfilename('fullpath')));
figDir = fullfile(root, 'figures');
if ~isfolder(figDir)
    mkdir(figDir);
end

f = figure('Color', 'w', 'Position', [100 100 900 640]);

subplot(2,2,1);
plot(out.p(:,2), out.p(:,1), 'b', 'LineWidth', 1.4); hold on
plot(out.ref(:,2), out.ref(:,1), 'r--', 'LineWidth', 1);
plot(out.p(1,2), out.p(1,1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
plot(out.p(end,2), out.p(end,1), 'ks', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
grid on; axis equal
xlabel('East (m)'); ylabel('North (m)');
title('水平轨迹（俯视）');
legend('实际', '参考', '起点', '终点', 'Location', 'best');

subplot(2,2,2);
plot(out.t, -out.p(:,3), 'b', out.t, -out.ref(:,3), 'r--');
grid on; xlabel('t (s)'); ylabel('高度 (m)'); title('高度跟踪');

subplot(2,2,3);
plot(out.t, rad2deg(out.eul(:,1)), out.t, rad2deg(out.eul(:,2)), ...
     out.t, rad2deg(out.eul(:,3)), 'LineWidth', 1);
grid on; xlabel('t (s)'); ylabel('角度 (deg)');
legend('\phi', '\theta', '\psi', 'Location', 'best'); title('姿态');

subplot(2,2,4);
plot(out.t, out.w, 'LineWidth', 1);
grid on; xlabel('t (s)'); ylabel('\omega (rad/s)');
legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', 'Location', 'best');
title('转速');

saveas(f, fullfile(figDir, ['demo_' caseName '.png']));
fprintf('图已保存 %s\n', fullfile(figDir, ['demo_' caseName '.png']));
end

function out = fw_demo()
p = uav.cfg.fw();
c = uav.cfg.fwctrl();
s = uav.cfg.simcfg();
s.tEnd = 60;
ref = uav.env.fw_mission(p.htrim, p.Vtrim);
out = uav.sim.run_fw(p, c, s, ref);

h = -out.p(:, 3);
V = vecnorm(out.v, 2, 2);
psi = rad2deg(uav.ctrl.wrap_pi(out.eul(:, 3)));

fprintf('\n== fw ==\n');
fprintf('末端高度  %.2f m，末端空速 %.2f m/s，末端航向 %.1f deg\n', ...
    h(end), V(end), psi(end));
fprintf('高度范围  %.1f .. %.1f m\n', min(h), max(h));
fprintf('空速范围  %.1f .. %.1f m/s\n', min(V), max(V));

root = fileparts(fileparts(mfilename('fullpath')));
figDir = fullfile(root, 'figures');
if ~isfolder(figDir)
    mkdir(figDir);
end

f = figure('Color', 'w', 'Position', [100 100 900 640]);

subplot(2,2,1);
plot(out.t, h, 'b', out.t, out.ref(:,1), 'r--'); grid on
xlabel('t (s)'); ylabel('高度 (m)'); title('高度跟踪');
legend('实际', '参考', 'Location', 'best');

subplot(2,2,2);
plot(out.t, V, 'b', out.t, out.ref(:,3), 'r--'); grid on
xlabel('t (s)'); ylabel('空速 (m/s)'); title('空速跟踪');
legend('实际', '参考', 'Location', 'best');

subplot(2,2,3);
plot(out.t, psi, 'b', out.t, rad2deg(out.ref(:,2)), 'r--'); grid on
xlabel('t (s)'); ylabel('航向 (deg)'); title('航向跟踪');
legend('实际', '参考', 'Location', 'best');

subplot(2,2,4);
plot(out.t, rad2deg(out.act(:,2)), out.t, rad2deg(out.act(:,3)), ...
     out.t, rad2deg(out.act(:,4)), 'LineWidth', 1); grid on
xlabel('t (s)'); ylabel('舵面偏角 (deg)');
legend('\delta_e', '\delta_a', '\delta_r', 'Location', 'best');
title('舵面');

saveas(f, fullfile(figDir, 'demo_fw.png'));
fprintf('图已保存 %s\n', fullfile(figDir, 'demo_fw.png'));
end

function out = tilt_demo()
p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 45;
ref = @uav.env.tilt_ref;
out = uav.sim.run_tilt(p, c, s, ref);

h   = -out.p(:,3);
V   = vecnorm(out.v, 2, 2);
bt  = rad2deg(out.beta);
th  = rad2deg(out.eul(:,2));
ep  = vecnorm(out.p - out.ref(:,1:3), 2, 2);

fprintf('\n== tilt ==\n');
fprintf('末端高度 %.2f m，末端空速 %.2f m/s，短舱 %.1f deg\n', h(end), V(end), bt(end));
fprintf('高度偏差 max %.2f m，水平误差 max %.2f m（末段 %.2f m）\n', ...
    max(abs(h - p.htrim)), max(ep(round(end*0.3):end)), ep(end));
fprintf('短舱范围 %.1f .. %.1f deg，俯仰 %.1f .. %.1f deg\n', min(bt), max(bt), min(th), max(th));

root = fileparts(fileparts(mfilename('fullpath')));
figDir = fullfile(root, 'figures');
if ~isfolder(figDir)
    mkdir(figDir);
end

f = figure('Color', 'w', 'Position', [100 100 900 640]);

subplot(2,2,1);
plot(out.t, h - p.htrim, 'b'); grid on; ylim([-1 1]);
yline(0, 'r--');
xlabel('t (s)'); ylabel('高度偏差 (m)'); title('高度保持');

subplot(2,2,2);
plot(out.t, V, 'b', out.t, out.ref(:,6), 'r--'); grid on
xlabel('t (s)'); ylabel('空速 (m/s)'); title('空速跟踪');
legend('实际', '参考', 'Location', 'best');

subplot(2,2,3);
yyaxis left
plot(out.t, bt, 'b'); ylabel('短舱角 \beta (deg)');
yyaxis right
plot(out.t, th, 'r'); ylabel('俯仰 \theta (deg)');
grid on; xlabel('t (s)'); title('短舱倾转与姿态');

subplot(2,2,4);
plot(out.t, out.om, 'LineWidth', 1); grid on
xlabel('t (s)'); ylabel('\omega (rad/s)');
legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', 'Location', 'best');
title('旋翼转速');

saveas(f, fullfile(figDir, 'demo_tilt.png'));
fprintf('图已保存 %s\n', fullfile(figDir, 'demo_tilt.png'));
end

function out = wind_demo()
p = uav.cfg.quad();
c = uav.cfg.ctrl();
s = uav.cfg.simcfg();
s.tEnd = 40;
s.ctrlDiv = 1;
s.logDiv = 10;
ref  = uav.env.hover(0, 0, -5, 0);
gust = uav.env.wind_dryden(s.tEnd, s.dt, 10, 2.0, 11, [3; 2; 0]);

s1 = s; s1.useEso = false;
o1 = uav.sim.run_wind(p, c, s1, ref, gust);
s2 = s; s2.useEso = true;
o2 = uav.sim.run_wind(p, c, s2, ref, gust);

e1 = vecnorm(o1.p(:, 1:2) - o1.ref(:, 1:2), 2, 2);
e2 = vecnorm(o2.p(:, 1:2) - o2.ref(:, 1:2), 2, 2);
tail = o1.t > 15;
fprintf('\n== wind ==\n');
fprintf('平均风 [3 2 0] m/s，湍流 2 m/s\n');
fprintf('PID     水平 RMS %.3f m，最大 %.3f m\n', sqrt(mean(e1(tail).^2)), max(e1(tail)));
fprintf('PID+ESO 水平 RMS %.3f m，最大 %.3f m\n', sqrt(mean(e2(tail).^2)), max(e2(tail)));
fprintf('ESO 扰动估计 dHat 稳态 [%.3f %.3f %.3f] m/s^2\n', mean(o2.dHat(tail, :), 1));

root = fileparts(fileparts(mfilename('fullpath')));
figDir = fullfile(root, 'figures');
if ~isfolder(figDir)
    mkdir(figDir);
end

f = figure('Color', 'w', 'Position', [100 100 900 640]);

subplot(2,2,1);
plot(o1.p(:,2), o1.p(:,1), 'b', 'LineWidth', 1.2); hold on
plot(o2.p(:,2), o2.p(:,1), 'r', 'LineWidth', 1.2);
plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
grid on; axis equal
xlabel('East (m)'); ylabel('North (m)');
title('悬停轨迹（俯视）');
legend('PID', 'PID+ESO', '参考点', 'Location', 'best');

subplot(2,2,2);
plot(o1.t, e1, 'b', o2.t, e2, 'r');
grid on; xlabel('t (s)'); ylabel('水平误差 (m)'); title('水平误差');
legend('PID', 'PID+ESO', 'Location', 'best');

subplot(2,2,3);
dTrue = p.drag * gust.Wmean / p.mass;
plot(o2.t, o2.dHat(:,1), 'b', o2.t, o2.dHat(:,2), 'r'); hold on
yline(dTrue(1), 'b--'); yline(dTrue(2), 'r--');
grid on; xlabel('t (s)'); ylabel('扰动加速度 (m/s^2)');
legend('dHat 北向', 'dHat 东向', '真值 北向', '真值 东向', 'Location', 'best');
title('ESO 扰动估计');

subplot(2,2,4);
plot(o1.t, rad2deg(o1.eul(:,1)), 'b', o2.t, rad2deg(o2.eul(:,1)), 'r');
grid on; xlabel('t (s)'); ylabel('\phi (deg)');
legend('PID', 'PID+ESO', 'Location', 'best'); title('滚转角');

saveas(f, fullfile(figDir, 'demo_wind.png'));
fprintf('图已保存 %s\n', fullfile(figDir, 'demo_wind.png'));
out = o2;
end