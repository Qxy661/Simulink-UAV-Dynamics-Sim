function out = demo(caseName)
%DEMO  运行内置演示并出图。
%   uav.demo('hover')   定点悬停
%   uav.demo('step')    位置阶跃（默认）
%   uav.demo('yaw')     航向阶跃

if nargin < 1
    caseName = 'step';
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
