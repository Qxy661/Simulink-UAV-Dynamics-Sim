%% plot_quad_results.m - 四旋翼仿真结果可视化
% 输入: sim_log - 仿真日志结构体
%        params - 参数结构体 (可选)

function plot_quad_results(sim_log, params)

if nargin < 2
    params = struct();
end

% 颜色定义
colors = lines(7);
ls = {'-', '--', ':'};

%% 图1: 3D 轨迹
figure('Name', '3D Trajectory', 'Position', [100, 100, 600, 500]);
plot3(sim_log.xe, sim_log.ye, sim_log.ze, 'b-', 'LineWidth', 1.5);
hold on; grid on;
plot3(sim_log.xe(1), sim_log.ye(1), sim_log.ze(1), 'go', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(sim_log.xe(end), sim_log.ye(end), sim_log.ze(end), 'ro', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('North [m]'); ylabel('East [m]'); zlabel('Down [m]');
title('3D Flight Trajectory');
legend('Trajectory', 'Start', 'End', 'Location', 'best');
axis equal; view(45, 30);

% 如果有参考轨迹则叠加
if isfield(sim_log, 'ref_x')
    plot3(sim_log.ref_x, sim_log.ref_y, sim_log.ref_z, 'r--', 'LineWidth', 1);
    legend('Actual', 'Start', 'End', 'Reference', 'Location', 'best');
end

%% 图2: 位置与速度
figure('Name', 'Position & Velocity', 'Position', [150, 150, 800, 600]);

subplot(2,2,1); hold on; grid on;
plot(sim_log.t, sim_log.xe, 'Color', colors(1,:), 'LineWidth', 1.5, 'DisplayName', 'North');
plot(sim_log.t, sim_log.ye, 'Color', colors(2,:), 'LineWidth', 1.5, 'DisplayName', 'East');
plot(sim_log.t, sim_log.ze, 'Color', colors(3,:), 'LineWidth', 1.5, 'DisplayName', 'Down');
xlabel('Time [s]'); ylabel('Position [m]'); title('Position (NED)');
legend('Location', 'best');

subplot(2,2,2); hold on; grid on;
plot(sim_log.t, sim_log.u, 'Color', colors(1,:), 'LineWidth', 1.5, 'DisplayName', 'u (body x)');
plot(sim_log.t, sim_log.v, 'Color', colors(2,:), 'LineWidth', 1.5, 'DisplayName', 'v (body y)');
plot(sim_log.t, sim_log.w, 'Color', colors(3,:), 'LineWidth', 1.5, 'DisplayName', 'w (body z)');
xlabel('Time [s]'); ylabel('Velocity [m/s]'); title('Body Velocity');
legend('Location', 'best');

subplot(2,2,3); hold on; grid on;
plot(sim_log.t, rad2deg(sim_log.phi), 'Color', colors(1,:), 'LineWidth', 1.5, 'DisplayName', 'Roll φ');
plot(sim_log.t, rad2deg(sim_log.theta), 'Color', colors(2,:), 'LineWidth', 1.5, 'DisplayName', 'Pitch θ');
plot(sim_log.t, rad2deg(sim_log.psi), 'Color', colors(3,:), 'LineWidth', 1.5, 'DisplayName', 'Yaw ψ');
xlabel('Time [s]'); ylabel('Angle [deg]'); title('Attitude (Euler Angles)');
legend('Location', 'best');

subplot(2,2,4); hold on; grid on;
plot(sim_log.t, rad2deg(sim_log.p), 'Color', colors(1,:), 'LineWidth', 1.5, 'DisplayName', 'p (roll)');
plot(sim_log.t, rad2deg(sim_log.q), 'Color', colors(2,:), 'LineWidth', 1.5, 'DisplayName', 'q (pitch)');
plot(sim_log.t, rad2deg(sim_log.r), 'Color', colors(3,:), 'LineWidth', 1.5, 'DisplayName', 'r (yaw)');
xlabel('Time [s]'); ylabel('Rate [deg/s]'); title('Angular Rates');
legend('Location', 'best');

%% 图3: 控制量输出
if isfield(sim_log, 'u1')
    figure('Name', 'Control Outputs', 'Position', [200, 200, 800, 400]);
    subplot(1,2,1); hold on; grid on;
    plot(sim_log.t, sim_log.u1, 'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Roll');
    plot(sim_log.t, sim_log.u2, 'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'Pitch');
    plot(sim_log.t, sim_log.u3, 'Color', colors(3,:), 'LineWidth', 1.2, 'DisplayName', 'Yaw');
    xlabel('Time [s]'); ylabel('Control'); title('Control Inputs (u1-u3)');
    legend('Location', 'best');

    subplot(1,2,2); hold on; grid on;
    plot(sim_log.t, sim_log.u4, 'Color', colors(4,:), 'LineWidth', 1.5, 'DisplayName', 'Thrust');
    % 叠加悬停油门线
    yline(params.hover_throttle, 'r--', 'Hover Throttle', 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('Throttle'); title('Thrust Command (u4)');
    legend('Location', 'best');
end

%% 图4: 模式切换
if isfield(sim_log, 'mode')
    figure('Name', 'Flight Mode', 'Position', [250, 250, 600, 200]);
    stairs(sim_log.t, sim_log.mode, 'LineWidth', 1.5);
    grid on; ylim([-0.5, 3.5]);
    yticks(0:3); yticklabels({'Vel', 'Rate', 'Att', 'Thr'});
    xlabel('Time [s]'); ylabel('Mode'); title('Flight Mode');
end

%% 图5: 能量分析
if isfield(sim_log, 'u4')
    figure('Name', 'Energy Analysis', 'Position', [300, 300, 600, 300]);

    % 动能
    speed = sqrt(sim_log.u.^2 + sim_log.v.^2 + sim_log.w.^2);
    KE = 0.5 * params.mass * speed.^2;

    % 势能 (NED: 高度 = -ze)
    altitude = -sim_log.ze;
    PE = params.mass * params.g * max(0, altitude);

    subplot(1,2,1); hold on; grid on;
    plot(sim_log.t, KE, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Kinetic');
    plot(sim_log.t, PE, 'r-', 'LineWidth', 1.2, 'DisplayName', 'Potential');
    plot(sim_log.t, KE + PE, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Total');
    xlabel('Time [s]'); ylabel('Energy [J]'); title('Mechanical Energy');
    legend('Location', 'best');

    subplot(1,2,2);
    speed_norm = speed / max(speed + eps);
    plot(sim_log.t, speed_norm, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]'); ylabel('Normalized Speed'); title('Speed Profile');
end

%% 图6: 电机输出
if isfield(sim_log, 'Omega')
    figure('Name', 'Motor Speeds', 'Position', [350, 350, 600, 300]);
    plot(sim_log.t, sim_log.Omega, 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]'); ylabel('Motor Speed'); title('Motor Speed Commands');
    legend('M1 (Rear-Left)', 'M2 (Front-Left)', 'M3 (Rear-Right)', ...
           'M4 (Front-Right)', 'Location', 'best');
end

%% 打印统计信息
fprintf('\n========== 仿真统计 ==========\n');
fprintf(' 仿真时长: %.1f s\n', sim_log.t(end));
fprintf(' 最大速度: %.2f m/s\n', max(sqrt(sim_log.u.^2 + sim_log.v.^2 + sim_log.w.^2)));
fprintf(' 最大倾斜: %.1f deg\n', max(rad2deg(sqrt(sim_log.phi.^2 + sim_log.theta.^2))));
fprintf(' 最大高度: %.1f m\n', -min(sim_log.ze));
fprintf(' 最终位置: (%.1f, %.1f, %.1f) m\n', ...
    sim_log.xe(end), sim_log.ye(end), sim_log.ze(end));
if isfield(sim_log, 'u4')
    fprintf(' 平均油门: %.2f\n', mean(sim_log.u4));
end
fprintf('================================\n');
end
