%% quad_trajectory.m - 参考轨迹生成器
% 生成多种测试轨迹，用于仿真验证
% 轨迹指令在 NED 体轴系下生成（与 multiModeQuad_ROS 一致）
%
% 用法:
%   traj = quad_trajectory('hover', t);
%   traj = quad_trajectory('circle', t, radius, omega, height);
%   traj = quad_trajectory('figure8', t);
%   traj = quad_trajectory('step', t);
%   traj = quad_trajectory('helix', t);

function traj = quad_trajectory(type, t, varargin)

switch lower(type)
    case 'hover'
        traj = hover(t);
    case 'circle'
        radius = 2.0; omega = 0.3; height = 1.0;
        if nargin >= 3, radius = varargin{1}; end
        if nargin >= 4, omega = varargin{2}; end
        if nargin >= 5, height = varargin{3}; end
        traj = circle(t, radius, omega, height);
    case 'figure8'
        traj = figure8(t);
    case 'step'
        traj = step_response(t);
    case 'helix'
        traj = helix(t);
    otherwise
        error('未知轨迹类型: %s (支持: hover/circle/figure8/step/helix)', type);
    end
end

%% ==================== 轨迹函数 ====================

% 1. 悬停
function traj = hover(t)
    N = length(t);
    traj.vx = zeros(N,1);
    traj.vy = zeros(N,1);
    traj.vz = zeros(N,1);
    traj.wx = zeros(N,1);
    traj.wy = zeros(N,1);
    traj.wz = zeros(N,1);
    traj.mode = zeros(N,1);  % 速度模式
    traj.name = 'Hover';
end

% 2. 圆形轨迹
function traj = circle(t, radius, omega, height)
    N = length(t);
    % 位置: x = r*cos(ωt), y = r*sin(ωt), z = -height
    % 速度: vx = -r*ω*sin(ωt), vy = r*ω*cos(ωt)
    traj.vx = -radius * omega * sin(omega * t(:));
    traj.vy =  radius * omega * cos(omega * t(:));
    traj.vz = -0.2 * ones(N,1);  % 缓慢上升 (NED)
    traj.wx = zeros(N,1);
    traj.wy = zeros(N,1);
    traj.wz = 0.1 * ones(N,1);   % 缓慢偏航
    traj.mode = zeros(N,1);
    traj.name = sprintf('Circle (r=%.1f, ω=%.2f)', radius, omega);
end

% 3. 8字形轨迹
function traj = figure8(t)
    N = length(t);
    omega = 0.2;
    scale = 2.0;
    % 位置: x = scale*sin(ωt), y = scale*sin(2ωt)
    % 速度: vx = scale*ω*cos(ωt), vy = 2*scale*ω*cos(2ωt)
    traj.vx =  scale * omega * cos(omega * t(:));
    traj.vy =  2 * scale * omega * cos(2 * omega * t(:));
    traj.vz = -0.1 * ones(N,1);
    traj.wx = zeros(N,1);
    traj.wy = zeros(N,1);
    traj.wz = 0.05 * ones(N,1);
    traj.mode = zeros(N,1);
    traj.name = 'Figure-8';
end

% 4. 阶跃响应
function traj = step_response(t)
    N = length(t);
    traj.vx = zeros(N,1);
    traj.vy = zeros(N,1);
    traj.vz = zeros(N,1);
    traj.wx = zeros(N,1);
    traj.wy = zeros(N,1);
    traj.wz = zeros(N,1);

    % 2s 后前飞 1m/s
    idx = t >= 2;
    traj.vx(idx) = 1.0;

    % 5s 后侧飞
    idx = t >= 5;
    traj.vy(idx) = 0.5;

    % 8s 后上升
    idx = t >= 8;
    traj.vz(idx) = -0.3;  % NED

    traj.mode = zeros(N,1);
    traj.name = 'Step Response';
end

% 5. 螺旋上升
function traj = helix(t)
    N = length(t);
    radius = 1.5;
    omega = 0.4;
    climb_rate = 0.2;

    traj.vx = -radius * omega * sin(omega * t(:));
    traj.vy =  radius * omega * cos(omega * t(:));
    traj.vz = -climb_rate * ones(N,1);  % NED: 持续上升
    traj.wx = zeros(N,1);
    traj.wy = zeros(N,1);
    traj.wz = 0.15 * ones(N,1);
    traj.mode = zeros(N,1);
    traj.name = sprintf('Helix (r=%.1f, climb=%.2f)', radius, climb_rate);
end
