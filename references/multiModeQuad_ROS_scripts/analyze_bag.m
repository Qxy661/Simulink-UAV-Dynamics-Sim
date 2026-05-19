%% analyze_bag.m - ROS bag 数据分析工具
% 分析录制的飞行数据, 绘制性能指标
%
% 用法:
%   analyze_bag('flight_data.bag')

function analyze_bag(bag_file)

if nargin < 1
    % 如果没有输入文件, 尝试查找最近的 bag
    bag_files = dir('*.bag');
    if isempty(bag_files)
        error('请指定 .bag 文件路径');
    end
    [~, idx] = max([bag_files.datenum]);
    bag_file = bag_files(idx).name;
    fprintf('使用最近的 bag 文件: %s\n', bag_file);
end

fprintf('正在分析: %s\n', bag_file);
bag = rosbag(bag_file);

%% 提取话题数据
topics = bag.AvailableTopics.Properties.RowNames;

% Pose 数据
if any(contains(topics, 'pose'))
    pose_bag = select(bag, 'Topic', topics{contains(topics, 'pose'), 1});
    pose_msgs = readMessages(pose_bag);
    pose_time = timeSeries(pose_bag);

    t = pose_time.Time - pose_time.Time(1);
    x = pose_msgs{1}.Pose.Position.X;  % 简化, 实际需循环提取
    y = pose_msgs{1}.Pose.Position.Y;
    z = pose_msgs{1}.Pose.Position.Z;

    % 实际需循环提取所有消息
    N = length(pose_msgs);
    t = zeros(N,1); x = zeros(N,1); y = zeros(N,1); z = zeros(N,1);
    for i = 1:N
        stamp = pose_msgs{i}.Header.Stamp;
        t(i) = stamp.Sec + stamp.Nsec * 1e-9;
        x(i) = pose_msgs{i}.Pose.Position.X;
        y(i) = pose_msgs{i}.Pose.Position.Y;
        z(i) = pose_msgs{i}.Pose.Position.Z;
    end
    t = t - t(1);

    % 绘图
    figure('Name', 'Bag Analysis', 'Position', [100, 100, 900, 600]);

    subplot(2,2,1); hold on; grid on;
    plot3(x, y, z, 'b-', 'LineWidth', 1.5);
    plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(x(end), y(end), z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('3D Trajectory'); grid on; view(45, 30);

    subplot(2,2,2); hold on; grid on;
    plot(t, x, 'DisplayName', 'North');
    plot(t, y, 'DisplayName', 'East');
    plot(t, z, 'DisplayName', 'Down');
    xlabel('Time [s]'); ylabel('Position [m]');
    title('Position vs Time'); legend('Location', 'best');

    subplot(2,2,3);
    velocity = sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2) ./ diff(t);
    plot(t(2:end), velocity, 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('Speed [m/s]');
    title('Velocity Profile');

    subplot(2,2,4);
    altitude = -z;
    plot(t, altitude, 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('Altitude [m]');
    title('Altitude vs Time');
end

% IMU 数据
if any(contains(topics, 'imu'))
    imu_bag = select(bag, 'Topic', topics{contains(topics, 'imu'), 1});
    imu_msgs = readMessages(imu_bag);
    N = length(imu_msgs);

    t_imu = zeros(N,1);
    ax = zeros(N,1); ay = zeros(N,1); az = zeros(N,1);
    wx = zeros(N,1); wy = zeros(N,1); wz = zeros(N,1);

    for i = 1:N
        stamp = imu_msgs{i}.Header.Stamp;
        t_imu(i) = stamp.Sec + stamp.Nsec * 1e-9;
        ax(i) = imu_msgs{i}.LinearAcceleration.X;
        ay(i) = imu_msgs{i}.LinearAcceleration.Y;
        az(i) = imu_msgs{i}.LinearAcceleration.Z;
        wx(i) = imu_msgs{i}.AngularVelocity.X;
        wy(i) = imu_msgs{i}.AngularVelocity.Y;
        wz(i) = imu_msgs{i}.AngularVelocity.Z;
    end
    t_imu = t_imu - t_imu(1);

    figure('Name', 'IMU Data', 'Position', [150, 150, 800, 400]);
    subplot(1,2,1); hold on; grid on;
    plot(t_imu, ax, 'DisplayName', 'ax');
    plot(t_imu, ay, 'DisplayName', 'ay');
    plot(t_imu, az, 'DisplayName', 'az');
    xlabel('Time [s]'); ylabel('Accel [m/s^2]');
    title('Linear Acceleration'); legend('Location', 'best');

    subplot(1,2,2); hold on; grid on;
    plot(t_imu, rad2deg(wx), 'DisplayName', 'wx');
    plot(t_imu, rad2deg(wy), 'DisplayName', 'wy');
    plot(t_imu, rad2deg(wz), 'DisplayName', 'wz');
    xlabel('Time [s]'); ylabel('Angular Rate [deg/s]');
    title('Angular Velocity'); legend('Location', 'best');
end

fprintf('分析完成!\n');
end
