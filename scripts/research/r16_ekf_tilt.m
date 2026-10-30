% r16_ekf_tilt  EKF 放到倾转构型上还能不能用
%
%   四旋翼上的 EKF（15 维误差状态、GPS/气压/磁/光流）在 60 s 悬停里能到 7.9 cm。
%   倾转构型有两个新麻烦：22 m/s 的高速段，以及短舱转动时推力方向的快速变化。
%   用真值轨迹喂传感器、EKF 开环跑，分三段（悬停 / 过渡 / 巡航）看误差；
%   再加一组"高速段关掉光流"的对照，验证 2% 标度误差在高速下被放大成速度偏置。
%
%   结果写 research/results/csv/r16_ekf_tilt.csv，
%   图写 research/results/fig/r16_ekf_tilt.png。

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(pwd));

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 45; s.logDiv = 1;
ref = @(t) uav.env.tilt_ref_v(t, 22);

out = uav.sim.run_tilt(p, c, s, ref);
truth.t = out.t; truth.p = out.p; truth.v = out.v;
truth.q = out.x(:, 7:10); truth.f = out.f;

cases = {'光流全程', true; '高速段关光流', false};
segs = {'悬停', 0, 5; '过渡', 5, 20; '巡航', 20, 45};
rows = {};
for i = 1:size(cases, 1)
    r = runCase(truth, cases{i,2});
    n = numel(truth.t);
    eulT = zeros(n,3); eulE = zeros(n,3);
    for k = 1:n
        eulT(k,:) = uav.plant.quat2euler(truth.q(k,:));
        eulE(k,:) = uav.plant.quat2euler(r.Q(k,:));
    end
    dE = mod(eulE - eulT + 180, 360) - 180;

    for j = 1:size(segs, 1)
        m = truth.t >= segs{j,2} & truth.t < segs{j,3};
        eP = r.P(m,:) - truth.p(m,:);
        eV = r.V(m,:) - truth.v(m,:);
        rows(end+1,:) = {cases{i,1}, segs{j,1}, ...
                         sqrt(mean(sum(eP.^2, 2))), sqrt(mean(sum(eV.^2, 2))), ...
                         max(abs(dE(m, :)), [], 'all')}; %#ok<SAGROW>
        fprintf('%-14s %-4s 位置 RMS %.3f m  速度 RMS %.3f m/s  最大姿态 %.2f°\n', ...
                cases{i,1}, segs{j,1}, rows{end,3}, rows{end,4}, rows{end,5});
    end
    if i == 1
        keep.P = r.P; keep.dE = dE;
    end
end

csvDir = fullfile('research','results','csv');
if ~isfolder(csvDir), mkdir(csvDir); end
T = cell2table(rows, 'VariableNames', {'case','segment','posRms','velRms','attMaxDeg'});
writetable(T, fullfile(csvDir, 'r16_ekf_tilt.csv'));

figDir = fullfile('research','results','fig');
if ~isfolder(figDir), mkdir(figDir); end
f = figure('Visible','off','Position',[100 100 900 340],'Color','w');
subplot(1,2,1); hold on
plot(truth.t, truth.p(:,1)-keep.P(:,1), 'LineWidth', 1.1);
plot(truth.t, truth.p(:,2)-keep.P(:,2), 'LineWidth', 1.1);
plot(truth.t, truth.p(:,3)-keep.P(:,3), 'LineWidth', 1.1);
hold off; grid on; xlabel('t (s)'); ylabel('位置误差 (m)');
legend({'北','东','高'}, 'Location','best'); title('光流全程')
subplot(1,2,2); hold on
plot(truth.t, keep.dE(:,1), 'LineWidth', 1.1);
plot(truth.t, keep.dE(:,2), 'LineWidth', 1.1);
plot(truth.t, keep.dE(:,3), 'LineWidth', 1.1);
hold off; grid on; xlabel('t (s)'); ylabel('姿态误差 (°)');
legend({'\phi','\theta','\psi'}, 'Location','best'); title('姿态误差')
exportgraphics(f, fullfile(figDir,'r16_ekf_tilt.png'), 'Resolution', 150);
close(f);

fprintf('\nreference: research/REPORT_r12_ekf_tilt.md\n');

function r = runCase(truth, flowAll)
sigmaG = 0.004; sigmaA = 0.08;
bgTrue = [0.02; -0.015; 0.01];
baTrue = [0.10; -0.08; 0.06];
mRef = [0.42; 0; 0.91]; mRef = mRef / norm(mRef);

nz = struct('sigmaG', sigmaG, 'sigmaA', sigmaA, 'sigmaBg', 1e-4, 'sigmaBa', 1e-3);
e = uav.est.ekf_init(truth.p(1,:)', truth.q(1,:), ...
        struct('sigmaP', 0.5, 'sigmaV', 0.3, 'sigmaQ', 0.05, ...
               'sigmaBg', 0.01, 'sigmaBa', 0.1));
rng(7);

n = numel(truth.t);
r.P = zeros(n,3); r.V = zeros(n,3); r.Q = zeros(n,4);
wPrev = zeros(3,1);
for k = 1:n
    dt = 1e-3;
    if k < n
        dq = uav.plant.qmul(truth.q(k+1,:)', [truth.q(k,1); -truth.q(k,2:4)']);
        wTrue = 2 * dq(2:4) / dt;
    else
        wTrue = wPrev;
    end
    wPrev = wTrue;
    zImu = uav.sens.imu_sample(wTrue + bgTrue, truth.f(k,:)' + baTrue, ...
                               zeros(3,1), zeros(3,1), sigmaG, sigmaA);
    e = uav.est.ekf_predict(e, zImu.gyro, zImu.accel, dt, nz);

    if mod(k-1, 200) == 0
        [zP, zV] = uav.sens.gps_sample(truth.p(k,:)', 1.0, 2.0, truth.v(k,:)', 0.15);
        e = uav.est.ekf_update_gps(e, zP, [1;1;2]);
        e = uav.est.ekf_update_vel(e, zV, 0.15);
    end
    if mod(k-1, 50) == 0
        zB = uav.sens.baro_sample(-truth.p(k,3), 0.15);
        e = uav.est.ekf_update_baro(e, zB, 0.15);
    end
    if mod(k-1, 20) == 0
        zM = uav.sens.mag_sample(truth.q(k,:)', mRef, 0.03);
        e = uav.est.ekf_update_mag(e, zM, 0.03, mRef);
        useFlow = flowAll || norm(truth.v(k,:)) < 6;
        if useFlow
            vBody = uav.plant.q2dcm(truth.q(k,:))' * truth.v(k,:)';
            zF = uav.sens.flow_sample(truth.q(k,:)', vBody, 0.03, 1.02);
            e = uav.est.ekf_update_flow(e, zF, 0.03);
        end
    end

    r.P(k,:) = e.p'; r.V(k,:) = e.v'; r.Q(k,:) = e.q';
end
end
