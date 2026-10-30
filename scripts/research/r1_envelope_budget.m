function T = r1_envelope_budget(tag, Vlist)
%R1_ENVELOPE_BUDGET  转捩包线诊断：逐样本拆解竖直方向的力预算。
%
%   问题：Vmax 超过 10 m/s 后开始掉高，12 m/s 掉 15 m、14 m/s 发散。
%   这里不算新控制器，只把每次仿真里"向上的力"与"向下的力"拆开，
%   看缺口出现在旋翼、机翼，还是两者都不够。
%
%   输出：
%     research/results/csv/r1_<V>hz.csv   每个巡航速度的逐样本预算
%     research/results/csv/r1_summary.csv 汇总
%     research/results/fig/r1_budget.png  力预算与高度偏差

if nargin < 1, tag = 'r1'; end
if nargin < 2 || isempty(Vlist), Vlist = 8:2:16; end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cd(root); addpath(root);

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();

rows  = cell(numel(Vlist), 9);

for i = 1:numel(Vlist)
    Vmax = Vlist(i);

    s = struct('dt', 1e-3, 'ctrlDiv', 1, 'tEnd', 45, 'logDiv', 10);
    ref = @(t) uav.env.tilt_ref_v(t, Vmax);
    out = uav.sim.run_tilt(p, c, s, ref);

    n    = numel(out.t);
    Vb   = out.x(:, 4:6);
    beta = out.x(:, 14);
    om   = out.x(:, 15:18);
    act  = out.x(:, 19:21);

    FzR = zeros(n, 1);   % 旋翼推力在 NED 竖直方向的分量（向下为正）
    FzW = zeros(n, 1);   % 机翼气动力在 NED 竖直方向的分量
    for k = 1:n
        q  = out.x(k, 7:10)';
        R  = uav.plant.q2dcm(q);
        T  = p.kt * sum(om(k, :).^2);
        Fb = [T*sin(beta(k)); 0; -T*cos(beta(k))];
        Fa = uav.plant.fw_aero(Vb(k, :)', out.x(k, 11:13)', [0; act(k, :)'], p);
        FR = R*Fb;  FA = R*Fa;
        FzR(k) = FR(3);
        FzW(k) = FA(3);
    end

    h    = -out.x(:, 3);
    dh   = h - ref(0).h;
    V    = vecnorm(Vb, 2, 2);
    th   = out.eul(:, 2);
    al   = atan2(Vb(:, 3), Vb(:, 1));
    deficit = FzR + FzW + p.mass*p.g;      % >0 表示净向下，掉高

    satR = mean(any(om >= 0.999*p.wMax, 2));

    T_ = table(out.t, h, dh, V, al, th, beta, ...
               FzR, FzW, FzR + FzW, deficit, max(om, [], 2), repmat(satR, n, 1));
    T_.Properties.VariableNames = {'t','h','dh','V','alpha','theta','beta', ...
        'FzRotor','FzWing','FzTotal','deficit','omMax','sat'};
    csv = fullfile(root, 'research', 'results', 'csv', ...
                   sprintf('%s_%02dhz.csv', tag, Vmax));
    writetable(T_, csv);

    rows(i, :) = {sprintf('Vmax %d', Vmax), max(abs(dh)), max(-dh), ...
                  rad2deg(max(al)), rad2deg(max(th)), rad2deg(max(beta)), ...
                  max(deficit), satR, csv};
    fprintf('Vmax %2d: max|dh| %6.2f m, 掉高 %6.2f m, alpha %5.1f deg, ' , ...
            Vmax, max(abs(dh)), max(-dh), rad2deg(max(al)));
    fprintf('theta %5.1f deg, beta %5.1f deg, deficit %5.1f N, sat %.2f\n', ...
            rad2deg(max(th)), rad2deg(max(beta)), max(deficit), satR);
end

S = cell2table(rows, 'VariableNames', {'case','maxAbsDh','loss','alphaMax', ...
    'thetaMax','betaMax','deficitMax','satMax','csv'});
writetable(S, fullfile(root, 'research', 'results', 'csv', sprintf('%s_summary.csv', tag)));

% 力预算图：取 Vmax = 12（第一个明显掉高的工况）
i = find(Vlist == 12, 1); if isempty(i), i = 1; end
out = uav.sim.run_tilt(p, c, struct('dt',1e-3,'ctrlDiv',1,'tEnd',45,'logDiv',10), ...
                       @(t) uav.env.tilt_ref_v(t, 12));
n = numel(out.t); FzR = zeros(n,1); FzW = zeros(n,1);
for k = 1:n
    q = out.x(k,7:10)'; R = uav.plant.q2dcm(q);
    T = p.kt*sum(out.x(k,15:18).^2); b = out.x(k,14);
    Fb = [T*sin(b); 0; -T*cos(b)];
    Fa = uav.plant.fw_aero(out.x(k,4:6)', out.x(k,11:13)', [0; out.x(k,19:21)'], p);
    FR = R*Fb; FA = R*Fa;
    FzR(k) = FR(3); FzW(k) = FA(3);
end

f = figure('Visible','off','Position',[100 100 900 620]);
tiledlayout(2,1);
nexttile;
plot(out.t, -(FzR + FzW), 'LineWidth', 1.4); hold on;
plot(out.t, p.mass*p.g*ones(n,1), 'k--');
ylabel('竖直合力 (N)'); grid on;
legend('旋翼+机翼（向上）','重力','Location','best');
title('V_{max} = 12 m/s：竖直方向力预算');
nexttile;
plot(out.t, -out.x(:,3) - 20, 'LineWidth', 1.4); ylabel('高度偏差 (m)');
xlabel('t (s)'); grid on;
exportgraphics(f, fullfile(root,'research','results','fig',sprintf('%s_budget.png', tag)), 'Resolution', 150);
close(f);

disp(S);
end
