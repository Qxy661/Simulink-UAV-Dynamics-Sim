% r14_thrust_margin_trade  推力裕度 2 的代价
%
%   REPORT_r6 的判据是 maxT >= mg/2（wMax >= sqrt(2)*hoverW）。这条要求
%   在本机上差 25% 的单台推力。25% 从哪来？两条路：提转速，或加大桨盘。
%   这里把两条路的代价算成同一张表，并用仿真确认临界点。
%
%   结果写 research/results/csv/r14_thrust_margin_trade.csv，
%   图写 research/results/fig/r14_thrust_margin_trade.png。

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(pwd));

p0 = uav.cfg.tilt();
c  = uav.cfg.tiltctrl();
s0 = uav.cfg.simcfg();

R0 = 0.30;                       % 现有桨半径（假定值，见 REPORT_r10 的说明）
needT = p0.mass * p0.g / 2;      % 单桨需要扛住一半重量
wNeed = sqrt(needT / p0.kt);     % 临界转速

fprintf('单桨需要 %.2f N，临界 wMax = %.1f rad/s（现 %.0f）\n', needT, wNeed, p0.wMax);

wList = [1100 1150 1200 1229 1250 1300];
rows = {};
for w = wList
    p = p0; p.wMax = w;
    margin   = (p.kt * w^2) / (p.mass * p.g / 4);
    hoverPct = 100 * p.hoverW / w;
    tipSpeed = w * R0;
    cf       = (w / p0.wMax)^2;             % 离心载荷（相对）
    % 同一桨盘下，推力上限提高需要更大功率：P ∝ T^(3/2)
    pwr      = (p.kt * w^2 / (p0.kt * p0.wMax^2))^1.5;

    % 用仿真确认悬停断桨是否真的活下来
    s = s0; s.tEnd = 25; s.logDiv = 20;
    s.alloc = 'rankrpi'; s.faultAware = true; s.faultHard = true;
    s.fault = struct('t', 8, 'k', 1, 'eta', 0);
    ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                      'psi', 0, 'h', 20, 'V', 0);
    out = uav.sim.run_tilt(p, c, s, ref);
    post = out.t > 8;
    hMin  = min(-out.p(post, 3));
    drift = max(vecnorm(out.p(post, 1:2), 2, 2));

    rows(end+1, :) = {w, margin, hoverPct, tipSpeed, cf, pwr, hMin, drift}; %#ok<SAGROW>
    fprintf('wMax %4.0f  裕度 %4.2f  悬停占 %4.1f%%  桨尖 %5.1f m/s  离心 %4.2fx  功率 %4.2fx  断桨 hMin %5.2f  漂移 %6.1f\n', ...
            w, margin, hoverPct, tipSpeed, cf, pwr, hMin, drift);
end

% 第二条路：同样桨尖速度下加大桨盘。Tmax ∝ R^2（kt∝R^4、w∝1/R）
Rneed = R0 * sqrt(needT / (p0.kt * p0.wMax^2));
fprintf('\n若保持桨尖速度不变、只加大桨盘：半径 %.3f -> %.3f m（%+.1f%%），转速 %.0f -> %.0f\n', ...
        R0, Rneed, 100*(Rneed/R0 - 1), p0.wMax, p0.wMax * R0 / Rneed);

csvDir = fullfile('research', 'results', 'csv');
if ~isfolder(csvDir), mkdir(csvDir); end
T = cell2table(rows, 'VariableNames', {'wMax','margin','hoverPct','tipSpeed', ...
    'centrifugal','power','hoverHMin','hoverDrift'});
writetable(T, fullfile(csvDir, 'r14_thrust_margin_trade.csv'));

figDir = fullfile('research', 'results', 'fig');
if ~isfolder(figDir), mkdir(figDir); end
f = figure('Visible', 'off', 'Position', [100 100 900 380], 'Color', 'w');
subplot(1,2,1); hold on
plot(cell2mat(rows(:,1)), cell2mat(rows(:,7)), '-o', 'LineWidth', 1.4);
yline(19, 'k--');
hold off; grid on
xlabel('wMax (rad/s)'); ylabel('断桨后最低高度 (m)'); title('悬停断桨：裕度与存活')
subplot(1,2,2); hold on
plot(cell2mat(rows(:,1)), cell2mat(rows(:,6)), '-s', 'LineWidth', 1.4);
plot(cell2mat(rows(:,1)), cell2mat(rows(:,5)), '-^', 'LineWidth', 1.4);
hold off; grid on
xlabel('wMax (rad/s)'); ylabel('相对 1100 的倍数'); title('代价：功率与离心载荷')
legend({'最大轴功率', '离心载荷'}, 'Location', 'northwest');
exportgraphics(f, fullfile(figDir, 'r14_thrust_margin_trade.png'), 'Resolution', 150);
close(f);

fprintf('\nreference: research/REPORT_r10_margin_trade.md\n');
