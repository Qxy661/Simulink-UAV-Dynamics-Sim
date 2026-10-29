function make_fault_gif()
%MAKE_FAULT_GIF  单电机降效的恢复动画。
%   1 号电机 5 s 时推力掉到 35%（桨叶损伤一类的部分失效），其余电机重新分配，
%   机体先偏斜再回到悬停。左图俯视机体与航迹，右图四台电机的有效推力。
%   输出 figures/demo_fault.gif。

root = fileparts(fileparts(mfilename('fullpath')));
p = uav.cfg.quad();
c = uav.cfg.ctrl();
s = uav.cfg.simcfg();
s.tEnd = 14;
s.logDiv = 5;              % 200 Hz 记录，动画够顺
eta = 0.35;
out = uav.sim.fault_inject(p, c, s, 5.0, 1, eta);

gifPath = fullfile(root, 'figures', 'demo_fault.gif');
if isfile(gifPath), delete(gifPath); end

r = p.a * [1 1; -1 1; -1 -1; 1 -1];    % 机体系四个旋翼位置（X 布局）

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 620 430]);
times = 0:0.25:14;
first = true;

for tf = times
    [~, k] = min(abs(out.t - tf));
    pos = out.p(k, :);
    psi = out.eul(k, 3);
    act = ones(4, 1);
    if tf >= 5, act(1) = eta; end
    spin = out.w(k, :)';
    spin = spin / max(spin);

    clf(fig);
    ax1 = subplot(1, 2, 1);
    hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
    plot(ax1, out.p(1:k, 2), out.p(1:k, 1), '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0);
    R2 = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    rot = (R2 * r')';
    for i = 1:4
        cxy = [pos(2) pos(1)] + rot(i, [2 1]);
        if tf >= 5 && i == 1
            col = [0.6 0.6 0.6];
        else
            col = [0.92 0.35 0.20];
        end
        rectangle(ax1, 'Position', [cxy(1)-0.11 cxy(2)-0.11 0.22 0.22], ...
            'Curvature', [1 1], 'FaceColor', col, 'EdgeColor', 'k');
    end
    axis(ax1, 'equal'); xlim(ax1, [-0.5 0.5]); ylim(ax1, [-0.5 0.5]);
    xlabel(ax1, 'east (m)'); ylabel(ax1, 'north (m)');
    title(ax1, sprintf('t = %4.1f s', tf));

    ax2 = subplot(1, 2, 2);
    b = bar(ax2, spin, 0.6, 'FaceColor', [0.92 0.35 0.20], 'EdgeColor', 'k');
    if tf >= 5
        b.CData(1, :) = [0.6 0.6 0.6];
    end
    ylim(ax2, [0 1.05]); grid(ax2, 'on'); box(ax2, 'on');
    set(ax2, 'XTick', 1:4, 'XTickLabel', {'1', '2', '3', '4'});
    ylabel(ax2, '归一化转速');
    title(ax2, '四台电机');

    err = norm(pos - [0 0 -5]);
    note = '正常悬停';
    if tf >= 5, note = sprintf('1 号电机降至 %.0f%%', 100*eta); end
    sgtitle(sprintf('位置误差 %.3f m    滚转 %+5.2f°    %s', ...
        err, out.eul(k, 1)*180/pi, note), 'FontSize', 10);

    drawnow;
    fr = getframe(fig);
    [A, map] = rgb2ind(fr.cdata, 32, 'nodither');
    if first
        imwrite(A, map, gifPath, 'gif', 'LoopCount', Inf, 'DelayTime', 0.07);
        first = false;
    else
        imwrite(A, map, gifPath, 'gif', 'WriteMode', 'append', 'DelayTime', 0.07);
    end
end

close(fig);
fprintf('wrote %s  (%d frames, %.0f KB)\n', gifPath, numel(times), ...
    dir(gifPath).bytes / 1024);
end
