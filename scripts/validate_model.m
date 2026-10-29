function validate_model()
%VALIDATE_MODEL  四旋翼脚本实现与 Simulink 模型对拍。
%   两者用同一组方程独立实现，同参考、同初值、同积分步长。
%   模型控制器每步更新，脚本侧对应 ctrlDiv=1。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
mdlFile = fullfile(root, 'models', 'quad_closed_loop.slx');

p   = uav.cfg.quad();
c   = uav.cfg.ctrl();
cfg = uav.cfg.simcfg();
cfg.ctrlDiv = 1;
cfg.tEnd = 14;
ref = @(t) uav.env.step(t, 1, [0;0;-5], [3;2;-5], 0.4);
out = uav.sim.run(p, c, cfg, ref);

load_system(mdlFile);
simOut = sim('quad_closed_loop', 'StopTime', num2str(cfg.tEnd));
bdclose('quad_closed_loop');

xs = reshape(simOut.x_out.Data, 13, []).';
xs = xs(1:cfg.logDiv:end, :);
n  = min(size(xs, 1), size(out.x, 1));
d  = xs(1:n, :) - out.x(1:n, :);

names = {'pn','pe','pd','u','v','w','qw','qx','qy','qz','p','q','r'};
fprintf('样本数 %d\n', n);
for i = 1:numel(names)
    fprintf('  %-3s %.3e\n', names{i}, max(abs(d(:, i))));
end
perr = vecnorm(d(:, 1:3), 2, 2);
fprintf('位置轨迹最大偏差 %.3e m，终点偏差 %.3e m\n', ...
    max(perr), norm(d(end, 1:3)));

assert(max(abs(d(:))) < 1e-9, 'validate:mismatch', '模型与脚本轨迹不一致');
fprintf('对拍通过\n');
end
