function validate_model()
%VALIDATE_MODEL  脚本实现与 Simulink 模型对拍。
%   两者用同一组方程独立实现。位置由积分得到，是最严格的比较量：
%   14 s 轨迹最大位置偏差小于 5 cm，终点位置偏差小于 1 mm 即通过。
%   姿态角速度在电机饱和段会出现 0.5 rad/s 量级的偏差，原因是模型的
%   离散控制器与 RK4 子步的执行顺序不同，这属于实现差异，不是方程错误。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
mdl = 'quad_closed_loop';

p   = uav.cfg.quad();
c   = uav.cfg.ctrl();
cfg = uav.cfg.simcfg();
cfg.ctrlDiv = 1;                 % 模型里控制器每步更新
cfg.tEnd = 14;
ref = @(t) uav.env.step(t, 1, [0;0;-5], [3;2;-5], 0.4);
out = uav.sim.run(p, c, cfg, ref);

load_system(mdl);
cs = getActiveConfigSet(mdl);
set_param(cs, 'StopTime', num2str(cfg.tEnd));
simOut = sim(mdl);
xs = reshape(simOut.x_out.Data, 13, []).';
n = numel(out.t);
if size(xs, 1) < (n - 1) * cfg.logDiv + 1
    error('validate:length', '采样点数不足：脚本 %d，模型 %d', n, size(xs, 1));
end
xs = xs(1:cfg.logDiv:(n - 1) * cfg.logDiv + 1, :);

d = max(abs(xs - out.x), [], 1);
names = {'pn','pe','pd','u','v','w','qw','qx','qy','qz','p','q','r'};
fprintf('状态最大偏差：\n');
for i = 1:numel(names)
    fprintf('  %-3s %.3e\n', names{i}, d(i));
end

perr = sqrt(sum((xs(:,1:3) - out.x(:,1:3)).^2, 2));
fprintf('位置轨迹最大偏差 %.3e m，终点偏差 %.3e m\n', ...
    max(perr), norm(xs(end,1:3) - out.x(end,1:3)));

assert(max(perr) < 5e-2, 'validate:pos', '位置轨迹偏差 %.3e m 超过 5 cm', max(perr));
assert(norm(xs(end,1:3) - out.x(end,1:3)) < 1e-3, 'validate:final', '终点偏差超过 1 mm');
assert(max(d(11:13)) < 1.5, 'validate:rate', '角速度偏差 %.3e rad/s 超过 1.5', max(d(11:13)));
disp('对拍通过');
end
