function validate_fw_model()
%VALIDATE_FW_MODEL  固定翼 Simulink 模型与脚本实现的对拍。
%   同参考、同初值、同积分步长，比较 13 维状态轨迹。
%   模型里控制器与执行机构采样 1e-3，脚本侧对应 ctrlDiv=1。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
mdlFile = fullfile(root, 'models', 'fw_closed_loop.slx');

p = uav.cfg.fw();
c = uav.cfg.fwctrl();
s = uav.cfg.simcfg();
s.ctrlDiv = 1;
s.tEnd    = 60;

ref = uav.env.fw_mission(p.htrim, p.Vtrim);
out = uav.sim.run_fw(p, c, s, ref);

load_system(mdlFile);
simOut = sim('fw_closed_loop', 'StopTime', '60');
bdclose('fw_closed_loop');

xs = reshape(simOut.x_out.Data, 13, []).';
xs = xs(1:s.logDiv:end, :);
n  = min(size(xs,1), size(out.x,1));
d  = xs(1:n, :) - out.x(1:n, :);
fprintf('样本数 %d，时间窗 %.2f s\n', n, out.t(n));
fprintf('最大偏差 %.3e（%.3f mm 位置，%.3e rad 姿态）\n', ...
    max(abs(d(:))), 1000*max(vecnorm(d(:,1:3),2,2)), max(abs(d(:,7:10)), [], 'all'));

assert(max(abs(d(:))) < 1e-6, '模型与脚本轨迹不一致');
fprintf('对拍通过\n');
end
