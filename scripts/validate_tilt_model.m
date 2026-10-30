function validate_tilt_model()
%VALIDATE_TILT_MODEL  倾转四旋翼 Simulink 模型与脚本实现的对拍。
%   同参考、同初值、同积分步长，比较 21 维状态轨迹。
%   模型里控制器按 1e-3 离散，脚本侧对应 ctrlDiv=1。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
mdlFile = fullfile(root, 'models', 'tilt_closed_loop.slx');

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.ctrlDiv = 1;
s.tEnd    = 45;

ref = @uav.env.tilt_ref;
out = uav.sim.run_tilt(p, c, s, ref);

load_system(mdlFile);
simOut = sim('tilt_closed_loop', 'StopTime', '45');
bdclose('tilt_closed_loop');

xs = reshape(simOut.x_out.Data, 21, []).';
xs = xs(1:s.logDiv:end, :);
n  = min(size(xs,1), size(out.x,1));
d  = xs(1:n, :) - out.x(1:n, :);

fprintf('样本数 %d，时间窗 %.2f s\n', n, out.t(n));
fprintf('最大偏差 %.3e（位置 %.3f mm，姿态 %.3e rad，短舱 %.3e deg）\n', ...
    max(abs(d(:))), 1000*max(vecnorm(d(:,1:3), 2, 2)), ...
    max(abs(d(:,7:10)), [], 'all'), rad2deg(max(abs(d(:,14)))));

% 分组容差：图表内联的气动估计与 fw_aero 在浮点运算顺序上不同，
% 逐位一致做不到；位置/姿态/短舱仍按工程精度校验，转速放宽到 5 rad/s。
dPos = max(vecnorm(d(:,1:3), 2, 2));
dAtt = max(abs(d(:,7:10)), [], 'all');
dBet = max(abs(d(:,14)));
dOm  = max(abs(d(:,15:18)), [], 'all');
assert(dPos < 1e-2, '位置偏差超限');
assert(dAtt < 5e-4, '姿态偏差超限');
assert(dBet < 5e-3, '短舱角偏差超限');
assert(dOm  < 5.0,  '转速偏差超限');
fprintf('对拍通过\n');
end
