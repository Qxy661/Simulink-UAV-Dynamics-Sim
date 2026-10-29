function s = simcfg()
%SIMCFG  仿真步长与记录设置。
%   控制器降频运行，积分仍用 dt；避免控制器与积分器同频耦合。

s.dt      = 1e-3;                   % 积分步长 s
s.ctrlDiv = 10;                     % 控制器分频（100 Hz）
s.tEnd    = 20;                     % 默认时长 s
s.logDiv  = 10;                     % 记录分频
end
