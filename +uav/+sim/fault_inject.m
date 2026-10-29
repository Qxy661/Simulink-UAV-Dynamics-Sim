function out = fault_inject(p, c, s, tFail, kFail, eta, ref)
%FAULT_INJECT  带电机失效/降效的四旋翼闭环仿真。
%   out = uav.sim.fault_inject(p, c, s, tFail, kFail, eta, ref)
%   tFail 秒后第 kFail 台电机的推力与反扭矩按 eta 缩放（0 = 完全停转，
%   0 < eta < 1 = 降效），其余电机由伪逆重新分配。ref 省略时 5 m 悬停。

if nargin < 4 || isempty(tFail), tFail = 5; end
if nargin < 5 || isempty(kFail), kFail = 1; end
if nargin < 6 || isempty(eta), eta = 0.35; end
if nargin < 7 || isempty(ref), ref = uav.env.hover(0, 0, -5, 0); end

s.fault = struct('t', tFail, 'k', kFail, 'eta', eta);
if ~isfield(s, 'ctrl'), s.ctrl = 'pid'; end
out = uav.sim.run(p, c, s, ref);
end
