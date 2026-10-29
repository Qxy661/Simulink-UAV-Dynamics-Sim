function ref = fw_mission(h0, V0)
%FW_MISSION  固定翼示例任务参考句柄。
%   ref = uav.env.fw_mission(h0, V0) 返回 @(t) 结构体，字段 h/psi/V。

if nargin < 1, h0 = 100; end
if nargin < 2, V0 = 18; end

ref = @(t) uav.env.fw_ref(t, h0, V0);
end
