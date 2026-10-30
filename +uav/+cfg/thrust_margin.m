function m = thrust_margin(p)
%THRUST_MARGIN 单桨失效的推力裕度：单台最大推力 / 悬停所需单台推力。
%
%   m = uav.cfg.thrust_margin(p)
%
%   四旋翼布局单台失效后，剩下三台里对角两台要承担 mg/2 才能同时满足
%   总推力与滚转/俯仰零力矩（推导见 research/REPORT_r6_hover_fault.md）。
%   所以「能悬停存活」的判据是 m >= 2，等价于 wMax >= sqrt(2)*hoverW。
%
%   本仓库的 tilt 配置 m = 1.60，不满足；这是设计取舍，不是 bug。

if nargin < 1 || isempty(p)
    p = uav.cfg.tilt();
end
m = (p.kt*p.wMax^2) / (p.mass*p.g/4);
end
