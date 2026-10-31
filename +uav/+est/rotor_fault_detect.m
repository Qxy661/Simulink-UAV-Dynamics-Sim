function [d, idx] = rotor_fault_detect(wCmd, wMeas, d)
%ROTOR_FAULT_DETECT 转速指令残差式的电机失效检测（计数式，给仿真与控制器用）。
%
%   d   = uav.est.rotor_fault_detect_init();
%   [d, idx] = uav.est.rotor_fault_detect(wCmd, wMeas, d);
%
%   wCmd  —— 分配器本拍给出的转速指令（4x1）
%   wMeas —— 实测转速（4x1），真机上来自电调回传或转速观测器
%   d     —— 检测器状态：cnt（各电机连续超限拍数）、dead（已判定失效的掩码）、
%            th（残差门限，默认 60 rad/s）、hold（连续多少拍才判定，默认 3）
%   idx   —— 本拍新判定失效的电机下标（无则空）
%
%   思路：正常电机的一阶响应在 20 ms 内跟上指令，残差是几十 rad/s 量级的
%   瞬态；电机完全停转时残差立刻等于指令值（数百 rad/s）。门限 60 rad/s
%   配 3 拍保持，能把指令阶跃的瞬态滤掉，检测延迟约 3–5 ms。
%
%   注意：这里假设控制器知道自己的指令，残差不需要额外传感器——电调回传
%   转速在真机上是标准配置（PX4 的 ESC telemetry）。

if nargin < 3 || isempty(d)
    d = struct('cnt', zeros(4,1), 'dead', false(4,1), 'th', 200, 'hold', 30);
end

r = abs(wCmd(:) - wMeas(:));
over = r > d.th;
d.cnt = (d.cnt + 1) .* over;              % 未超限则清零
newDead = (d.cnt >= d.hold) & ~d.dead;
idx = find(newDead);
if ~isempty(idx)
    d.dead(idx) = true;
end
end
