function w = motor_step(w, wcmd, p, dt)
%MOTOR_STEP  一阶电机模型的精确离散更新。
%   tau*wdot + w = wcmd 的解析解，避免小时间常数下的显式欧拉不稳定。

alpha = exp(-dt/p.tauM);
w = wcmd + (w - wcmd)*alpha;
w = min(max(w, p.wMin), p.wMax);
end
