function a = actuator_step(cmd, a, p, dt)
%ACTUATOR_STEP  舵机与油门的一阶滞后（精确离散）。
%   cmd : [throttle; de; da; dr] 指令
%   a   : 上一拍的执行机构位置
%   时间常数：油门 tauT，三个舵面 tauA。

tau = [p.tauT; p.tauA; p.tauA; p.tauA];
k   = exp(-dt ./ tau);
a   = cmd + (a - cmd) .* k;
end
