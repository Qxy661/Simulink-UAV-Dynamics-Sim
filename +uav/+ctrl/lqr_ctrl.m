function [T, M, st] = lqr_ctrl(x, r, p, c, b, st, dt) %#ok<INUSD>
%LQR_CTRL  线性二次型状态反馈（悬停点线性化，12 状态）。
%   [T, M, st] = uav.ctrl.lqr_ctrl(x, r, p, c, b, st, dt)
%   输出总推力 T（N）与机体力矩 M（N*m），与 uav.ctrl.cascade 对齐。
%   增益在首次调用时由 dlqr 求出并缓存，之后只做一次矩阵乘法。

persistent K
if isempty(K)
    [Ad, Bd] = uav.ctrl.hover_model(p, b.Ts);
    K = dlqr(Ad, Bd, b.Qlqr, b.Rlqr);
end

att = uav.plant.quat2euler(x(7:10));
dpsi = uav.ctrl.wrap_pi(att(3) - r.psi);
xl = [x(1:3) - r.p; x(4:6) - r.v; att(1:2); dpsi; x(11:13)];

u = -K*xl + [p.mass*p.g; 0; 0; 0];

T = min(max(u(1), 0), c.maxT);
M = min(max(u(2:4), -c.maxM), c.maxM);
st = st;
end
