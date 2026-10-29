function [T, M, st] = mpc_ctrl(x, r, p, c, b, st, dt) %#ok<INUSD>
%MPC_CTRL  线性模型预测控制（悬停点模型，12 状态、4 输入）。
%   [T, M, st] = uav.ctrl.mpc_ctrl(x, r, p, c, b, st, dt)
%   把预测时域内的状态约束展开成二次型，用 quadprog 解一次 QP。
%   输入 4 维为 [dT; Mx; My; Mz]，其中 dT 是相对悬停推力的增量，
%   输出总推力 T = m*g + dT。控制器采样周期取 b.Ts（默认 0.01 s）。
%   不依赖 MPC 工具箱，只用到 Optimization Toolbox 的 quadprog。

persistent Ad Bd Sx Su Qb Rb lb ub
n  = 12;
m  = 4;
Np = b.Np;
Nc = b.Nc;

if isempty(Ad)
    [Ad, Bd] = uav.ctrl.hover_model(p, b.Ts);

    Sx = zeros(Np*n, n);
    Su = zeros(Np*n, Nc*m);
    for i = 1:Np
        Sx((i-1)*n + (1:n), :) = Ad^i;
        for j = 0:min(i, Nc) - 1
            Su((i-1)*n + (1:n), j*m + (1:m)) = Ad^(i-1-j)*Bd;
        end
    end

    Qb = kron(eye(Np), diag(b.Wy));
    Rb = kron(eye(Nc), diag(b.Ru));
    lb = repmat([-p.mass*p.g; -c.maxM], Nc, 1);
    ub = repmat([c.maxT - p.mass*p.g; c.maxM], Nc, 1);
end

att  = uav.plant.quat2euler(x(7:10));
x0   = [x(1:3); x(4:6); att; x(11:13)];
xr   = [r.p; r.v; 0; 0; r.psi; 0; 0; 0];
Xref = repmat(xr, Np, 1);

H = Su'*Qb*Su + Rb;
H = (H + H')/2;                 % 消除累加误差带来的非对称，quadprog 会拒绝
f = Su'*Qb*(Sx*x0 - Xref);

opt = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
u = quadprog(H, f, [], [], [], [], lb, ub, [], opt);

T = min(max(p.mass*p.g + u(1), 0), c.maxT);
M = min(max(u(2:4), -c.maxM), c.maxM);
st = st;
end
