function e = ekf_correct(e, y, H, Rm)
%EKF_CORRECT 通用量测更新。
%   y  残差（量测减去预测），H 量测矩阵（对 15 维误差状态），Rm 量测噪声协方差。
%   用 Joseph 形式更新协方差，避免长时间运行后 P 丢掉正定性。
%
%   见 docs/estimation.md 的"为什么用 Joseph 形式"。

S = H * e.P * H' + Rm;
S = (S + S') / 2;
K = e.P * H' / S;
dx = K * y;

A = eye(15) - K * H;
e.P = A * e.P * A' + K * Rm * K';
e.P = (e.P + e.P') / 2;

e.p = e.p + dx(1:3);
e.v = e.v + dx(4:6);
dq = [1; 0.5 * dx(7:9)];
e.q = uav.plant.qmul(e.q, dq);
e.q = e.q / norm(e.q);
e.bg = e.bg + dx(10:12);
e.ba = e.ba + dx(13:15);
end
