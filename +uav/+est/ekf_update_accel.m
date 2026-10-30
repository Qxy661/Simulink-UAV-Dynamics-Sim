function e = ekf_update_accel(e, fMeas, sigma, gate, rate, rateGate)
%EKF_UPDATE_ACCEL 用重力方向修正横滚与俯仰。
%   准静态时加速度计读到的比力是重力反方向在机体系的分量：
%       f_pred = R(q)' * [0;0;-9.81]
%   与磁力计同理，量测矩阵取 skew(f_pred)。比力模长偏离 g 超过 gate 时不更新
%   ——剧烈机动时读数里混进了线加速度，硬用会把姿态拉歪。
%
%   见 docs/estimation.md 的"加速度计为什么能修正倾角"。

if nargin < 4 || isempty(gate)
    gate = 0.3;
end
if nargin < 6 || isempty(rateGate)
    rateGate = 0.3;
end
if nargin >= 5 && ~isempty(rate) && norm(rate) > rateGate
    return;
end

nf = norm(fMeas);
if abs(nf - 9.81) > gate
    return;
end

R = uav.plant.q2dcm(e.q);
fPred = R' * [0; 0; -9.81];

H = zeros(3, 15);
H(:, 7:9) = uav.est.skew(fPred);

e = uav.est.ekf_correct(e, fMeas(:) - fPred, H, eye(3) * sigma^2);
end
