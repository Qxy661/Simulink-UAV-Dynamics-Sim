function e = ekf_update_mag(e, mMeas, sigma, mRef)
%EKF_UPDATE_MAG 三轴磁力计更新姿态。
%   量测模型：m_body = R(q)' * mRef，mRef 为当地磁场单位方向（NED）。
%   姿态误差取机体系小角度 delta（q_true = q_est ⊗ delta），则
%       m_true = R' mRef - delta × (R' mRef) = m_pred + m_pred × delta
%   所以量测矩阵在姿态误差三列上就是 skew(m_pred)。比"直接量航向角"稳，
%   大倾角下也不会把偏航修正注入横滚/俯仰。
%
%   见 docs/estimation.md。

mRef = mRef(:) / norm(mRef);
mMeas = mMeas(:) / norm(mMeas);

R = uav.plant.q2dcm(e.q);
mPred = R' * mRef;

H = zeros(3, 15);
H(:, 7:9) = uav.est.skew(mPred);

e = uav.est.ekf_correct(e, mMeas - mPred, H, eye(3) * sigma^2);
end
