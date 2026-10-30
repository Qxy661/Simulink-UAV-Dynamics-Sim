function e = ekf_update_flow(e, vFlow, sigma)
%EKF_UPDATE_FLOW 光流水平速度量测（机体系 x/y 两轴）。
%   量测在机体系，量测矩阵要把 NED 速度误差转到机体系：H = R(1:2,:)。
%   vFlow 为 2x1（m/s），sigma 为标量或 2x1。

R = uav.plant.q2dcm(e.q);
H = zeros(2, 15);
H(1:2, 4:6) = R(1:2, 1:3);

if isscalar(sigma)
    sigma = [sigma; sigma];
end

vBodyPred = R(1:2, 1:3) * e.v;
e = uav.est.ekf_correct(e, vFlow(:) - vBodyPred, H, diag(sigma.^2));
end
