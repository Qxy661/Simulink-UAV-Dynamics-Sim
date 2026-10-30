function e = ekf_update_gps(e, pMeas, sigma)
%EKF_UPDATE_GPS 位置量测（NED，三轴标准差 sigma）。
%   水平与垂直噪声不同时，sigma 可传 [sigmaH; sigmaH; sigmaV]。

if isscalar(sigma)
    sigma = [sigma; sigma; sigma];
end

H = zeros(3, 15);
H(1:3, 1:3) = eye(3);
Rm = diag(sigma.^2);
e = uav.est.ekf_correct(e, pMeas(:) - e.p, H, Rm);
end
