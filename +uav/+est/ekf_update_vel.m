function e = ekf_update_vel(e, vMeas, sigma)
%EKF_UPDATE_VEL 速度量测（NED，三轴标准差 sigma）。
%   GNSS 接收机在给位置的同时给地速，精度通常比位置差分高一个量级。
%   sigma 可传标量或 [sigmaN; sigmaE; sigmaD]。

if isscalar(sigma)
    sigma = [sigma; sigma; sigma];
end

H = zeros(3, 15);
H(1:3, 4:6) = eye(3);
Rm = diag(sigma.^2);
e = uav.est.ekf_correct(e, vMeas(:) - e.v, H, Rm);
end
