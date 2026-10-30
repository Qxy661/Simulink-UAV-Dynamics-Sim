function e = ekf_update_baro(e, hMeas, sigma)
%EKF_UPDATE_BARO 气压高度量测。hMeas 为向上为正的高度，状态里 pd 向下，故 H 取 -1。

H = zeros(1, 15);
H(1, 3) = -1;
e = uav.est.ekf_correct(e, hMeas - (-e.p(3)), H, sigma^2);
end
