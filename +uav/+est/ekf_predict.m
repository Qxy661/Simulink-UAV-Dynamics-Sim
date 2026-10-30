function e = ekf_predict(e, gyro, accel, dt, nz)
%EKF_PREDICT 用一帧 IMU 数据推进 EKF。
%   gyro 为机体系角速度（rad/s），accel 为机体系比力（m/s^2，含重力反作用），
%   都已叠加零偏与噪声。nz 给出各噪声密度：
%       nz.sigmaA  加速度计白噪声（m/s^2，按 1 Hz 带宽折算）
%       nz.sigmaG  陀螺白噪声（rad/s）
%       nz.sigmaBg 陀螺零偏随机游走（rad/s/sqrt(s)）
%       nz.sigmaBa 加计零偏随机游走（m/s^2/sqrt(s)）
%
%   误差状态转移矩阵按连续方程离散：
%       d(theta)/dt = -bg,  d(v)/dt = -R [f]x theta - R ba,  d(p)/dt = v

R = uav.plant.q2dcm(e.q);
w = gyro(:) - e.bg;
f = accel(:) - e.ba;

G = [1; 0.5 * w * dt];
e.q = uav.plant.qmul(e.q, G);
e.q = e.q / norm(e.q);

aN = R * f + [0; 0; 9.81];
e.v = e.v + aN * dt;
e.p = e.p + e.v * dt;

F = eye(15);
F(1:3, 4:6) = eye(3) * dt;
F(4:6, 7:9) = -R * uav.est.skew(f) * dt;
F(4:6, 13:15) = -R * dt;
F(7:9, 10:12) = -eye(3) * dt;

Q = zeros(15);
Q(4:6, 4:6) = eye(3) * (nz.sigmaA^2 * dt);
Q(7:9, 7:9) = eye(3) * (nz.sigmaG^2 * dt);
Q(10:12, 10:12) = eye(3) * (nz.sigmaBg^2 * dt);
Q(13:15, 13:15) = eye(3) * (nz.sigmaBa^2 * dt);

e.P = F * e.P * F' + Q;
e.P = (e.P + e.P') / 2;
e.n = e.n + 1;
end
