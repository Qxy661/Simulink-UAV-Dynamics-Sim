function z = imu_sample(gyroTrue, accelTrue, bg, ba, sigmaG, sigmaA)
%IMU_SAMPLE 一帧理想 IMU 读数加零偏与白噪声。
%   gyroTrue/accelTrue 为机体系真值；bg/ba 为常值零偏；sigma 为该采样率下的
%   单样本标准差（由噪声密度按 sqrt 带宽折算得到，见 docs/estimation.md）。

z.gyro = gyroTrue(:) + bg(:) + sigmaG * randn(3, 1);
z.accel = accelTrue(:) + ba(:) + sigmaA * randn(3, 1);
end
