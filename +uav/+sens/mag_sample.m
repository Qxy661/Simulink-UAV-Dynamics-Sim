function z = mag_sample(qTrue, mRef, sigma)
%MAG_SAMPLE 机体三轴磁力计读数。
%   mRef 为当地磁场在 NED 下的单位方向（默认取 [0.42;0;0.91]，中间纬度近似），
%   机体读数 = R(q)'*mRef 加白噪声。

if nargin < 2 || isempty(mRef)
    mRef = [0.42; 0; 0.91];
end
mRef = mRef(:) / norm(mRef);
R = uav.plant.q2dcm(qTrue);
z = R' * mRef + sigma * randn(3, 1);
end