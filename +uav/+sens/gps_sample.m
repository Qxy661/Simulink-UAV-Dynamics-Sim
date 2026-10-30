function [zPos, zVel] = gps_sample(pTrue, sigmaH, sigmaV, vTrue, sigmaVel)
%GPS_SAMPLE 位置与地速量测，水平与垂直噪声分开给。
%   zVel 省略时按 sigmaVel=0.15 m/s 生成，模拟常见的 GNSS 地速输出。
%   给 nargout<2 只算位置，避免不需要速度的脚本多开销。

zPos = pTrue(:) + [sigmaH * randn(2, 1); sigmaV * randn(1, 1)];
if nargout < 2
    return;
end
if nargin < 5 || isempty(sigmaVel)
    sigmaVel = 0.15;
end
zVel = vTrue(:) + sigmaVel * randn(3, 1);
end
