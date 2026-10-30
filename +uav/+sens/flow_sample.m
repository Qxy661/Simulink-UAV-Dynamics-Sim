function z = flow_sample(qTrue, vBodyTrue, sigma, scale)
%FLOW_SAMPLE 光流/视觉里程计的水平速度量测（机体系）。
%   真实光流测的是像面位移，需要高度与地面假设才能换成速度；这里按
%   已做过高度补偿的下视光流模块建模：输出机体系水平速度，带白噪声
%   与一个常值标度误差（scale，典型 ±2%）。
%   qTrue 只用于接口一致，当前模型不引入旋转误差。

if nargin < 3 || isempty(sigma)
    sigma = 0.05;
end
if nargin < 4 || isempty(scale)
    scale = 1.02;
end

z = scale * vBodyTrue(1:2) + sigma * randn(2, 1);
end
