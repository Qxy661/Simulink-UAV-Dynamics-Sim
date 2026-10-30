function gust = wind_dryden(tEnd, dt, V, sigma, seed, Wmean)
%WIND_DRYDEN  Dryden 大气紊流的离散时间序列（NED）。
%   gust = wind_dryden(tEnd, dt, V, sigma, seed) 返回 struct：
%     gust.t  1xN 时间
%     gust.W  Nx3 风速（NED，m/s）
%     gust.V  参考空速
%   V 为参考空速（m/s），sigma 为纵向湍流强度（m/s），低空取
%   Lu=200 m、Lv=200 m、Lw=50 m。三个轴用一阶成形滤波器独立生成：
%       x[k+1] = a*x[k] + b*randn,  a = exp(-V*dt/L),  b = sigma*sqrt(1-a^2)
%   这样得到的是稳态方差 sigma^2 的有色噪声，功率谱与 Dryden 一致。

if nargin < 6 || isempty(Wmean)
    Wmean = [0; 0; 0];
end
if nargin < 5 || isempty(seed)
    seed = 11;
end
if nargin < 4 || isempty(sigma)
    sigma = 2.0;
end
if nargin < 3 || isempty(V)
    V = 10;
end

Lu = 200; Lv = 200; Lw = 50;
L = [Lu; Lv; Lw];
n = round(tEnd / dt) + 1;

rng(seed, 'twister');
W = repmat(Wmean(:)', n, 1);
x = zeros(3, 1);
for k = 2:n
    a = exp(-V * dt ./ L);
    b = sigma * sqrt(1 - a.^2);
    x = a .* x + b .* randn(3, 1);
    W(k, :) = (x + Wmean(:))';
end

gust.t = (0:n-1) * dt;
gust.W = W;
gust.V = V;
gust.sigma = sigma;
gust.Wmean = Wmean(:);
end
