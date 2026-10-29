function y = wrap_pi(x)
%WRAP_PI  角度归一化到 (-pi, pi]。

y = mod(x + pi, 2*pi) - pi;
y(y <= -pi) = pi;
end
