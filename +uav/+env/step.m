function ref = step(t, t0, p0, p1, psi)
%STEP  位置阶跃参考，t0 之前保持 p0，之后保持 p1。
%   p0/p1 为 NED 列向量；速度与加速度参考为零。

if t < t0
    ref = uav.env.hover(p0(1), p0(2), p0(3), psi);
else
    ref = uav.env.hover(p1(1), p1(2), p1(3), psi);
end
end
