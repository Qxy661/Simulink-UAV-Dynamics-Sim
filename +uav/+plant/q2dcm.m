function R = q2dcm(q)
%Q2DCM  四元数转旋转矩阵（机体系 -> NED）。
%   q = [w; x; y; z]，要求单位化；非单位输入按单位处理。
%   对应 ZYX 欧拉角：R = Rz(psi)*Ry(theta)*Rx(phi)。

n = norm(q);
if n < 1e-12
    q = [1; 0; 0; 0];
else
    q = q/n;
end

w = q(1); x = q(2); y = q(3); z = q(4);

R = [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y);
     2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x);
     2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)];
end
