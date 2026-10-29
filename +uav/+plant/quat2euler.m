function e = quat2euler(q)
%QUAT2EULER  四元数转 ZYX 欧拉角 [phi; theta; psi]，单位 rad。
%   输入四元数为机体系 -> NED，奇异点 theta = +-pi/2 处 theta 被截断。

n = norm(q);
if n < 1e-12
    e = [0; 0; 0];
    return
end
q = q/n;
w = q(1); x = q(2); y = q(3); z = q(4);

sinTheta = 2*(w*y - z*x);
sinTheta = max(-1, min(1, sinTheta));

phi   = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
theta = asin(sinTheta);
psi   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
e = [phi; theta; psi];
end
