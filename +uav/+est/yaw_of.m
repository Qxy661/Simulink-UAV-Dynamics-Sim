function psi = yaw_of(q)
%YAW_OF 由四元数取 ZYX 航向角（rad），范围 (-pi, pi]。

qw = q(1); qx = q(2); qy = q(3); qz = q(4);
psi = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2));
end
