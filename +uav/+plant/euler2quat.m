function q = euler2quat(phi, theta, psi)
%EULER2QUAT  由 ZYX 欧拉角构造四元数（body 到 NED）。
%   q = qz(psi) * qy(theta) * qx(phi)

cx = cos(phi/2);   sx = sin(phi/2);
cy = cos(theta/2); sy = sin(theta/2);
cz = cos(psi/2);   sz = sin(psi/2);

qx = [cx; sx; 0; 0];
qy = [cy; 0; sy; 0];
qz = [cz; 0; 0; sz];

q = uav.plant.qmul(qz, uav.plant.qmul(qy, qx));
end
