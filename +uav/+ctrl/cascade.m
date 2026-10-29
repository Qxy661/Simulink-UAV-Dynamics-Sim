function [Tcmd, Mcmd, st] = cascade(x, ref, p, c, st, dt)
%CASCADE  位置-姿态级联控制器。
%   位置环在 NED 下算期望加速度，再反解推力方向与期望姿态；
%   姿态环用欧拉角误差 + 机体角速度阻尼，输出机体力矩。
%   st 为控制器状态（积分项），由调用方持有，避免全局变量。

pn = x(1:3);
Vb = x(4:6);
q  = x(7:10);
w  = x(11:13);

R  = uav.plant.q2dcm(q);
Vn = R*Vb;

% 位置环
ep = ref.p - pn;
ev = ref.v - Vn;

a = c.Kp.*ep + c.Kd.*ev + c.Ki.*st.ip + ref.a;

% 倾角限幅：只压水平分量，保留高度通道
aHor = a(1:2);
aMax = p.g*tan(c.maxTilt);
if norm(aHor) > aMax
    a(1:2) = aHor*(aMax/norm(aHor));
end

st.ip = st.ip + ep*dt;
st.ip = min(max(st.ip, -c.iMax), c.iMax);

% 推力方向：机体 -z 轴指向 (g_n - a)，即期望机体 z 轴为 z
v = [0; 0; p.g] - a;
nv = norm(v);
if nv < 1e-6
    v = [0; 0; 1];
    nv = 1;
end
z = v/nv;

Tcmd = p.mass*nv;
Tcmd = min(max(Tcmd, 0), c.maxT);

% 由 z 轴与航向构造期望姿态，避免大航向下欧拉角反解耦合
xc = [cos(ref.psi); sin(ref.psi); 0];
yb = cross(z, xc);
ny = norm(yb);
if ny < 1e-6
    xc = [1; 0; 0];
    yb = cross(z, xc);
    ny = norm(yb);
end
yb = yb/ny;
xb = cross(yb, z);
Rd = [xb, yb, z];

phiD   =  atan2(Rd(3, 2), Rd(3, 3));
thetaD = -asin(max(-1, min(1, Rd(3, 1))));
psiD   =  atan2(Rd(2, 1), Rd(1, 1));

% 姿态环
att = uav.plant.quat2euler(q);
eAtt = uav.ctrl.wrap_pi([phiD; thetaD; psiD] - att);

st.ia = st.ia + eAtt*dt;
st.ia = min(max(st.ia, -c.iMaxAtt), c.iMaxAtt);

Mcmd = c.KpAtt.*eAtt + c.KiAtt.*st.ia - c.KdAtt.*w;
Mcmd = min(max(Mcmd, -c.maxM), c.maxM);
end
