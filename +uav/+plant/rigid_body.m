function xd = rigid_body(x, Fb, Mb, p)
%RIGID_BODY  刚体六自由度方程（机体 FRD，世界 NED）。
%   状态 x = [pn;pe;pd; u;v;w; qw;qx;qy;qz; p;q;r]。
%   Fb/Mb 为机体系合力与合力矩（不含重力）。

Vb = x(4:6);
q  = x(7:10);
w  = x(11:13);

R  = uav.plant.q2dcm(q);
Vn = R*Vb;

Vbd = Fb/p.mass - cross(w, Vb) + R'*[0; 0; p.g];
qd  = 0.5*uav.plant.qmul(q, [0; w]);
wd  = p.I \ (Mb - cross(w, p.I*w));

xd = [Vn; Vbd; qd; wd];
end
