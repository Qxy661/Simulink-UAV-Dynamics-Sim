function [cmd, st] = tilt_cascade(x, ref, st, p, c, dt)
%TILT_CASCADE  倾转四旋翼统一级联控制（空速调度 + 权限分配）。
%   x   : 21 维状态，前 13 维与四旋翼/固定翼一致，14 短舱角，15:18 旋翼转速，19:21 舵面
%   ref : 结构体，字段 p（NED 位置）、v、a、psi、h、V
%   cmd : [om(4); de; da; dr; betaCmd]，8 维
%
%   位置环先算期望加速度 a，再由推力需求 Tn=m*(a-g) 得到旋翼要补的力
%   Frot = R'*Tn - Fa。姿态指令按空速权重 k 在两支之间线性混合：
%     k=0 四旋翼支：机体 -z 轴对准 Tn；
%     k=1 固定翼支：俯仰维持配平迎角 + 高度/速度反馈，升力由机翼出。
%   硬性按约束 R*d(beta)=n 反解姿态在 cb≈0.3 处会跳变，实测把俯仰打到 -17°，
%   机翼迎角反号，越低头越掉高，所以改成混合而不是解约束。
%
%   U1 用投影而不是模长：U1=max(d'*Frot,0)，d=[sin b;0;-cos b]。模长会把
%   横向分量也算成推力，β=90° 时推力全给前向，转速顶到限幅收不回来。

q    = x(7:10);
R    = uav.plant.q2dcm(q);
att  = uav.plant.quat2euler(q);
psi  = att(3);
w    = x(11:13);
act  = x(19:21);
beta = x(14);

Vb    = x(4:6);
Vn    = R*Vb;
V     = norm(Vb);
alpha = atan2(Vb(3), Vb(1));
h     = -x(3);
hdot  = V*sin(att(2) - alpha);
qbar  = 0.5*p.rho*V^2;

% 转捩调度用参考空速而不是实测空速：实测空速低于计划时，若按它算 k，
% k 会回落，短舱跟着往回收，机体在过渡段来回摆。用 ref.V 做开环调度，
% 短舱按任务计划单调前倾，实测速度只作为内环的被控量。
k = (ref.V - p.V1) / (p.V2 - p.V1);
k = max(0, min(1, k));

% ---- 位置环 ----
ep = ref.p(:) - x(1:3);
ev = ref.v(:) - Vn;

st.ip = st.ip + ep*dt;
st.ip = max(-c.iMax, min(c.iMax, st.ip));

a = c.Kp.*ep + c.Kd.*ev + ref.a(:) + c.Ki.*st.ip;

nh = norm(a(1:2));
if nh > c.aHoriz
    a(1:2) = a(1:2) * (c.aHoriz/nh);
end
a(3) = max(-c.aVert, min(c.aVert, a(3)));

% 限速：位置误差大时位置环会一直给满加速度，速度一路超。超过参考两米每秒
% 就把沿速度方向的加速度压成"减速"，让速度回到参考附近。
Vref = norm(ref.v);
if V > Vref + 2 && V > 1
    ehat  = Vn / V;
    along = a' * ehat;
    aCap  = -(V - Vref - 2);
    if along > aCap
        a = a + ehat*(aCap - along);
    end
end

% ---- 需要的合力与旋翼份额 ----
Tn   = p.mass * (a - [0; 0; p.g]);        % NED，非重力合力（悬停时 [0;0;-mg]）
Tnb  = R' * Tn;

Fa = uav.plant.fw_aero(Vb, w, [0; act(:)], p);
Frot = Tnb - Fa;
if norm(Frot) < 1e-6
    Frot = [0; 0; -p.mass*p.g];
end

% ---- 姿态指令：四旋翼支与固定翼支按 k 混合 ----
qbarS  = max(qbar, 1.0);
CLreq  = p.mass*p.g / (qbarS*p.S);
alphaT = (CLreq - p.CL0) / p.CLa;
alphaT = max(c.alphaMin, min(c.alphaMax, alphaT));
alphaT = k * alphaT;           % 悬停时不能用机翼配平迎角

% 四旋翼支：机体 -z 轴对准 Tn
zq = -Tn / max(norm(Tn), 1e-9);
xc = [cos(psi); sin(psi); 0];
yb = cross(zq, xc);
ny = norm(yb);
if ny < 1e-6
    xc = [1; 0; 0];
    yb = cross(zq, xc);
    ny = norm(yb);
end
yb = yb / ny;
xb = cross(yb, zq);
Rdq = [xb yb zq];
phiQ   = atan2(Rdq(3,2), Rdq(3,3));
thetaQ = -asin(max(-1, min(1, Rdq(3,1))));

% 固定翼支：配平迎角 + 高度与空速反馈
eH  = ref.h - h;
st.iH = st.iH + eH*dt;
st.iH = max(-c.iHmax, min(c.iHmax, st.iH));

thetaF = alphaT + c.Kth_h*eH + c.Kth_hi*st.iH - c.Kth_hd*hdot + c.KthV*(V - ref.V);
thetaF = max(-c.thMax, min(c.thMax, thetaF));
phiF   = c.Kphi_psi * uav.ctrl.wrap_pi(ref.psi - psi);
phiF   = max(-c.phiMax, min(c.phiMax, phiF));

phiD   = (1 - k)*phiQ   + k*phiF;
thetaD = (1 - k)*thetaQ + k*thetaF;
psiD   = ref.psi;

% ---- 姿态环 ----
eAtt = uav.ctrl.wrap_pi([phiD; thetaD; psiD] - att);
st.ia = st.ia + eAtt*dt;
st.ia = max(-c.iMaxAtt, min(c.iMaxAtt, st.ia));
Mraw = c.KpAtt.*eAtt + c.KiAtt.*st.ia - c.KdAtt.*w;

% 舵面支的力矩增益要随动压放大：力矩到舵偏是 de = M/(qbar*S*c*Cmde)，
% 动压从 20 Pa 涨到 120 Pa 时同一个 M 只能给出 1/6 的舵偏。不补这一项，
% 巡航段 30° 的俯仰误差只换来 2° 舵偏，机体一路抬头爬高。
wRot0 = max(0, 1 - k);
gb    = max(1, max(qbarS, 1.0)/c.qAttRef);
M     = wRot0*Mraw + (1 - wRot0)*gb*Mraw;

% 力矩预算随权限走：悬停时惯量小，几 N·m 就够；巡航动压上来后舵面能给出
% 几十 N·m，若仍按悬停的 maxM 截断，俯仰指令同样会被压住。
MfLim = [qbarS*p.S*p.b*p.Clda*p.daMax;
         qbarS*p.S*p.c*abs(p.Cmde)*p.deMax;
         qbarS*p.S*p.b*p.Cndr*p.drMax];
Mlim  = wRot0*c.maxM + (1 - wRot0)*MfLim;
M = max(-Mlim, min(Mlim, M));

% ---- 分配：旋翼份额与舵面份额 ----
sb = sin(beta);
cb = cos(beta);
d  = [sb; 0; -cb];
U1max = 4*p.kt*p.wMax^2;
U1v = norm(Frot);
U1f = max(0, d'*Frot);
U1 = max(0, min(U1max, (1 - k)*U1v + k*U1f));

wRot = max(0, 1 - k);
Mxr = wRot * M(1);
Myr = wRot * M(2);
Mzr = wRot * M(3);

U2 = cb*Mxr + sb*Mzr;
U3 = Myr / max(cb, 0.20);
U4 = (-sb*Mxr + cb*Mzr) / p.kq;

[om, ~] = uav.plant.tilt_mix(U1, U2, U3, U4, p);

qs = max(qbar, 1.0);
da = (1 - wRot) * M(1) / (qs*p.S*p.b*p.Clda);
de = (1 - wRot) * M(2) / (qs*p.S*p.c*p.Cmde);
dr = (1 - wRot) * M(3) / (qs*p.S*p.b*p.Cndr);

actCmd = [ max(-p.deMax, min(p.deMax, de));
           max(-p.daMax, min(p.daMax, da));
           max(-p.drMax, min(p.drMax, dr)) ];

betaCmd = p.betaMin + (p.betaMax - p.betaMin)*k;

% 早期这里写 st.dbg = [a(:); phiD; thetaD; U1; k] 用于调参，
% 代码生成不允许在结构体被读取后再加字段，已删除；需要诊断时临时加回来。

cmd = [om(:); actCmd(:); betaCmd];
end
