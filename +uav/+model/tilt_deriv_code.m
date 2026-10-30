function code = tilt_deriv_code(p)
%TILT_DERIV_CODE  生成倾转四旋翼 Simulink 图表的导数源码。
%   与 uav.plant.tilt_wrench + uav.plant.rigid_body 同一套算式，
%   再加短舱、转速、舵面的一阶滞后。21 维状态导数一次算完，纯函数、可被 RK4 反复调用。

L = { ...
    'function xd = tiltderiv(x, cmd)'
    '% 21 维导数：13 刚体 + 短舱 + 4 转速 + 3 舵面，含一阶滞后'};
L{end+1} = 'Vb = x(4:6); q = x(7:10); w = x(11:13);';
L{end+1} = 'beta = x(14); om = x(15:18); act = x(19:21);';
L{end+1} = 'R = dcm(q);';
L{end+1} = 'u = Vb(1); vv = Vb(2); wb = Vb(3);';
L{end+1} = 'pp = w(1); qq = w(2); rr = w(3);';
L{end+1} = 'de = act(1); da = act(2); dr = act(3);';
L{end+1} = 'V = norm(Vb); Ve = max(V, 1.0);';
L{end+1} = 'alpha = asin(max(-0.34, min(0.34, wb/Ve)));';
L{end+1} = 'bsa = asin(max(-0.34, min(0.34, vv/Ve)));';
L{end+1} = sprintf('qbar = 0.5*%s*V^2;', uav.model.fmt(p.rho));

% 气动
L{end+1} = sprintf('CL = %s + %s*alpha + %s*qq*%s/(2*Ve) + %s*de;', ...
    uav.model.fmt(p.CL0), uav.model.fmt(p.CLa), uav.model.fmt(p.CLq), ...
    uav.model.fmt(p.c), uav.model.fmt(p.CLde));
L{end+1} = sprintf('CD = %s + %s*CL^2;', uav.model.fmt(p.CD0), uav.model.fmt(p.kind));
L{end+1} = sprintf('CY = %s*bsa + %s*dr;', uav.model.fmt(p.CYb), uav.model.fmt(p.CYdr));
L{end+1} = sprintf('D = qbar*%s*CD; L = qbar*%s*CL; Y = qbar*%s*CY;', ...
    uav.model.fmt(p.S), uav.model.fmt(p.S), uav.model.fmt(p.S));
L{end+1} = 'Xa = -D*cos(alpha) + L*sin(alpha);';
L{end+1} = 'Za = -D*sin(alpha) - L*cos(alpha);';
L{end+1} = 'Fa = [Xa; Y; Za];';
L{end+1} = sprintf('Ma1 = qbar*%s*%s*(%s*bsa + %s*pp*%s/(2*Ve) + %s*rr*%s/(2*Ve) + %s*da);', ...
    uav.model.fmt(p.S), uav.model.fmt(p.b), uav.model.fmt(p.Clb), uav.model.fmt(p.Clp), ...
    uav.model.fmt(p.b), uav.model.fmt(p.Clr), uav.model.fmt(p.b), uav.model.fmt(p.Clda));
L{end+1} = sprintf('Ma2 = qbar*%s*%s*(%s + %s*alpha + %s*qq*%s/(2*Ve) + %s*de);', ...
    uav.model.fmt(p.S), uav.model.fmt(p.c), uav.model.fmt(p.Cm0), uav.model.fmt(p.Cma), ...
    uav.model.fmt(p.Cmq), uav.model.fmt(p.c), uav.model.fmt(p.Cmde));
L{end+1} = sprintf('Ma3 = qbar*%s*%s*(%s*bsa + %s*pp*%s/(2*Ve) + %s*rr*%s/(2*Ve) + %s*dr);', ...
    uav.model.fmt(p.S), uav.model.fmt(p.b), uav.model.fmt(p.Cnb), uav.model.fmt(p.Cnp), ...
    uav.model.fmt(p.b), uav.model.fmt(p.Cnr), uav.model.fmt(p.b), uav.model.fmt(p.Cndr));
L{end+1} = 'Ma = [Ma1; Ma2; Ma3];';

% 旋翼
L{end+1} = sprintf('T = %s*om.^2;', uav.model.fmt(p.kt));
L{end+1} = 'U1 = sum(T);';
L{end+1} = sprintf('U2 = -sum([%s; %s; %s; %s].*T);', ...
    uav.model.fmtv(p.ry(1)), uav.model.fmtv(p.ry(2)), uav.model.fmtv(p.ry(3)), uav.model.fmtv(p.ry(4)));
L{end+1} = sprintf('U3 = sum([%s; %s; %s; %s].*T);', ...
    uav.model.fmtv(p.rx(1)), uav.model.fmtv(p.rx(2)), uav.model.fmtv(p.rx(3)), uav.model.fmtv(p.rx(4)));
L{end+1} = sprintf('U4 = sum([%s; %s; %s; %s].*T);', ...
    uav.model.fmtv(p.spin(1)), uav.model.fmtv(p.spin(2)), uav.model.fmtv(p.spin(3)), uav.model.fmtv(p.spin(4)));
L{end+1} = 'sb = sin(beta); cb = cos(beta);';
L{end+1} = 'Fr = [sb; 0; -cb]*U1;';
L{end+1} = sprintf('Mr = [cb*U2 - %s*sb*U4; cb*U3; sb*U2 + %s*cb*U4];', ...
    uav.model.fmt(p.kq), uav.model.fmt(p.kq));
L{end+1} = 'Fb = Fa + Fr;';
L{end+1} = 'Mb = Ma + Mr;';

% 刚体
L{end+1} = 'Vn = R*Vb;';
L{end+1} = sprintf('Vbd = Fb/%s - cross(w, Vb) + R''*[0; 0; %s];', ...
    uav.model.fmt(p.mass), uav.model.fmt(p.g));
L{end+1} = 'qd = 0.5*qmul(q, [0; w]);';
L{end+1} = sprintf('Imat = [%s 0 0; 0 %s 0; 0 0 %s];', ...
    uav.model.fmt(p.I(1,1)), uav.model.fmt(p.I(2,2)), uav.model.fmt(p.I(3,3)));
L{end+1} = 'wd = Imat \ (Mb - cross(w, Imat*w));';

% 执行机构滞后
L{end+1} = sprintf('betaDot = (cmd(8) - beta)/%s;', uav.model.fmt(p.tauBeta));
L{end+1} = sprintf('betaDot = min(max(betaDot, -%s), %s);', ...
    uav.model.fmt(p.betaRate), uav.model.fmt(p.betaRate));
L{end+1} = sprintf('omDot = (cmd(1:4) - om)/%s;', uav.model.fmt(p.tauM));
L{end+1} = sprintf('actDot = (cmd(5:7) - act)/%s;', uav.model.fmt(p.tauA));
L{end+1} = 'xd = [Vn; Vbd; qd; wd; betaDot; omDot; actDot];';
L{end+1} = 'end';

code = strjoin([L; uav.model.dcm_lines(); uav.model.qmul_lines()], newline);
end
