function code = tilt_ctrl_code(p, c)
%TILT_CTRL_CODE  生成倾转四旋翼 Simulink 图表的控制器源码。
%   与 uav.ctrl.tilt_cascade 同一套算式，参数写成字面量，图表内不调用外部函数。
%   输入 x(21) 与 r(12)，输出 cmd(8) = [om(4); de; da; dr; betaCmd]。

L = { ...
    'function cmd = tiltctrl(x, r)'
    '% 倾转四旋翼级联控制：位置环 -> 姿态混合 -> 权限分配'};
L{end+1} = 'persistent ip ia iH';
L{end+1} = 'if isempty(ip)';
L{end+1} = '    ip = zeros(3,1); ia = zeros(3,1); iH = 0;';
L{end+1} = 'end';
L{end+1} = 'dt = 0.001;';
L{end+1} = 'Vb = x(4:6);';
L{end+1} = 'q  = x(7:10);';
L{end+1} = 'w  = x(11:13);';
L{end+1} = 'beta = x(14);';
L{end+1} = 'act  = x(19:21);';
L{end+1} = 'R  = dcm(q);';
L{end+1} = 'att = euler(q);';
L{end+1} = 'psi = att(3);';
L{end+1} = 'Vn = R*Vb;';
L{end+1} = 'V  = norm(Vb);';
L{end+1} = 'alpha = atan2(Vb(3), Vb(1));';
L{end+1} = 'h = -x(3);';
L{end+1} = 'hdot = V*sin(att(2) - alpha);';
L{end+1} = sprintf('qbar = 0.5*%s*V^2;', uav.model.fmt(p.rho));
L{end+1} = sprintf('k = (r(12) - %s)/(%s - %s);', ...
    uav.model.fmt(p.V1), uav.model.fmt(p.V2), uav.model.fmt(p.V1));
L{end+1} = 'k = min(max(k, 0), 1);';

% 位置环
L{end+1} = 'ep = r(1:3) - x(1:3);';
L{end+1} = 'ev = r(4:6) - Vn;';
L{end+1} = 'ip = ip + ep*dt;';
L{end+1} = sprintf('ip = min(max(ip, -%s), %s);', ...
    uav.model.fmtv(c.iMax), uav.model.fmtv(c.iMax));
L{end+1} = sprintf('a = %s.*ep + %s.*ev + r(7:9) + %s.*ip;', ...
    uav.model.fmtv(c.Kp), uav.model.fmtv(c.Kd), uav.model.fmtv(c.Ki));
L{end+1} = 'nh = norm(a(1:2));';
L{end+1} = sprintf('if nh > %s', uav.model.fmt(c.aHoriz));
L{end+1} = sprintf('    a(1:2) = a(1:2)*(%s/nh);', uav.model.fmt(c.aHoriz));
L{end+1} = 'end';
L{end+1} = sprintf('a(3) = min(max(a(3), -%s), %s);', ...
    uav.model.fmt(c.aVert), uav.model.fmt(c.aVert));
L{end+1} = 'Vref = norm(r(4:6));';
L{end+1} = 'if V > Vref + 2 && V > 1';
L{end+1} = '    ehat = Vn/V;';
L{end+1} = '    along = a''*ehat;';
L{end+1} = '    aCap = -(V - Vref - 2);';
L{end+1} = '    if along > aCap';
L{end+1} = '        a = a + ehat*(aCap - along);';
L{end+1} = '    end';
L{end+1} = 'end';

% 合力与旋翼份额
L{end+1} = sprintf('Tn = %s*(a - [0; 0; %s]);', uav.model.fmt(p.mass), uav.model.fmt(p.g));
L{end+1} = 'Tnb = R''*Tn;';
L{end+1} = 'Fa = aero(Vb, w, act);';
L{end+1} = 'Frot = Tnb - Fa;';
L{end+1} = 'if norm(Frot) < 1e-6';
L{end+1} = sprintf('    Frot = [0; 0; -%s*%s];', uav.model.fmt(p.mass), uav.model.fmt(p.g));
L{end+1} = 'end';

% 姿态指令混合
L{end+1} = 'qbarS = max(qbar, 1.0);';
L{end+1} = sprintf('CLreq = %s*%s/(qbarS*%s);', ...
    uav.model.fmt(p.mass), uav.model.fmt(p.g), uav.model.fmt(p.S));
L{end+1} = sprintf('alphaT = (CLreq - %s)/%s;', uav.model.fmt(p.CL0), uav.model.fmt(p.CLa));
L{end+1} = sprintf('alphaT = min(max(alphaT, %s), %s);', ...
    uav.model.fmt(c.alphaMin), uav.model.fmt(c.alphaMax));
L{end+1} = 'alphaT = k*alphaT;';

L{end+1} = 'zq = -Tn/max(norm(Tn), 1e-9);';
L{end+1} = 'xc = [cos(psi); sin(psi); 0];';
L{end+1} = 'yb = cross(zq, xc);';
L{end+1} = 'ny = norm(yb);';
L{end+1} = 'if ny < 1e-6';
L{end+1} = '    xc = [1; 0; 0];';
L{end+1} = '    yb = cross(zq, xc);';
L{end+1} = '    ny = norm(yb);';
L{end+1} = 'end';
L{end+1} = 'yb = yb/ny;';
L{end+1} = 'xb = cross(yb, zq);';
L{end+1} = 'Rdq = [xb yb zq];';
L{end+1} = 'phiQ = atan2(Rdq(3,2), Rdq(3,3));';
L{end+1} = 'thetaQ = -asin(min(max(Rdq(3,1), -1), 1));';

L{end+1} = 'eH = r(11) - h;';
L{end+1} = 'iH = iH + eH*dt;';
L{end+1} = sprintf('iH = min(max(iH, -%s), %s);', ...
    uav.model.fmt(c.iHmax), uav.model.fmt(c.iHmax));
L{end+1} = sprintf('thetaF = alphaT + %s*eH + %s*iH - %s*hdot + %s*(V - r(12));', ...
    uav.model.fmt(c.Kth_h), uav.model.fmt(c.Kth_hi), ...
    uav.model.fmt(c.Kth_hd), uav.model.fmt(c.KthV));
L{end+1} = sprintf('thetaF = min(max(thetaF, -%s), %s);', ...
    uav.model.fmt(c.thMax), uav.model.fmt(c.thMax));
L{end+1} = sprintf('ePsi = mod(r(10) - psi + pi, 2*pi) - pi;');
L{end+1} = sprintf('phiF = %s*ePsi;', uav.model.fmt(c.Kphi_psi));
L{end+1} = sprintf('phiF = min(max(phiF, -%s), %s);', ...
    uav.model.fmt(c.phiMax), uav.model.fmt(c.phiMax));

L{end+1} = 'phiD   = (1 - k)*phiQ + k*phiF;';
L{end+1} = 'thetaD = (1 - k)*thetaQ + k*thetaF;';
L{end+1} = 'psiD   = r(10);';

% 姿态环
L{end+1} = 'eAtt = mod([phiD; thetaD; psiD] - att + pi, 2*pi) - pi;';
L{end+1} = 'ia = ia + eAtt*dt;';
L{end+1} = sprintf('ia = min(max(ia, -%s), %s);', ...
    uav.model.fmtv(c.iMaxAtt), uav.model.fmtv(c.iMaxAtt));
L{end+1} = sprintf('Mraw = %s.*eAtt + %s.*ia - %s.*w;', ...
    uav.model.fmtv(c.KpAtt), uav.model.fmtv(c.KiAtt), uav.model.fmtv(c.KdAtt));
L{end+1} = 'wRot0 = max(0, 1 - k);';
L{end+1} = sprintf('gb = max(1, max(qbar, 1.0)/%s);', uav.model.fmt(c.qAttRef));
L{end+1} = 'M = wRot0*Mraw + (1 - wRot0)*gb*Mraw;';
L{end+1} = sprintf('MfLim = [qbar*%s*%s*%s*%s; qbar*%s*%s*%s*%s; qbar*%s*%s*%s*%s];', ...
    uav.model.fmt(p.S), uav.model.fmt(p.b), uav.model.fmt(p.Clda), uav.model.fmt(p.daMax), ...
    uav.model.fmt(p.S), uav.model.fmt(p.c), uav.model.fmt(abs(p.Cmde)), uav.model.fmt(p.deMax), ...
    uav.model.fmt(p.S), uav.model.fmt(p.b), uav.model.fmt(p.Cndr), uav.model.fmt(p.drMax));
L{end+1} = sprintf('Mlim = wRot0*%s + (1 - wRot0)*MfLim;', uav.model.fmtv(c.maxM));
L{end+1} = 'M = min(max(M, -Mlim), Mlim);';

% 分配
L{end+1} = 'sb = sin(beta); cb = cos(beta);';
L{end+1} = 'd = [sb; 0; -cb];';
L{end+1} = sprintf('U1max = 4*%s*%s^2;', uav.model.fmt(p.kt), uav.model.fmt(p.wMax));
L{end+1} = 'U1v = norm(Frot);';
L{end+1} = 'U1f = max(0, d''*Frot);';
L{end+1} = 'U1 = max(0, min(U1max, (1 - k)*U1v + k*U1f));';
L{end+1} = 'wRot = max(0, 1 - k);';
L{end+1} = 'Mxr = wRot*M(1); Myr = wRot*M(2); Mzr = wRot*M(3);';
L{end+1} = 'U2 = cb*Mxr + sb*Mzr;';
L{end+1} = 'U3 = Myr/max(cb, 0.20);';
L{end+1} = sprintf('U4 = (-sb*Mxr + cb*Mzr)/%s;', uav.model.fmt(p.kq));
L{end+1} = sprintf('T1 = 0.25*U1 - 0.25*U2/%s + 0.25*U3/%s + 0.25*U4;', ...
    uav.model.fmt(p.aY), uav.model.fmt(p.aX));
L{end+1} = sprintf('T2 = 0.25*U1 - 0.25*U2/%s - 0.25*U3/%s - 0.25*U4;', ...
    uav.model.fmt(p.aY), uav.model.fmt(p.aX));
L{end+1} = sprintf('T3 = 0.25*U1 + 0.25*U2/%s - 0.25*U3/%s + 0.25*U4;', ...
    uav.model.fmt(p.aY), uav.model.fmt(p.aX));
L{end+1} = sprintf('T4 = 0.25*U1 + 0.25*U2/%s + 0.25*U3/%s - 0.25*U4;', ...
    uav.model.fmt(p.aY), uav.model.fmt(p.aX));
L{end+1} = 'T = [T1; T2; T3; T4];';
L{end+1} = sprintf('om = sqrt(max(T, 0)/%s);', uav.model.fmt(p.kt));
L{end+1} = sprintf('om = min(om, %s);', uav.model.fmt(p.wMax));

L{end+1} = 'qs = max(qbar, 1.0);';
L{end+1} = sprintf('da = (1 - wRot)*M(1)/(qs*%s*%s*%s);', ...
    uav.model.fmt(p.S), uav.model.fmt(p.b), uav.model.fmt(p.Clda));
L{end+1} = sprintf('de = (1 - wRot)*M(2)/(qs*%s*%s*%s);', ...
    uav.model.fmt(p.S), uav.model.fmt(p.c), uav.model.fmt(p.Cmde));
L{end+1} = sprintf('dr = (1 - wRot)*M(3)/(qs*%s*%s*%s);', ...
    uav.model.fmt(p.S), uav.model.fmt(p.b), uav.model.fmt(p.Cndr));
L{end+1} = sprintf('actCmd = [min(max(de, -%s), %s); min(max(da, -%s), %s); min(max(dr, -%s), %s)];', ...
    uav.model.fmt(p.deMax), uav.model.fmt(p.deMax), ...
    uav.model.fmt(p.daMax), uav.model.fmt(p.daMax), ...
    uav.model.fmt(p.drMax), uav.model.fmt(p.drMax));
L{end+1} = sprintf('betaCmd = %s + (%s - %s)*k;', ...
    uav.model.fmt(p.betaMin), uav.model.fmt(p.betaMax), uav.model.fmt(p.betaMin));
L{end+1} = 'cmd = [om(:); actCmd(:); betaCmd];';
L{end+1} = 'end';

% 气动力：与 uav.plant.fw_aero 同一套系数式，油门固定为 0（倾转构型没有螺旋桨油门）
A = { ...
    'function Fa = aero(Vb, w, act)'
    '% 机体系气动力，act = [de; da; dr]，油门项为 0'};
A{end+1} = 'u = Vb(1); v = Vb(2); wb = Vb(3);';
A{end+1} = 'pp = w(1); qq = w(2); rr = w(3);';
A{end+1} = 'de = act(1); da = act(2); dr = act(3);';
A{end+1} = 'V = norm(Vb); Ve = max(V, 1.0);';
A{end+1} = 'alpha = atan2(wb, u);';
A{end+1} = 'bsa = asin(min(max(v/Ve, -1), 1));';
A{end+1} = sprintf('qbar = 0.5*%s*V^2;', uav.model.fmt(p.rho));
A{end+1} = sprintf('CL = %s + %s*alpha + %s*qq*%s/(2*Ve) + %s*de;', ...
    uav.model.fmt(p.CL0), uav.model.fmt(p.CLa), uav.model.fmt(p.CLq), ...
    uav.model.fmt(p.c), uav.model.fmt(p.CLde));
A{end+1} = sprintf('CD = %s + %s*CL^2;', uav.model.fmt(p.CD0), uav.model.fmt(p.kind));
A{end+1} = sprintf('CY = %s*bsa + %s*dr;', uav.model.fmt(p.CYb), uav.model.fmt(p.CYdr));
A{end+1} = sprintf('D = qbar*%s*CD; L = qbar*%s*CL; Y = qbar*%s*CY;', ...
    uav.model.fmt(p.S), uav.model.fmt(p.S), uav.model.fmt(p.S));
A{end+1} = 'Xa = -D*cos(alpha) + L*sin(alpha);';
A{end+1} = 'Za = -D*sin(alpha) - L*cos(alpha);';
A{end+1} = 'Fa = [Xa; Y; Za];';
A{end+1} = 'end';

code = strjoin([L; A; uav.model.dcm_lines(); uav.model.euler_lines()], newline);
end
