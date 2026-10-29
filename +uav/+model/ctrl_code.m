function code = ctrl_code(p, c)
%CTRL_CODE  生成 Simulink 图表用的级联控制器源码。
%   参数在生成时写成字面量，图表只依赖自身，避免 Simulink 尺寸推断失败。

L = { ...
    'function [T, M] = ctrl(x, refv)'
    '% 级联控制器：位置环 -> 推力方向 -> 姿态环'};
L{end+1} = 'persistent st';
L{end+1} = 'if isempty(st)';
L{end+1} = '    st.ip = [0;0;0];';
L{end+1} = '    st.ia = [0;0;0];';
L{end+1} = 'end';
L{end+1} = 'pn = x(1:3); Vb = x(4:6); q = x(7:10); w = x(11:13);';
L{end+1} = 'R = dcm(q);';
L{end+1} = 'Vn = R*Vb;';
L{end+1} = 'ep = refv(1:3) - pn;';
L{end+1} = 'ev = -Vn;';
L{end+1} = sprintf('a = %s.*ep + %s.*ev + %s.*st.ip;', ...
    uav.model.fmtv(c.Kp), uav.model.fmtv(c.Kd), uav.model.fmtv(c.Ki));
L{end+1} = 'aHor = a(1:2);';
L{end+1} = sprintf('aMax = %s*tan(%s);', uav.model.fmt(p.g), uav.model.fmt(c.maxTilt));
L{end+1} = 'if norm(aHor) > aMax';
L{end+1} = '    a(1:2) = aHor*(aMax/norm(aHor));';
L{end+1} = 'end';
L{end+1} = sprintf('st.ip = st.ip + ep*%s;', uav.model.fmt(1e-3));
L{end+1} = sprintf('st.ip = min(max(st.ip, -%s), %s);', ...
    uav.model.fmtv(c.iMax), uav.model.fmtv(c.iMax));
L{end+1} = sprintf('v = [0;0;%s] - a;', uav.model.fmt(p.g));
L{end+1} = 'nv = norm(v);';
L{end+1} = 'if nv < 1e-6';
L{end+1} = '    v = [0;0;1]; nv = 1;';
L{end+1} = 'end';
L{end+1} = 'z = v/nv;';
L{end+1} = sprintf('T = %s*nv;', uav.model.fmt(p.mass));
L{end+1} = sprintf('T = min(max(T, 0), %s);', uav.model.fmt(c.maxT));
L{end+1} = 'psi = refv(4);';
L{end+1} = 'xc = [cos(psi); sin(psi); 0];';
L{end+1} = 'yb = cross(z, xc);';
L{end+1} = 'ny = norm(yb);';
L{end+1} = 'if ny < 1e-6';
L{end+1} = '    xc = [1;0;0]; yb = cross(z, xc); ny = norm(yb);';
L{end+1} = 'end';
L{end+1} = 'yb = yb/ny;';
L{end+1} = 'xb = cross(yb, z);';
L{end+1} = 'Rd = [xb yb z];';
L{end+1} = 'phiD = atan2(Rd(3,2), Rd(3,3));';
L{end+1} = 'thetaD = -asin(max(-1, min(1, Rd(3,1))));';
L{end+1} = 'psiD = atan2(Rd(2,1), Rd(1,1));';
L{end+1} = 'att = euler(q);';
L{end+1} = 'e = [phiD;thetaD;psiD] - att;';
L{end+1} = 'e = mod(e + pi, 2*pi) - pi;';
L{end+1} = sprintf('st.ia = st.ia + e*%s;', uav.model.fmt(1e-3));
L{end+1} = sprintf('st.ia = min(max(st.ia, -%s), %s);', ...
    uav.model.fmtv(c.iMaxAtt), uav.model.fmtv(c.iMaxAtt));
L{end+1} = sprintf('M = %s.*e + %s.*st.ia - %s.*w;', ...
    uav.model.fmtv(c.KpAtt), uav.model.fmtv(c.KiAtt), uav.model.fmtv(c.KdAtt));
L{end+1} = sprintf('M = min(max(M, -%s), %s);', ...
    uav.model.fmtv(c.maxM), uav.model.fmtv(c.maxM));
L{end+1} = 'end';

code = strjoin([L; uav.model.dcm_lines(); uav.model.euler_lines()], newline);
end
