function code = fw_plant_code(p)
%FW_PLANT_CODE  生成固定翼 Simulink 图表用的刚体+气动导数源码。
%   纯函数：输入 x 与执行机构位置，输出 13 维导数，便于 RK4 多次调用。

L = { ...
    'function xd = fwplant(x, act)'
    '% 气动力/力矩 + 六自由度刚体方程'};
L{end+1} = 'Vb = x(4:6); u = Vb(1); v = Vb(2); wb = Vb(3);';
L{end+1} = 'q = x(7:10); w = x(11:13);';
L{end+1} = sprintf('m = %s;', uav.model.fmt(p.mass));
L{end+1} = sprintf('g = %s;', uav.model.fmt(p.g));
L{end+1} = sprintf('rho = %s; S = %s; b = %s; c = %s;', ...
    uav.model.fmt(p.rho), uav.model.fmt(p.S), uav.model.fmt(p.b), uav.model.fmt(p.c));
L{end+1} = sprintf('Tmax = %s;', uav.model.fmt(p.Tmax));
L{end+1} = 'thr = act(1); de = act(2); da = act(3); dr = act(4);';
L{end+1} = 'V = max(norm(Vb), 1);';
L{end+1} = 'alpha = asin(max(-0.34, min(0.34, wb/V)));';
L{end+1} = 'beta = asin(max(-0.34, min(0.34, v/V)));';
L{end+1} = 'qbar = 0.5*rho*V^2;';
L{end+1} = sprintf('CL = %s + %s*alpha + %s*w(2)*c/(2*V) + %s*de;', ...
    uav.model.fmt(p.CL0), uav.model.fmt(p.CLa), uav.model.fmt(p.CLq), uav.model.fmt(p.CLde));
L{end+1} = sprintf('CD = %s + %s*CL^2;', uav.model.fmt(p.CD0), uav.model.fmt(p.kind));
L{end+1} = sprintf('CY = %s*beta + %s*dr;', uav.model.fmt(p.CYb), uav.model.fmt(p.CYdr));
L{end+1} = 'Lft = qbar*S*CL; Dg = qbar*S*CD; Y = qbar*S*CY;';
L{end+1} = 'Xa = -Dg*cos(alpha) + Lft*sin(alpha);';
L{end+1} = 'Za = -Dg*sin(alpha) - Lft*cos(alpha);';
L{end+1} = 'Fb = [Xa + thr*Tmax; Y; Za];';
L{end+1} = sprintf('Cl = %s*beta + %s*w(1)*b/(2*V) + %s*w(3)*b/(2*V) + %s*da;', ...
    uav.model.fmt(p.Clb), uav.model.fmt(p.Clp), uav.model.fmt(p.Clr), uav.model.fmt(p.Clda));
L{end+1} = sprintf('Cm = %s + %s*alpha + %s*w(2)*c/(2*V) + %s*de;', ...
    uav.model.fmt(p.Cm0), uav.model.fmt(p.Cma), uav.model.fmt(p.Cmq), uav.model.fmt(p.Cmde));
L{end+1} = sprintf('Cn = %s*beta + %s*w(1)*b/(2*V) + %s*w(3)*b/(2*V) + %s*dr;', ...
    uav.model.fmt(p.Cnb), uav.model.fmt(p.Cnp), uav.model.fmt(p.Cnr), uav.model.fmt(p.Cndr));
L{end+1} = 'Mb = [qbar*S*b*Cl; qbar*S*c*Cm; qbar*S*b*Cn];';
L{end+1} = 'R = dcm(q);';
L{end+1} = 'Vbd = Fb/m - cross(w, Vb) + R''*[0;0;g];';
L{end+1} = 'qd = 0.5*qmul(q, [0; w]);';
L{end+1} = sprintf('I = [%s %s %s; %s %s %s; %s %s %s];', ...
    uav.model.fmt(p.I(1,1)), uav.model.fmt(p.I(1,2)), uav.model.fmt(p.I(1,3)), ...
    uav.model.fmt(p.I(2,1)), uav.model.fmt(p.I(2,2)), uav.model.fmt(p.I(2,3)), ...
    uav.model.fmt(p.I(3,1)), uav.model.fmt(p.I(3,2)), uav.model.fmt(p.I(3,3)));
L{end+1} = 'wd = I \ (Mb - cross(w, I*w));';
L{end+1} = 'xd = [R*Vb; Vbd; qd; wd];';
L{end+1} = 'end';

code = strjoin([L; uav.model.dcm_lines(); uav.model.qmul_lines()], newline);
end
