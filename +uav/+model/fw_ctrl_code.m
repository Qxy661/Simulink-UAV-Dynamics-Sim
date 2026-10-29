function code = fw_ctrl_code(p, c)
%FW_CTRL_CODE  生成固定翼 Simulink 图表用的控制器源码。
%   参数写成字面量，配平前馈用局部函数 trimV(V) 现场求解，与 fw_trim 同算法。

L = { ...
    'function cmd = fwctrl(x, rv)'
    '% 固定翼级联控制：高度-俯仰-升降舵、航向-滚转-副翼、速度-油门'};
L{end+1} = 'persistent iV';
L{end+1} = 'if isempty(iV)';
L{end+1} = '    iV = 0;';
L{end+1} = 'end';
L{end+1} = 'att = euler(x(7:10));';
L{end+1} = 'phi = att(1); theta = att(2); psi = att(3);';
L{end+1} = 'Vb = x(4:6);';
L{end+1} = sprintf('V = max(norm(Vb), %s);', uav.model.fmt(1));
L{end+1} = 'alpha = atan2(Vb(3), Vb(1));';
L{end+1} = 'p_ = x(11); q = x(12); r = x(13);';
L{end+1} = 'h = -x(3);';
L{end+1} = 'hdot = V*sin(theta - alpha);';
L{end+1} = 'ePsi = mod(rv(2) - psi + pi, 2*pi) - pi;';
L{end+1} = 'tr = trimV(rv(3));';
L{end+1} = 'aT = tr(1); deT = tr(2); thrT = tr(3);';
L{end+1} = 'eV = rv(3) - V;';
L{end+1} = sprintf('iV = iV + %s*eV*%s;', uav.model.fmt(c.KiV), uav.model.fmt(1e-3));
L{end+1} = sprintf('iV = min(max(iV, -%s), %s);', uav.model.fmt(c.iVmax), uav.model.fmt(c.iVmax));
L{end+1} = sprintf('thr = thrT + %s*eV + iV;', uav.model.fmt(c.KpV));
L{end+1} = 'thr = min(max(thr, 0), 1);';
L{end+1} = sprintf('thCmd = aT + %s*(rv(1) - h) - %s*hdot;', ...
    uav.model.fmt(c.Kth_h), uav.model.fmt(c.Kth_hd));
L{end+1} = sprintf('thCmd = min(max(thCmd, -%s), %s);', ...
    uav.model.fmt(c.thMax), uav.model.fmt(c.thMax));
L{end+1} = sprintf('qCmd = %s*(thCmd - theta);', uav.model.fmt(c.Kq_th));
L{end+1} = sprintf('de = deT - %s*(qCmd - q) - %s*(thCmd - theta);', ...
    uav.model.fmt(c.Kde_q), uav.model.fmt(c.Kde_th));
L{end+1} = sprintf('phiCmd = %s*ePsi;', uav.model.fmt(c.Kphi_psi));
L{end+1} = sprintf('phiCmd = min(max(phiCmd, -%s), %s);', ...
    uav.model.fmt(c.phiMax), uav.model.fmt(c.phiMax));
L{end+1} = sprintf('pCmd = %s*(phiCmd - phi);', uav.model.fmt(c.Kp_phi));
L{end+1} = sprintf('da = %s*(pCmd - p_);', uav.model.fmt(c.Kda_p));
L{end+1} = sprintf('dr = -%s*r + %s*phi;', uav.model.fmt(c.Kdr_r), uav.model.fmt(c.Kdr_phi));
L{end+1} = sprintf('cmd = [thr; min(max(de, -%s), %s); min(max(da, -%s), %s); min(max(dr, -%s), %s)];', ...
    uav.model.fmt(p.deMax), uav.model.fmt(p.deMax), ...
    uav.model.fmt(p.daMax), uav.model.fmt(p.daMax), ...
    uav.model.fmt(p.drMax), uav.model.fmt(p.drMax));
L{end+1} = 'end';

% 配平：与 uav.plant.fw_trim 同一套不动点迭代
T = { ...
    'function tr = trimV(V)'
    '% 定直平飞配平，返回 [alpha; de; thr]'};
T{end+1} = sprintf('qbar = 0.5*%s*V^2;', uav.model.fmt(p.rho));
T{end+1} = sprintf('W = %s*%s;', uav.model.fmt(p.mass), uav.model.fmt(p.g));
T{end+1} = sprintf('CL = W/(qbar*%s);', uav.model.fmt(p.S));
T{end+1} = 'for it = 1:6';
T{end+1} = sprintf('    CD = %s + %s*CL^2;', uav.model.fmt(p.CD0), uav.model.fmt(p.kind));
T{end+1} = sprintf('    D = qbar*%s*CD;', uav.model.fmt(p.S));
T{end+1} = sprintf('    A = [%s %s; %s %s];', ...
    uav.model.fmt(p.CLa), uav.model.fmt(p.CLde), uav.model.fmt(p.Cma), uav.model.fmt(p.Cmde));
T{end+1} = sprintf('    x = A \\ [CL - %s; -%s];', uav.model.fmt(p.CL0), uav.model.fmt(p.Cm0));
T{end+1} = '    a = x(1);';
T{end+1} = sprintf('    CL = (W*cos(a) - D*sin(a))/(qbar*%s*cos(a));', uav.model.fmt(p.S));
T{end+1} = 'end';
T{end+1} = sprintf('CD = %s + %s*CL^2;', uav.model.fmt(p.CD0), uav.model.fmt(p.kind));
T{end+1} = sprintf('D = qbar*%s*CD;', uav.model.fmt(p.S));
T{end+1} = sprintf('L = qbar*%s*CL;', uav.model.fmt(p.S));
T{end+1} = sprintf('tr = [a; x(2); (D*cos(a) - L*sin(a) + W*sin(a))/%s];', uav.model.fmt(p.Tmax));
T{end+1} = 'end';

code = strjoin([L; T; uav.model.euler_lines()], newline);
end
