function code = tilt_ref_code()
%TILT_REF_CODE  生成倾转四旋翼参考图表的源码。
%   与 uav.env.tilt_ref 同一套解析式，参数写成字面量。
%   输出 12 维：[p(3); v(3); a(3); psi; h; V]。

Vmax = 22.0;
Ta   = 15.0;
t1   = 5.0;
t2   = t1 + Ta;
h0   = 20.0;

L = { ...
    'function r = tiltref(t)'
    '% 悬停-转捩-巡航任务参考，返回 [p(3); v(3); a(3); psi; h; V]'};
L{end+1} = sprintf('Vmax = %s;', uav.model.fmt(Vmax));
L{end+1} = sprintf('Ta = %s;', uav.model.fmt(Ta));
L{end+1} = sprintf('t1 = %s;', uav.model.fmt(t1));
L{end+1} = sprintf('t2 = %s;', uav.model.fmt(t2));
L{end+1} = sprintf('h0 = %s;', uav.model.fmt(h0));
L{end+1} = 'pAcc = Vmax*Ta/2;';
L{end+1} = 'if t <= t1';
L{end+1} = '    V = 0; aN = 0; pN = 0;';
L{end+1} = 'elseif t <= t2';
L{end+1} = '    tau = t - t1;';
L{end+1} = '    V  = Vmax/2*(1 - cos(pi*tau/Ta));';
L{end+1} = '    aN = Vmax/2*(pi/Ta)*sin(pi*tau/Ta);';
L{end+1} = '    pN = Vmax/2*(tau - (Ta/pi)*sin(pi*tau/Ta));';
L{end+1} = 'else';
L{end+1} = '    V = Vmax; aN = 0; pN = pAcc + Vmax*(t - t2);';
L{end+1} = 'end';
L{end+1} = 'r = [pN; 0; -h0; V; 0; 0; aN; 0; 0; 0; h0; V];';
L{end+1} = 'end';

code = strjoin(L, newline);
end
