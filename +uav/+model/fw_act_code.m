function code = fw_act_code(p, tr)
%FW_ACT_CODE  生成舵机/油门一阶滞后的图表源码（精确离散，每步更新一次）。
%   tr 为配平结构体，用作初值。

L = { ...
    'function act = fwact(cmd)'
    '% 执行机构一阶滞后：每步更新一次，与 run_fw 的离散方式一致'};
L{end+1} = 'persistent a';
L{end+1} = 'if isempty(a)';
L{end+1} = sprintf('    a = [%s; %s; 0; 0];', ...
    uav.model.fmt(tr.thr), uav.model.fmt(tr.de));
L{end+1} = 'end';
L{end+1} = sprintf('tau = [%s; %s; %s; %s];', ...
    uav.model.fmt(p.tauT), uav.model.fmt(p.tauA), ...
    uav.model.fmt(p.tauA), uav.model.fmt(p.tauA));
L{end+1} = sprintf('k = exp(-%s ./ tau);', uav.model.fmt(1e-3));
L{end+1} = 'a = cmd + (a - cmd).*k;';
L{end+1} = 'act = a;';
L{end+1} = 'end';

code = strjoin(L, newline);
end
