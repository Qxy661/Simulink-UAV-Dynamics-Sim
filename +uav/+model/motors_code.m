function code = motors_code(p)
%MOTORS_CODE  生成 Simulink 图表用的电机源码（一阶惯性，解析离散）。

L = { ...
    'function w = motors(wcmd)'
    '% 一阶电机模型，按 1 ms 步长解析离散'
    'persistent w0'
    sprintf('if isempty(w0), w0 = %s*ones(4,1); end', uav.model.fmt(p.hoverW))
    sprintf('alpha = exp(-%s/%s);', uav.model.fmt(1e-3), uav.model.fmt(p.tauM))
    'w0 = wcmd + (w0 - wcmd)*alpha;'
    sprintf('w0 = min(max(w0, %s), %s);', uav.model.fmt(p.wMin), uav.model.fmt(p.wMax))
    'w = w0;'
    'end'};

code = strjoin(L, newline);
end
