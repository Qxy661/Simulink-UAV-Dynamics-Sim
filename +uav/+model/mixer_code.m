function code = mixer_code(p)
%MIXER_CODE  生成 Simulink 图表用的混控源码。

L = { ...
    'function wcmd = mix(TM)'
    '% 混控：力/力矩 -> 转速，负的转速平方截断为 0'
    sprintf('w2 = %s * TM;', mat2str(p.Binv, 17))
    'w2 = max(w2, 0);'
    'wcmd = sqrt(w2);'
    sprintf('wcmd = min(wcmd, %s);', uav.model.fmt(p.wMax))
    'end'};

code = strjoin(L, newline);
end
