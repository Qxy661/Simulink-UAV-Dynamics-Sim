function code = wrench_code(p)
%WRENCH_CODE  生成 Simulink 图表用的力/力矩源码。

L = { ...
    'function [Fb, Mb] = wrench(w, x)'
    '% 旋翼推力与反扭矩，含机体轴线性阻力'
    'Vb = x(4:6);'
    sprintf('Tid = %s * w.^2;', uav.model.fmt(p.kt))
    sprintf('Fb = [0;0;-sum(Tid)] - %s*Vb;', uav.model.fmt(p.drag))
    sprintf('Mb = [-%s*(Tid(1) + Tid(2)) + %s*(Tid(3) + Tid(4));', ...
        uav.model.fmt(p.a), uav.model.fmt(p.a))
    sprintf('       %s*(Tid(1) - Tid(2) - Tid(3) + Tid(4));', uav.model.fmt(p.a))
    sprintf('       %s*(Tid(1) - Tid(2) + Tid(3) - Tid(4))];', uav.model.fmt(p.kq))
    'end'};

code = strjoin(L, newline);
end
