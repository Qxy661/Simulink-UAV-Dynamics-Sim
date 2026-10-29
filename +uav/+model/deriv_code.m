function code = deriv_code(p)
%DERIV_CODE  生成 Simulink 图表用的刚体导数源码。

L = { ...
    'function xdot = deriv(x, FM)'
    '% 刚体六自由度：世界 NED，机体 FRD，状态 13 维'
    'Fb = FM(1:3); Mb = FM(4:6);'
    'Vb = x(4:6); q = x(7:10); w = x(11:13);'
    'R = dcm(q);'
    'qd = 0.5*qmul(q, [0;w]);'
    sprintf('Vbd = Fb/%s - cross(w, Vb) + R''*[0;0;%s];', ...
        uav.model.fmt(p.mass), uav.model.fmt(p.g))
    sprintf('I = [%s 0 0; 0 %s 0; 0 0 %s];', ...
        uav.model.fmt(p.I(1,1)), uav.model.fmt(p.I(2,2)), uav.model.fmt(p.I(3,3)))
    'wd = I \ (Mb - cross(w, I*w));'
    'xdot = [R*Vb; Vbd; qd; wd];'
    'end'};

code = strjoin([L; uav.model.dcm_lines(); uav.model.qmul_lines()], newline);
end
