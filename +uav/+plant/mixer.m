function [wcmd, sat] = mixer(TM, p)
%MIXER  力/力矩指令分配到四个旋翼。
%   输入 TM = [T; Mx; My; Mz]，输出目标转速 wcmd 与饱和标志。
%   先解 w^2，再对负值与超限做截断；截断后的真实力/力矩由 quad_wrench 重新计算。

w2 = p.Binv * TM(:);
sat = any(w2 < 0);
w2 = max(w2, 0);

wcmd = sqrt(w2);
over = wcmd > p.wMax;
if any(over)
    sat = true;
    wcmd(over) = p.wMax;
end
end
