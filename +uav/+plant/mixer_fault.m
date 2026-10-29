function [wcmd, sat] = mixer_fault(TM, p, act)
%MIXER_FAULT  带电机失效/降效的转速分配。
%   [wcmd, sat] = uav.plant.mixer_fault(TM, p, act)
%   act 为 4x1 有效系数：1 正常，0 完全失效，0~1 之间表示推力与反扭矩按比例
%   下降。按有效系数缩放混控矩阵再求伪逆——失效电机的力矩缺口由其余电机
%   补，补不上时偏航力矩只能放弃。

act = act(:);
if all(act == 1)
    [wcmd, sat] = uav.plant.mixer(TM, p);
    return;
end

B = p.B .* act';                 % 每列按该电机有效系数缩放
w2 = pinv(B) * TM(:);
w2 = max(w2, 0);
wmax2 = p.wMax^2;
sat = any(w2 > wmax2);
w2 = min(w2, wmax2);

wcmd = sqrt(w2);
wcmd(act == 0) = 0;
end
