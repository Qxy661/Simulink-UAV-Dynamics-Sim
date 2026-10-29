function [om, sat] = tilt_mix(U1, U2, U3, U4, p)
%TILT_MIX  虚拟控制量 -> 四个旋翼转速。
%   解 [U1;U2;U3;U4] = M * [T1;T2;T3;T4]
%       M = [ 1    1    1    1 ;
%            -aY  -aY   aY   aY ;
%             aX  -aX  -aX   aX ;
%             1   -1    1   -1 ]
%   逆矩阵按显式算式展开，便于在 Simulink 图表里内联。
%   sat 为 1 表示有旋翼推力被截断到 [0, wMax]（或出现过负推力需求）。

T1 = 0.25*U1 - 0.25*U2/p.aY + 0.25*U3/p.aX + 0.25*U4;
T2 = 0.25*U1 - 0.25*U2/p.aY - 0.25*U3/p.aX - 0.25*U4;
T3 = 0.25*U1 + 0.25*U2/p.aY - 0.25*U3/p.aX + 0.25*U4;
T4 = 0.25*U1 + 0.25*U2/p.aY + 0.25*U3/p.aX - 0.25*U4;

T = [T1; T2; T3; T4];
sat = any(T < 0);

om  = sqrt(max(T, 0) / p.kt);
sat = sat || any(om > p.wMax);
om  = min(om, p.wMax);
end
