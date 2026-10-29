function [Fb, Mb] = quad_wrench(w, Vb, p)
%QUAD_WRENCH  旋翼转速到机体力/力矩。
%   旋翼推力沿机体 -z，阻力取线性近似（仅低速段有效）。
%   w 为 4x1 转速 rad/s。

T = p.kt * w.^2;
a = p.a;

Fb = [0; 0; -sum(T)] - p.drag*Vb;
Mb = [ -a*(T(1) + T(2)) + a*(T(3) + T(4));
        a*(T(1) - T(2) - T(3) + T(4));
        p.kq*(T(1) - T(2) + T(3) - T(4)) ];
end
