function R = qmul(a, b)
%QMUL  Hamilton 四元数乘法 a (x) b。
%   与 [w;x;y;z] 排列一致，用于 qdot = 0.5*q (x) [0;omega]。

aw = a(1); av = a(2:4);
bw = b(1); bv = b(2:4);
R = [aw*bw - av'*bv;
     aw*bv + bw*av + cross(av, bv)];
end
