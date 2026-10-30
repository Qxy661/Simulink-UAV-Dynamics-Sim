function S = skew(v)
%SKEW 3x3 反对称矩阵，满足 skew(a)*b = cross(a,b)。
S = [0, -v(3), v(2);
     v(3), 0, -v(1);
    -v(2), v(1), 0];
end
