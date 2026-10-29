function L = qmul_lines()
%QMUL_LINES  图表内联的局部函数 qmul（四元数乘法）。

L = { ...
    'function r = qmul(a, b)'
    'aw = a(1); av = a(2:4);'
    'bw = b(1); bv = b(2:4);'
    'r = [aw*bw - av''*bv; aw*bv + bw*av + cross(av, bv)];'
    'end'};
end
