function L = euler_lines()
%EULER_LINES  图表内联的局部函数 euler（ZYX 欧拉角）。

L = { ...
    'function e = euler(q)'
    'n = norm(q);'
    'if n < 1e-12, e = [0;0;0]; return; end'
    'q = q/n;'
    'w = q(1); x = q(2); y = q(3); z = q(4);'
    's2 = 2*(w*y - z*x);'
    's2 = max(-1, min(1, s2));'
    'e = [atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));'
    '     asin(s2);'
    '     atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))];'
    'end'};
end
