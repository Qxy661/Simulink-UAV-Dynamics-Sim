function s = fmtv(v)
%FMTV  把列向量写成 MATLAB 代码里的 [a;b;c] 字面量。

parts = arrayfun(@(x) sprintf('%.17g', x), v(:), 'UniformOutput', false);
s = ['[' strjoin(parts, ';') ']'];
end
