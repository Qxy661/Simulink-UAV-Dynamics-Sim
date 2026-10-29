function s = fmt(x)
%FMT  把一个标量写成 MATLAB 代码字面量，保留全部有效数字。

s = sprintf('%.17g', x);
end
