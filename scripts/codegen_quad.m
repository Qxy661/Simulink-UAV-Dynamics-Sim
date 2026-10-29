function codegen_quad()
%CODEGEN_QUAD  重新生成四旋翼闭环仿真的 C 代码。
%   scripts\codegen_quad.m
%   生成结果写入 codegen\：quad_sim.c/h 与运行时支撑文件。
%   c\ 下的 main.c 链接这些文件构成不依赖 MATLAB 的运行器。
%
%   只生成源码（GenCodeOnly），不依赖本机的 C 编译器；
%   编译由 c\Makefile 用 gcc 完成。

root = fileparts(fileparts(mfilename('fullpath')));
out  = fullfile(root, 'codegen');
src  = fullfile(root, '+uav', '+codegen', 'quad_sim.m');

if ~isfile(src)
    error('codegen_quad:missingSource', '找不到 %s', src);
end
if isfolder(out)
    rmdir(out, 's');
end

cfg = coder.config('lib');
cfg.TargetLang  = 'C';
cfg.GenCodeOnly = true;

codegen(src, '-config', cfg, '-args', {zeros(3, 1), 0.4}, '-d', out)

% 生成目录里的中间产物不需要入库
extra = {'buildInfo.mat', 'codedescriptor.dmr', 'codeInfo.mat', ...
         'compileInfo.mat', 'rtw_proj.tmw', '_clang-format', 'examples', ...
         'interface', 'html'};
for k = 1:numel(extra)
    p = fullfile(out, extra{k});
    if isfile(p)
        delete(p);
    elseif isfolder(p)
        rmdir(p, 's');
    end
end

fprintf('生成完成：%s\n', out);
d = dir(fullfile(out, '*.c'));
fprintf('  %d 个 .c 文件，共 %.1f KB\n', numel(d), sum([d.bytes])/1024);
end
