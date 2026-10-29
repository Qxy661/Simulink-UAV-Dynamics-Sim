function codegen_c()
%CODEGEN_C  为三个构型重新生成 C 代码。
%   scripts\codegen_c.m
%   输出到 codegen\quad、codegen\fw、codegen\tilt，每个目录自包含。
%   c\ 下的运行器分别链接这三个目录，构成不依赖 MATLAB 的可执行文件。
%
%   只生成源码（GenCodeOnly），不需要本机 C 编译器；
%   编译由 c\Makefile 用 gcc 完成。

root = fileparts(fileparts(mfilename('fullpath')));

specs = { ...
    'quad', 'quad_sim', {zeros(3, 1), 0.4}, 1400, 13; ...
    'fw',   'fw_sim',   {},                  6000, 13; ...
    'tilt', 'tilt_sim', {},                  4500, 21};

for k = 1:size(specs, 1)
    name = specs{k, 1};
    fn   = specs{k, 2};
    args = specs{k, 3};
    out  = fullfile(root, 'codegen', name);
    src  = fullfile(root, '+uav', '+codegen', [fn '.m']);

    if ~isfile(src)
        error('codegen_c:missingSource', '找不到 %s', src);
    end
    if isfolder(out)
        rmdir(out, 's');
    end

    cfg = coder.config('lib');
    cfg.TargetLang  = 'C';
    cfg.GenCodeOnly = true;

    if isempty(args)
        codegen(src, '-config', cfg, '-d', out)
    else
        codegen(src, '-config', cfg, '-args', args, '-d', out)
    end

    % 中间产物不入库
    extra = {'buildInfo.mat', 'codedescriptor.dmr', 'codeInfo.mat', ...
             'compileInfo.mat', 'rtw_proj.tmw', '_clang-format', 'examples', ...
             'interface', 'html'};
    for e = 1:numel(extra)
        q = fullfile(out, extra{e});
        if isfile(q)
            delete(q);
        elseif isfolder(q)
            rmdir(q, 's');
        end
    end

    d = dir(fullfile(out, '*.c'));
    fprintf('  %-5s %2d 个 .c，%.1f KB，约 %d 样本 x %d 维\n', ...
        name, numel(d), sum([d.bytes])/1024, specs{k, 4}, specs{k, 5});
end

fprintf('生成完成：%s\n', fullfile(root, 'codegen'));
end
