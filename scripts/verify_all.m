function verify_all()
%VERIFY_ALL 一条命令跑完仓库里的全部检查。
%   依次执行：49 项单元与闭环测试、三个 Simulink 模型重建与对拍、C 运行器对拍。
%   任一项失败时在末尾以 error 结束，退出码非零，可直接给 CI 或人工复核用。
%
%   用法：
%       cd scripts
%       verify_all
%
%   见 docs/validation.md。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);

n = 0; ok = false(1, 8); note = cell(1, 8); name = cell(1, 8);

% ---- 1. 单元与闭环测试 ----
n = n + 1; name{n} = '单元与闭环测试';
fprintf('\n== %d. %s\n', n, name{n});
t0 = tic;
try
    suite = runtests(fullfile(root, 'tests'));
    passed = sum([suite.Passed]);
    total = numel(suite);
    ok(n) = (passed == total);
    note{n} = sprintf('%d/%d 通过，%.1f s', passed, total, toc(t0));
catch err
    ok(n) = false;
    note{n} = err.message;
end
fprintf('   %s\n', note{n});

% ---- 2. Simulink 模型重建与对拍 ----
cfgs = { 'quad', 'build_quad_model', 'validate_model';
         'fw',   'build_fw_model',   'validate_fw_model';
         'tilt', 'build_tilt_model', 'validate_tilt_model' };

for i = 1:size(cfgs, 1)
    n = n + 1;
    name{n} = sprintf('Simulink 对拍（%s）', cfgs{i, 1});
    fprintf('\n== %d. %s\n', n, name{n});
    t0 = tic;
    try
        run(fullfile(root, 'scripts', [cfgs{i, 2} '.m']));
        run(fullfile(root, 'scripts', [cfgs{i, 3} '.m']));
        ok(n) = true;
        note{n} = sprintf('通过，%.1f s', toc(t0));
    catch err
        ok(n) = false;
        note{n} = err.message;
    end
    fprintf('   %s\n', note{n});
end

% ---- 3. C 运行器对拍 ----
n = n + 1; name{n} = 'C 运行器对拍';
fprintf('\n== %d. %s\n', n, name{n});
exe = fullfile(root, 'c', 'build', 'uav_quad_runner');
if ispc
    exe = [exe '.exe'];
end
if ~isfile(exe)
    ok(n) = false;
    note{n} = '未找到 c/build/uav_quad_runner，先在 c/ 下执行 make';
else
    t0 = tic;
    try
        run(fullfile(root, 'scripts', 'validate_c_runner.m'));
        ok(n) = true;
        note{n} = sprintf('通过，%.1f s', toc(t0));
    catch err
        ok(n) = false;
        note{n} = err.message;
    end
end
fprintf('   %s\n', note{n});

% ---- 汇总 ----
fprintf('\n== 汇总\n');
for i = 1:n
    fprintf('   %-24s %-6s %s\n', name{i}, ternary(ok(i)), note{i});
end
bdclose('all');

if ~all(ok(1:n))
    error('uav:verifyFailed', '%d 项检查未通过', sum(~ok(1:n)));
end
fprintf('\n全部通过。\n');
end

function s = ternary(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end
