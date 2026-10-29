function validate_c_runner()
%VALIDATE_C_RUNNER  用 C 运行器复算三个构型，和脚本实现逐点比对。
%   scripts\validate_c_runner.m
%
%   先编译 c\ 下的运行器（mingw32-make 或 make），再运行本脚本。
%   每个构型：跑 C 可执行文件 → 读 CSV → 与对应 uav.sim.run* 的结果比较
%   → 写参考序列到 c\ref\ 供 CI 使用。
%
%   容差 1e-6：两边都是 IEEE double，差别只来自 libm 与 MATLAB 对
%   exp/sin/atan2 的末位不同，长时间积分后仍在 1e-12 量级。

root = fileparts(fileparts(mfilename('fullpath')));
cDir = fullfile(root, 'c');
outDir = fullfile(cDir, 'out');
refDir = fullfile(cDir, 'ref');
if ~isfolder(outDir)
    mkdir(outDir);
end
if ~isfolder(refDir)
    mkdir(refDir);
end

fprintf('== 四旋翼 位置阶跃 ==\n');
run_mission(cDir, outDir, refDir, 'quad', 'quad', 'quad_step_ref.csv', 14, 13);

fprintf('\n== 固定翼 爬升-转弯-加速 ==\n');
run_mission(cDir, outDir, refDir, 'fw', 'fw', 'fw_mission_ref.csv', 60, 13);

fprintf('\n== 倾转四旋翼 悬停-转捩-巡航 ==\n');
run_mission(cDir, outDir, refDir, 'tilt', 'tilt', 'tilt_mission_ref.csv', 45, 21);
end

function run_mission(cDir, outDir, refDir, tag, simName, refName, tEnd, nState)
exe = fullfile(cDir, 'build', ['uav_' tag '_runner.exe']);
if ~isfile(exe)
    exe = fullfile(cDir, 'build', ['uav_' tag '_runner']);
end
if ~isfile(exe)
    error('validate_c_runner:noExe', ...
        '找不到 %s，先在 c 目录执行 mingw32-make（或 make）', exe);
end

csv = fullfile(outDir, [tag '.csv']);
[st, msg] = system(sprintf('"%s" "%s"', exe, csv));
if st ~= 0
    error('validate_c_runner:run', '运行器失败：%s', msg);
end

T   = readmatrix(csv);
xC  = T(:, 2:end);
out = script_result(simName, tEnd);

if size(xC, 1) ~= size(out.x, 1) || size(xC, 2) ~= size(out.x, 2)
    error('validate_c_runner:size', '%s 尺寸不一致：C %dx%d，脚本 %dx%d', ...
        tag, size(xC, 1), size(xC, 2), size(out.x, 1), size(out.x, 2));
end

d = abs(xC - out.x);
[dm, idx] = max(d(:));
[r, c] = ind2sub(size(d), idx);
fprintf('  样本 %d，状态 %d 维\n', size(xC, 1), size(xC, 2));
fprintf('  最大偏差 %.3e（第 %d 样本，第 %d 维）\n', dm, r - 1, c);
fprintf('  末端 C    p = [%.6f %.6f %.6f]\n', xC(end, 1), xC(end, 2), xC(end, 3));
fprintf('  末端 脚本 p = [%.6f %.6f %.6f]\n', ...
    out.x(end, 1), out.x(end, 2), out.x(end, 3));

tol = 1e-6;
assert(dm < tol, 'validate_c_runner:tolerance', ...
    '%s 最大偏差 %.3e 超过容差 %.0e', tag, dm, tol);
fprintf('  %s 通过\n', tag);

refFile = fullfile(refDir, refName);
write_csv(refFile, out, nState);
fprintf('  参考序列 %s\n', refFile);
end

function out = script_result(simName, tEnd)
p = uav.cfg.quad();
c = uav.cfg.ctrl();
s = uav.cfg.simcfg();
s.tEnd = tEnd;

switch simName
    case 'quad'
        ref = @(t) uav.env.step(t, 1, [0; 0; -5], [3; 2; -5], 0.4);
        out = uav.sim.run(p, c, s, ref);
    case 'fw'
        ref = uav.env.fw_mission();
        p = uav.cfg.fw();
        c = uav.cfg.fwctrl();
        out = uav.sim.run_fw(p, c, s, ref);
    case 'tilt'
        ref = @(t) uav.env.tilt_ref(t);
        p = uav.cfg.tilt();
        c = uav.cfg.tiltctrl();
        out = uav.sim.run_tilt(p, c, s, ref);
    otherwise
        error('unknown mission %s', simName);
end
end

function write_csv(path, out, nState)
names = {'pn','pe','pd','u','v','w','qw','qx','qy','qz','p','q','r', ...
         'beta','om1','om2','om3','om4','de','da','dr'};
fid = fopen(path, 'w');
fprintf(fid, 't');
for k = 1:nState
    fprintf(fid, ',%s', names{k});
end
fprintf(fid, '\n');
for i = 1:size(out.x, 1)
    fprintf(fid, '%.6f', out.t(i));
    fprintf(fid, ',%.17g', out.x(i, :));
    fprintf(fid, '\n');
end
fclose(fid);
end
