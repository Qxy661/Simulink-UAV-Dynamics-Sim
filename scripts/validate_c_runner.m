function validate_c_runner()
%VALIDATE_C_RUNNER  用 C 运行器复算四旋翼阶跃，和脚本实现逐点比对。
%   scripts\validate_c_runner.m
%
%   步骤：编译（若需要）→ 运行 c\build\uav_quad_runner → 读 CSV
%   → 与 uav.sim.run 的同任务结果比较。容差 1e-6：两边都用 IEEE double，
%   差别只来自 libm 与 MATLAB 对 exp/sin/atan2 的末位不同，14 s 积分放大后
%   仍在 1e-10 量级；定 1e-6 留出跨平台余量。

root = fileparts(fileparts(mfilename('fullpath')));
cDir = fullfile(root, 'c');
exe  = fullfile(cDir, 'build', 'uav_quad_runner.exe');
csv  = fullfile(cDir, 'out', 'quad_step.csv');

if ~isfile(exe)
    error('validate_c_runner:noExe', ...
        '找不到 %s，先在 c 目录执行 mingw32-make（或 make）', exe);
end
if ~isfolder(fullfile(cDir, 'out'))
    mkdir(fullfile(cDir, 'out'));
end

[st, msg] = system(sprintf('"%s"', exe));
if st ~= 0
    error('validate_c_runner:run', 'C 运行器失败：%s\n%s', msg, exe);
end

T = readmatrix(csv);
tC = T(:, 1);
xC = T(:, 2:14);

% 同一任务的脚本侧结果
p = uav.cfg.quad();
c = uav.cfg.ctrl();
s = uav.cfg.simcfg();
s.tEnd = 14;
ref = @(t) uav.env.step(t, 1, [0; 0; -5], [3; 2; -5], 0.4);
out = uav.sim.run(p, c, s, ref);

if size(xC, 1) ~= size(out.x, 1)
    error('validate_c_runner:size', ...
        '样本数不一致：C %d，脚本 %d', size(xC, 1), size(out.x, 1));
end

d = abs(xC - out.x);
fprintf('样本 %d 个，%d 维状态\n', size(xC, 1), size(xC, 2));
names = {'pn','pe','pd','u','v','w','qw','qx','qy','qz','p','q','r'};
for k = 1:13
    fprintf('  %-3s 最大偏差 %.3e\n', names{k}, max(d(:, k)));
end
fprintf('状态最大偏差 %.3e\n', max(d(:)));
fprintf('末端 C     p = [%.6f %.6f %.6f]\n', xC(end, 1), xC(end, 2), xC(end, 3));
fprintf('末端 脚本  p = [%.6f %.6f %.6f]\n', out.x(end, 1), out.x(end, 2), out.x(end, 3));

tol = 1e-6;
assert(max(d(:)) < tol, 'validate_c_runner:tolerance', ...
    '最大偏差 %.3e 超过容差 %.0e', max(d(:)), tol);

% 参考 CSV 供 CI 比对（不含 MATLAB 的机器也能跑）
refDir = fullfile(cDir, 'ref');
if ~isfolder(refDir)
    mkdir(refDir);
end
refFile = fullfile(refDir, 'quad_step_ref.csv');
fid = fopen(refFile, 'w');
fprintf(fid, 't,pn,pe,pd,u,v,w,qw,qx,qy,qz,p,q,r\n');
fprintf(fid, '%.6f,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n', ...
    [out.t, out.x]');
fclose(fid);
fprintf('参考序列已写入 %s\n', refFile);
end
