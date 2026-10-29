function r = fw_ref(t, h0, V0)
%FW_REF  固定翼任务参考（纯函数，便于 codegen 与 Simulink 复用）。
%   t=5 s 爬升 30 m，t=20 s 右转 60 度，t=35 s 加速到 22 m/s。

h = h0;
if t >= 5
    h = h0 + 30;
end

psi = 0;
if t >= 20
    psi = 60*pi/180;
end

V = V0;
if t >= 35
    V = 22;
end

r = struct('h', h, 'psi', psi, 'V', V);
end
