function [om, act, sat] = tilt_alloc(M, Dz, Lest, k, beta, qbar, p, c)
%TILT_ALLOC  力矩与升力需求 -> 旋翼转速与舵面偏角。
%   M     : 3x1 机体力矩需求 [Mx; My; Mz] (N*m)
%   Dz    : 需要的竖直支撑力 (N)，向上为正
%   Lest  : 当前机翼可提供的升力估计 (N)
%   k     : 机翼权重 0..1（由空速调度），旋翼权重为 1-k
%   beta  : 短舱倾角 (rad)
%   qbar  : 动压 (Pa)
%
%   分配两块：
%   1) 竖直力：旋翼补足 Dz 减去机翼承担的部分；beta 接近 90 度时 cos(beta) 很小，
%      用 0.2 做下限避免除零，此时旋翼竖直分量本来就已接近零。
%   2) 力矩：旋翼与舵面按 (1-k) / k 分担；旋翼那部分再用 tilt_mix 的逆映射解出 U2/U3/U4。
%
%   舵面残余力矩用线性气动导数反解，qbar 有下限保护（悬停时没有气动舵效）。

% ---- 竖直力 ----
% 机翼的升力按实际值全额扣掉，不乘调度权重 k：
% 早期写的是 max(Dz - k*Lest, 0)，本意是"机翼承担 k 份额"，但机翼升力是
% 物理事实、不是分配出来的份额。k=0.5、Lest=110 N、Dz=78.5 N 时，旧式
% 给旋翼留了 78.5-55=23.5 N，加上机翼 110 N，实际竖直力 133 N，多出 55 N
% 只能变成爬升——这正是减速段峰值 17~31 m 高度代价的来源（r7 图谱里
% 即使减速时长拉到 25 s 也压不下去）。改成全额扣减后旋翼只补缺口。
Drot = max(Dz - Lest, 0);
cb   = max(cos(beta), 0.2);
U1   = Drot / cb;

% ---- 力矩：旋翼份额 ----
wRot = max(0, 1 - k);
Mxr  = wRot * M(1);
Myr  = wRot * M(2);
Mzr  = wRot * M(3);

U2 = cb*Mxr + sin(beta)*Mzr;
U3 = Myr / cb;
U4 = (-sin(beta)*Mxr + cb*Mzr) / p.kq;

[om, sat] = uav.plant.tilt_mix(U1, U2, U3, U4, p);

% ---- 力矩：舵面份额 ----
qs = max(qbar, 1.0);
da = (1 - wRot) * M(1) / (qs*p.S*p.b*p.Clda);
de = (1 - wRot) * M(2) / (qs*p.S*p.c*p.Cmde);
dr = (1 - wRot) * M(3) / (qs*p.S*p.b*p.Cndr);

act = [ max(-p.deMax, min(p.deMax, de));
        max(-p.daMax, min(p.daMax, da));
        max(-p.drMax, min(p.drMax, dr)) ];
end
