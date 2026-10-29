function sc = tilt_schedule(refV, p)
%TILT_SCHEDULE  空速调度：机翼权重 k 与短舱角 beta。
%   sc = uav.ctrl.tilt_schedule(refV, p)
%   refV : 参考空速（不是实测空速，理由见 uav.ctrl.tilt_cascade）
%
%   k    = clamp((refV - V1)/(V2 - V1), 0, 1)
%   beta = betaMin + (betaMax - betaMin)*k
%
%   V1 以下纯旋翼，V2 以上纯机翼。V2 是本机型的稳定上界，
%   再高会掉高失速，见 docs/tiltrotor.md 的包线表。

k = (refV - p.V1) / (p.V2 - p.V1);
k = max(0, min(1, k));

sc.k    = k;
sc.beta = p.betaMin + (p.betaMax - p.betaMin)*k;
end
