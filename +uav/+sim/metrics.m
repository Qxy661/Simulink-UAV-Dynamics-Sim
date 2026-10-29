function m = metrics(out)
%METRICS  闭环仿真后段误差统计。
%   跟踪误差取最后 30% 时间窗，避免阶跃瞬态污染结论；
%   倾角与饱和统计整段，反映控制器的实际工作范围。

n = numel(out.t);
i0 = max(1, round(0.7*n));
e = out.ref(i0:end, :) - out.p(i0:end, :);
m.errMax = max(vecnorm(e, 2, 2));
m.errRms = sqrt(mean(sum(e.^2, 2)));
m.tiltMax = max(vecnorm(out.eul(:, 1:2), 2, 2));
m.satRatio = mean(out.sat);
end
