function [om, sat, Tmix] = tilt_mix_fault(U1, U2, U3, U4, p, act, method)
%TILT_MIX_FAULT  考虑单电机降效的虚拟控制量分配。
%   [om, sat, Tmix] = uav.plant.tilt_mix_fault(U1, U2, U3, U4, p, act, method)
%
%   正常分配解 B*T = [U1;U2;U3;U4]（B 见 uav.plant.tilt_mix）。电机 i 降效
%   因子 act(i)∈[0,1] 时，实际产生的推力是 act(i)*T(i)，于是约束变成
%
%       B * diag(act) * T = U
%
%   4 个未知、4 个方程，但秩只有 3（一台失效时）——偏航方向的分量不再可控。
%
%   method:
%     'pinv'（默认）——最小范数解，滚转/俯仰/总推力尽量满足，偏航残差留给
%        机体自转。悬停时剩下的三台可能被推到饱和，姿态随后发散（r10 实测）。
%     'rpi'——带饱和的处理：先解最小范数，把超限的那一台钉在边界，再对剩下的
%        三台重解，重复到没有新的越界。代价是最小范数性，收益是总推力与
%        滚转/俯仰优先满足，悬停断桨不再必然发散。
%
%   输入 act 为 4x1，全 1 时退化成普通逆混控。

if nargin < 6 || isempty(act), act = ones(4, 1); end
if nargin < 7 || isempty(method), method = 'pinv'; end
act = max(0, min(1, act(:)));

if all(act > 0.999)
    [om, sat] = uav.plant.tilt_mix(U1, U2, U3, U4, p);
    if nargout > 2
        Tmix = p.kt*om.^2;
    end
    return
end

% 有效混控矩阵：把降效因子折进 B 的列
B = [ 1        1        1        1      ;
     -p.aY    -p.aY     p.aY     p.aY   ;
      p.aX    -p.aX    -p.aX     p.aX   ;
      1       -1        1       -1     ];
Bd = B .* act';            % 第 i 列乘 act(i)

U = [U1; U2; U3; U4];
Tmax = p.kt*p.wMax^2;

if strcmp(method, 'rpi')
    [T, sat] = alloc_rpi(Bd, U, Tmax, act, false);
elseif strcmp(method, 'rank')
    % 秩感知最小二乘：秩不足时先判断哪一维不可控（这里是偏航），把它从
    % 目标里剔掉，只对可满足的三维做精确求解。否则最小二乘会把总推力
    % 一起折中——悬停断桨时 U1 被压到 70 N（应为 78.5 N），机体会持续下沉。
    Tmax = p.kt*p.wMax^2;
    W = diag([1; 1; 1; double(rank(Bd) >= 4)]);
    T = lsqminnorm(W*Bd, W*U, 1e-10);
    T = max(T, 0);
    sat = any(T > Tmax);
    T = min(T, Tmax);
elseif strcmp(method, 'rankrpi')
    % 秩感知 + 饱和重分配：先剔掉不可控的偏航，再把越界电机钉在边界后
    % 对剩余电机重解。悬停断桨（r10）里这一档是唯一能稳住高度的组合。
    Tmax = p.kt*p.wMax^2;
    [T, sat] = alloc_rpi(Bd, U, Tmax, act, rank(Bd) < 4);
else
    T = pinv(Bd) * U;
    T = max(T, 0);
    sat = any(T > Tmax);
    T = min(T, Tmax);
end

om  = sqrt(T / p.kt);
Tmix = T;
end

function [T, sat] = alloc_rpi(Bd, U, Tmax, act, dropYaw)
% 带饱和的重分配：越界的电机钉在边界，对剩下的电机重解，直到没有新越界。
if nargin < 5 || isempty(dropYaw), dropYaw = false; end
T = zeros(4, 1);
free = act > 1e-9;          % 完全失效的电机不参与
sat = false;
T = max(T, 0);

if dropYaw
    W = diag([1; 1; 1; 0]);     % 偏航不可控，从目标里剔掉
else
    W = eye(4);
end

for it = 1:4
    idx = find(free);
    if isempty(idx), break; end
    rem = U - Bd(:, ~free) * T(~free);
    Tf = lsqminnorm(W*Bd(:, idx), W*rem, 1e-10);

    hi = Tf >= Tmax;
    lo = Tf <= 0;
    if any(hi) || any(lo)
        sat = sat || any(hi);
        hit = idx(hi);  T(hit) = Tmax;  free(hit) = false;
        hit = idx(lo);  T(hit) = 0;     free(hit) = false;
        continue
    end
    T(idx) = Tf;
    break
end
T = min(max(T, 0), Tmax);
end
