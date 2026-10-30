function e = ekf_init(p0, q0, opts)
%EKF_INIT 15 维误差状态 EKF 的初始结构。
%   状态顺序：位置(3)、速度(3)、姿态误差(3)、陀螺零偏(3)、加计零偏(3)。
%   姿态本体仍是四元数 e.q，协方差只对 3 维旋转误差建模，避免四元数归一化
%   约束带来的奇异协方差。
%
%   e = uav.est.ekf_init(p0, q0) 或 ekf_init(p0, q0, opts)
%   opts.sigmaP 初始位置标准差（默认 1 m），opts.sigmaQ 初始姿态（默认 0.1 rad）
%
%   见 docs/estimation.md。

if nargin < 3 || isempty(opts)
    opts = struct('sigmaP', 1.0, 'sigmaV', 0.5, 'sigmaQ', 0.1, ...
                  'sigmaBg', 0.01, 'sigmaBa', 0.1);
end

q0 = q0(:);
e.p = p0(:);
e.v = zeros(3, 1);
e.q = q0 / norm(q0);
e.bg = zeros(3, 1);
e.ba = zeros(3, 1);
e.P = diag([opts.sigmaP^2 * ones(1, 3), opts.sigmaV^2 * ones(1, 3), ...
            opts.sigmaQ^2 * ones(1, 3), opts.sigmaBg^2 * ones(1, 3), ...
            opts.sigmaBa^2 * ones(1, 3)]);
e.n = 0;
end
