function c = fwctrl()
%FWCTRL  固定翼控制器参数。
%   高度-俯仰-升降舵、航向-滚转-副翼、速度-油门三条回路，外加偏航阻尼。
%   在 18 m/s、100 m 平飞点整定；爬升 30 m 与 60 度转弯在 20 s 内收敛。

% 高度回路：高度误差 -> 俯仰角指令
c.Kth_h   = 0.030;      % rad/m
c.Kth_hd  = 0.040;      % rad/(m/s)，抑制爬升率
c.thMax   = 0.35;       % rad，俯仰指令限幅（20 deg）

% 俯仰回路：俯仰误差 -> 俯仰角速度指令 -> 升降舵
% Cmde < 0（后缘下偏产生低头力矩），故误差项在控制器里取负号
c.Kq_th   = 2.0;        % 1/s
c.Kde_q   = 0.60;       % rad/(rad/s)
c.Kde_th  = 0.50;       % rad/rad

% 速度回路：空速误差 -> 油门
c.KpV     = 0.150;      % 1/(m/s)
c.KiV     = 0.020;
c.iVmax   = 0.30;

% 航向回路：航向误差 -> 滚转角指令
c.Kphi_psi = 0.80;      % rad/rad
c.phiMax   = 0.52;      % rad（30 deg）

% 滚转回路：滚转误差 -> 滚转角速度指令 -> 副翼
c.Kp_phi  = 3.0;        % 1/s
c.Kda_p   = 0.40;       % rad/(rad/s)

% 偏航阻尼与转弯协调
c.Kdr_r   = 0.50;       % rad/(rad/s)
c.Kdr_phi = 0.10;       % rad/rad
end
