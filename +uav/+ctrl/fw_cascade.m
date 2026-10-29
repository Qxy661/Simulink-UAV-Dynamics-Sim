function [act, st] = fw_cascade(x, ref, st, p, c, dt)
%FW_CASCADE  固定翼级联控制。
%   x   : 13 维状态 [pn; pe; pd; u; v; w; qw; qx; qy; qz; p; q; r]
%   ref : 参考结构体，字段 h（高度，m）、psi（航向，rad）、V（空速，m/s）
%   st  : 控制器状态，字段 iV（油门积分）
%   act : [throttle; de; da; dr]
%
%   俯仰指令以配平迎角为基准：theta_cmd = alpha_trim + 高度项。
%   航迹角 gamma = theta - alpha，所以爬升率近似为 V*sin(gamma)。

att = uav.plant.quat2euler(x(7:10));
phi = att(1); theta = att(2); psi = att(3);
Vb = x(4:6);
V  = max(norm(Vb), 1);
alpha = atan2(Vb(3), Vb(1));
p_ = x(11); q = x(12); r = x(13);

h    = -x(3);
hdot = V*sin(theta - alpha);
ePsi = uav.ctrl.wrap_pi(ref.psi - psi);

% 配平前馈
tr = uav.plant.fw_trim(p, ref.V);

% 速度回路
eV = ref.V - V;
st.iV = st.iV + c.KiV*eV*dt;
st.iV = max(-c.iVmax, min(c.iVmax, st.iV));
thr = tr.thr + c.KpV*eV + st.iV;
thr = max(0, min(1, thr));

% 高度回路
thCmd = tr.alpha + c.Kth_h*(ref.h - h) - c.Kth_hd*hdot;
thCmd = max(-c.thMax, min(c.thMax, thCmd));

% 俯仰回路：Cmde 为负，正舵（后缘下偏）产生低头力矩，故误差项取负号
qCmd = c.Kq_th*(thCmd - theta);
de = tr.de - c.Kde_q*(qCmd - q) - c.Kde_th*(thCmd - theta);

% 航向回路
phiCmd = c.Kphi_psi*ePsi;
phiCmd = max(-c.phiMax, min(c.phiMax, phiCmd));

% 滚转回路
pCmd = c.Kp_phi*(phiCmd - phi);
da = c.Kda_p*(pCmd - p_);

% 偏航阻尼 + 转弯协调
dr = -c.Kdr_r*r + c.Kdr_phi*phi;

act = [thr; ...
       max(-p.deMax, min(p.deMax, de)); ...
       max(-p.daMax, min(p.daMax, da)); ...
       max(-p.drMax, min(p.drMax, dr))];
end
