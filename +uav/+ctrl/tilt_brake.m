function [cmd, st] = tilt_brake(x, ref, st, p, c, dt)
%TILT_BRAKE  减速（back-transition）模式控制器。
%   [cmd, st] = uav.ctrl.tilt_brake(x, ref, st, p, c, dt)
%
%   设计动机（见 research/REPORT_r3_decel.md）：
%   原有的姿态混合式级联在减速段会失稳，原因不是增益，而是三件事同时发生：
%   ① 短舱按参考空速收回，比机体真实速度提前 6–8 m/s；
%   ② 抬头角由速度误差直接放大，迎角一到 20° 以上升力翻倍；
%   ③ 多余的升力没有去处，只能变成爬升，爬升又让高度环反向拉平。
%
%   这条通道把减速当成一个独立模态处理，三件事分开：
%   - 短舱：按**实测空速**的走廊收回（Vlo 以下才开始收）；
%   - 俯仰：空速误差 → 俯仰角指令，带限幅（刹车靠气动阻力，不靠姿态硬拽）；
%   - 高度：高度 PI → 竖直支撑力，交给 tilt_alloc 分给机翼升力与旋翼。
%
%   输入 x 为 21 维状态，ref 字段 h/V/psi，st 为控制器状态（ip/ia/iH）。
%   输出 cmd = [om(4); de; da; dr; betaCmd] 与状态 st。

% ---- 气动状态 ----
Vb    = x(4:6);
V     = norm(Vb);
att   = uav.plant.quat2euler(x(7:10));
psi   = att(3);
alpha = atan2(Vb(3), max(Vb(1), 0.5));
h     = -x(3);
qbar  = 0.5*p.rho*V^2;

% ---- 1) 短舱：实测空速走廊 ----
Vlo = c.brakeVlo;          % 这里以下才开始收短舱
Vhi = c.brakeVhi;          % 这里以上保持巡航构型
k   = (V - Vlo) / (Vhi - Vlo);
k   = max(0, min(1, k));
betaCmd = p.betaMin + (p.betaMax - p.betaMin)*k;

% ---- 2) 俯仰：空速误差 + 高度误差 -> 俯仰角（抬头为正）----
tr   = uav.plant.fw_trim(p, max(V, 8));
eV   = V - ref.V;                        % 比参考快 -> 正值 -> 抬头
eH   = ref.h - h;
hdot = V*sin(att(2) - alpha);
% 高度误差只在低速段进俯仰：高速段短舱 90°、升力由机翼给，
% 这时再用高度误差拉俯仰，等于在 22 m/s 上做 30° 的拉杆，
% 结果就是 32°→−28°→−78° 的来回振荡（第二轮实测）。
wH = max(0, 1 - V/14);
thCmd = tr.alpha + c.brakeKV*eV + wH*(c.brakeKth_h*eH - c.brakeKth_hd*hdot);
thCmd = max(-c.brakeThMin, min(c.brakeThMax, thCmd));

% ---- 3) 俯仰内环 -> 升降舵 ----
% 舵效随动压变强：同一片舵在高空速下产生的力矩是低空速的几十倍，
% 所以增益按 qAttRef/qbar 缩一次，否则高速段一个 0.3 rad 的误差就打到舵限幅。
eTh = thCmd - att(2);
qCmd = c.brakeKq*eTh;
gb   = max(0, min(1, c.qAttRef/max(qbar, 1)));
deCmd = tr.de + gb*(c.brakeKde_q*(qCmd - x(12)) + c.brakeKde_th*eTh);
deCmd = max(-p.deMax, min(p.deMax, deCmd));

% ---- 4) 滚转/偏航保持 ----
phiCmd = 0;
pCmd   = c.brakeKp_phi*(phiCmd - att(1));
daCmd  = c.brakeKda_p*(pCmd - x(11));
daCmd  = max(-p.daMax, min(p.daMax, daCmd));

rCmd   = -c.brakeKdr_psi*uav.ctrl.wrap_pi(psi - ref.psi);
drCmd  = c.brakeKdr_r*(rCmd - x(13));
drCmd  = max(-p.drMax, min(p.drMax, drCmd));

% ---- 5) 竖直支撑力：旋翼只补机翼的缺口 ----
% β 接近 90° 时 cosβ→0，任何"竖直需求"都会被放大成巨大的前向推力，
% 这是第一轮发散的直接原因。所以这里只给旋翼一个受限的补足量：
% 需求不超过重力的 1.3 倍，且机翼升力为负（大迎角失速）时按 0 处理。
Dz = p.mass*p.g + c.brakeKh*eH + c.brakeKhd*(-hdot);
Dz = max(0, min(1.3*p.mass*p.g, Dz));

% ---- 6) 期望力矩：把舵面指令折成力矩，再交给分配器 ----
% 舵面力矩（线性气动导数，与 tilt_alloc 的反解互逆）
qs = max(qbar, 1.0);
Mde = qs*p.S*p.c*p.Cmde*deCmd;
Mda = qs*p.S*p.b*p.Clda*daCmd;
Mdr = qs*p.S*p.b*p.Cndr*drCmd;
M = [Mda; Mde; Mdr];

% 机翼升力估计：负值（失速/大负迎角）一律按 0，否则旋翼会被要求补上千牛
% 的"负升力"，进而饱和并把机体推着往前跑。
Lest = qbar*p.S*(p.CL0 + p.CLa*alpha);
Lest = max(0, min(1.2*p.mass*p.g, Lest));

% 旋翼只补机翼补不上的那部分竖直力，而且只在短舱还没完全前倾时补：
% β=90° 的旋翼是纯前向推进器，任何竖直需求都会变成 1/cosβ 倍的前向推力。
% 第一版直接调用 tilt_alloc 就栽在这里——β=90° 时 102 N 的竖直需求
% 变成 510 N 前向推力，机体 6 s 内从 22 m/s 冲到 38 m/s。
cb   = max(cos(betaCmd), 0.25);
Drot = (1 - k) * max(0, Dz - Lest);
U1   = Drot / cb;

% 力矩：旋翼 (1-k)、舵面 k，与 tilt_alloc 同一套映射
wRot = max(0, 1 - k);
Mxr = wRot*M(1);  Myr = wRot*M(2);  Mzr = wRot*M(3);
U2 = cos(betaCmd)*Mxr + sin(betaCmd)*Mzr;
U3 = Myr / cb;
U4 = (-sin(betaCmd)*Mxr + cos(betaCmd)*Mzr) / p.kq;
[om, sat] = uav.plant.tilt_mix(U1, U2, U3, U4, p);

act = [deCmd; daCmd; drCmd];

% 低速时舵面几乎无权限：把舵面清零，俯仰/滚转交给旋翼（wRot→1）
if V < 5
    act = [0; 0; 0];
end

cmd = [om; act; betaCmd];
st.sat = sat;
end
