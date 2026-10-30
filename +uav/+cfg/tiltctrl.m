function c = tiltctrl()
%TILTCTRL  倾转四旋翼控制器参数。
%   位置环是统一的一层；姿态指令由"推力方向反解"（悬停）与"高度-俯仰配平"（巡航）
%   按空速混合；力矩按旋翼权 (1-k) 与舵面权 k 分配。

% 位置环：位置/速度误差 -> 期望加速度（NED）
c.Kp     = [1.0; 1.0; 1.6];     % 1/s^2 per m
c.Kd     = [1.8; 1.8; 2.2];     % 1/s per m/s
c.Ki     = [0.0; 0.0; 0.8];
c.iMax   = [0.0; 0.0; 1.2];     % m/s^2
c.aHoriz = 2.5;                % m/s^2，水平加速度限幅（约 30 deg 倾角）
c.aVert  = 4.0;                 % m/s^2，垂直加速度限幅
c.aDecMax = 1.5;                % m/s^2，转捩段后向加速度上限（平飞阻力 / 质量）

% 姿态环：姿态误差 -> 机体力矩
% 这里的 KdAtt 与四旋翼那套不同：[1;1;0.8] 对 I=[0.9;1.2;1.6] 只有 0.3 的阻尼比，
% 一旦偏离悬停平衡点就进极限环（r17 实测：1 m 位置偏差振荡到 0.83 m、俯仰 18°，
% 且 Kd 越大越差——位置环的高频指令叠在欠阻尼姿态环上）。[4;4;2] 把阻尼比拉到
% 约 0.9，悬停 2 m 初始偏差收敛到 0.38 m、2 m/s 侧风只漂 0.32 m。
c.KpAtt  = [6.0; 6.0; 3.0];
c.KdAtt  = [4.00; 4.00; 2.00];
c.KiAtt  = [0.0; 0.0; 0.60];
c.iMaxAtt= [0.0; 0.0; 0.35];    % N*m
c.maxM   = [2.5; 2.5; 1.0];     % N*m
c.qAttRef= 20.0;                % Pa，舵面支的力矩增益随动压放大到这个值

% 巡航段的姿态参考：高度误差 -> 俯仰角
c.Kth_h  = 0.030;               % rad/m
c.Kth_hi = 0.006;               % rad/(m*s)，高度积分：β=90° 时竖直推力≈0，
                                % 比例项拉不掉残余爬升，只能靠积分把俯仰配平点挪过去
c.iHmax  = 40.0;                % m*s
c.Kth_hd = 0.040;               % rad/(m/s)
c.KthV   = 0.030;               % rad/(m/s)，巡航段速度误差 -> 俯仰
c.thMax  = 0.30;                % rad
c.Kphi_psi = 0.80;              % rad/rad
c.phiMax   = 0.45;              % rad
c.alphaMin = -0.20;             % rad
c.alphaMax =  0.30;             % rad

% ---- 减速（back-transition）模态：uav.ctrl.tilt_brake 用 ----
c.brakeVlo = 4.0;               % m/s，实测空速低于此才开始收短舱
c.brakeVhi = 13.0;              % m/s，高于此保持巡航构型（β=90°）
c.brakeKV  = 0.025;             % rad/(m/s)，空速误差 -> 俯仰角
c.brakeKth_h  = 0.020;          % rad/m，高度误差 -> 俯仰角（β≈90° 时高度只能靠航迹角）
c.brakeKth_hd = 0.030;          % rad/(m/s)，下沉率阻尼
c.brakeThMin = 0.15;            % rad，允许低头 −8.6°
c.brakeThMax = 0.35;            % rad，最多抬头 20°
c.brakeKq    = 1.2;             % 1/s，俯仰角误差 -> 俯仰角速度
c.brakeKde_q = 0.30;            % rad per rad/s
c.brakeKde_th= 0.60;            % rad per rad
c.brakeKp_phi= 2.0;             % 1/s
c.brakeKda_p = 0.35;            % rad per rad/s
c.brakeKdr_psi = 0.80;          % rad/rad
c.brakeKdr_r = 0.45;            % rad per rad/s
c.brakeKh    = 0.20;            % m/s^2 per m
c.brakeKhi   = 0.06;            % m/s^2 per (m*s)
c.brakeKhd   = 0.35;            % m/s^2 per m/s
end
