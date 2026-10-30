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

% 姿态环：姿态误差 -> 机体力矩
c.KpAtt  = [6.0; 6.0; 3.0];
c.KdAtt  = [1.00; 1.00; 0.80];
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
end
