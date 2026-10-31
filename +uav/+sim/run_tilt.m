function out = run_tilt(p, c, s, ref, x0)
%RUN_TILT  倾转四旋翼闭环仿真。
%   out = uav.sim.run_tilt(p, c, s, ref, x0)
%   ref 为结构体或 @(t) 句柄；省略 x0 时从 ref(0) 的悬停状态起飞。
%
%   状态 x（21 维）：
%     1:13  刚体 [pn;pe;pd; u;v;w; qw;qx;qy;qz; q;p;r]
%     14    短舱倾角 beta
%     15:18 旋翼转速
%     19:21 舵面 [de; da; dr]
%   控制器按 ctrlDiv 分频，短舱/电机/舵面在 plant 内部做一阶滞后。

if nargin < 4
    error('run_tilt:ref', '需要参考信号');
end

r0 = eval_ref(ref, 0);
if nargin < 5 || isempty(x0)
    x0 = [r0.p(:); 0; 0; 0; 1; 0; 0; 0; 0; 0; 0; ...
          p.betaMin; p.hoverW*ones(4,1); 0; 0; 0];
end

dt   = s.dt;
N    = round(s.tEnd/dt);
nLog = floor(N/s.logDiv) + 1;

tLog = zeros(nLog, 1);
xLog = zeros(nLog, 21);
pLog = zeros(nLog, 3);
vLog = zeros(nLog, 3);
eLog = zeros(nLog, 3);
wLog = zeros(nLog, 3);
cLog = zeros(nLog, 8);
 fLog = zeros(nLog, 3);   % 机体比力（用于 EKF 开环验证）
rLog = zeros(nLog, 6);

x   = x0(:);
% 检测事件（r21）：无论是否开 logFault 都记下来，用来统计阵风下的误报。
tDetLog = [];
iDetLog = [];
% 姿态量测延迟（r20）：控制器看到的四元数滞后 attDelay 个控制拍。
% 估计器的相位滞后比精度更致命（四旋翼 r13 的极限环就是这么来的），
% 这里用纯延迟扫出"能忍多少毫秒"。
attDelay = 0;
if isfield(s, 'pert') && isfield(s.pert, 'attDelayTicks')
    attDelay = round(s.pert.attDelayTicks);
end
if attDelay > 0
    qHist = repmat(x(7:10), 1, attDelay + 1);
end
cmd = [p.hoverW*ones(4,1); 0; 0; 0; p.betaMin];
st  = struct('ip', zeros(3,1), 'ia', zeros(3,1), 'iH', 0);
det = struct('cnt', zeros(4,1), 'dead', false(4,1), 'th', 200, 'hold', 30);
if isfield(s, 'detTh'),   det.th   = s.detTh;   end
if isfield(s, 'detHold'), det.hold = s.detHold; end

% 扰动观测（r18，opt-in）：用位置量测驱动三轴 ESO，把估计出的扰动加速度
% 从期望加速度里扣掉。悬停抗风靠它，而不是靠位置误差硬顶。
useEso = isfield(s, 'useEso') && s.useEso;
if useEso
    wo = 8;
    if isfield(s, 'woEso'), wo = s.woEso; end
    eso = uav.ctrl.eso_init(wo);
    esoNoise = 0;
    if isfield(s, 'esoNoise'), esoNoise = s.esoNoise; end
    % 补偿上限，默认 6 m/s²：本构型 6 m/s 侧风的气动阻力约 0.7 N，
    % 折算成加速度不到 0.1 m/s²，再大的估计就是量测误差或模型失配。
    esoMax = 6;
    if isfield(s, 'esoMax'), esoMax = s.esoMax; end
    aEso = zeros(3, 1);
end

j = 0;
for kk = 1:N
    t = (kk-1)*dt;

    actR = ones(4, 1);
    if isfield(s, 'fault') && t >= s.fault.t
        % k 与 eta 允许是向量，用于多桨失效；标量时退化为单桨
        actR(s.fault.k) = s.fault.eta;
    end

    if mod(kk-1, s.ctrlDiv) == 0
        rNow = eval_ref(ref, t);
        % 减速模态：参考在减速、且空速还够（<3 m/s 时交回悬停级联）。
        % 只有 s.brake 打开时才走这条通道，默认关闭以免影响转捩与包线实验。
        useBrake = isfield(s, 'brake') && s.brake && ...
                   rNow.a(1) < -0.05 && norm(x(4:6)) > 3.0;
        % 应急模式：判死之后放弃水平控制，只保高度与姿态（自转不管）。
        % 悬停断桨时水平位置环会把偏航自转当成航向误差去追，反而把机体带翻
        % （r10 实测滚转 168°、掉高 106 m）。这里把水平参考锁到当前点、
        % 航向锁到当前航向，姿态环只做稳姿与定高，并按 emergencyRate 下降。
        useEmerg = isfield(s, 'emergency') && s.emergency && ...
                   ((isfield(s, 'fault') && any(actR < 1)) || ...
                    (isfield(s, 'detect') && s.detect && any(det.dead)));
        if useEmerg
            if ~isfield(det, 'tDet'), det.tDet = t; end
            hCmd = -rNow.p(3) - s.emergencyRate*max(0, t - det.tDet);
            hCmd = max(0, hCmd);
            eulNow = uav.plant.quat2euler(x(7:10));
            rNow.p = [x(1); x(2); -hCmd];
            rNow.v = zeros(3,1); rNow.a = zeros(3,1);
            rNow.psi = eulNow(3);
            rNow.h = hCmd; rNow.V = norm(x(4:6));
        end
        % 扰动观测（r18，opt-in）：用位置量测驱动三轴 ESO，把估计出的
        % 扰动加速度从期望加速度里扣掉。悬停抗风靠它，而不是靠误差硬顶。
        if useEso && ~useEmerg
            pMeas = x(1:3) + esoNoise * randn(3, 1);
            [eso, dHatNow] = uav.ctrl.eso_step(eso, pMeas, aEso, s.dt*s.ctrlDiv);
            % 补偿量限幅（r19）：风速再大，扰动加速度估计也不该超过
            % 气动阻力能给出的量级；不封顶时 6 m/s 侧风把推力顶到饱和
            % （四台 1100 rad/s）后姿态环失去权限，机体会翻。
            dHatNow = max(-esoMax, min(esoMax, dHatNow));
            % aEso 记的是"控制器自己算的期望加速度"（补偿前），否则 ESO 的
            % 输入里含 −dHat，会把补偿量再估一遍，形成双份补偿的静差。
            aEso = rNow.a;
            rNow.a = rNow.a - dHatNow;
        end
        % 控制输入摄动（r22）：只影响控制器看到的量测，不影响真值积分。
        % attBias 是加在估计四元数上的小角偏置（弧度），posNoise 是位置白噪
        % 标准差。用来回答"估计器差多少会出事"，而不是只测真值闭环。
        xCtrl = x;
        if isfield(s, 'pert')
            if isfield(s.pert, 'posNoise') && s.pert.posNoise > 0
                xCtrl(1:3) = xCtrl(1:3) + s.pert.posNoise*randn(3,1);
            end
            if isfield(s.pert, 'posBias'), xCtrl(1:3) = xCtrl(1:3) + s.pert.posBias; end
            if isfield(s.pert, 'attBias') && any(s.pert.attBias)
                dq = [1; 0.5*s.pert.attBias(:)];
                dq = dq/norm(dq);
                xCtrl(7:10) = uav.plant.qmul(dq, x(7:10));
            end
            if attDelay > 0
                qHist = [x(7:10), qHist(:, 1:end-1)];
                xCtrl(7:10) = qHist(:, end);
            end
        end
        if useBrake
            [cmd, st] = uav.ctrl.tilt_brake(xCtrl, rNow, st, p, c, s.dt*s.ctrlDiv);
        else
            % faultAware=false 时控制器不知道故障（旧行为），用于对照实验
            if isfield(s, 'faultAware') && ~s.faultAware
                actC = ones(4, 1);
            elseif isfield(s, 'detect') && s.detect
                % 不预先告诉控制器 η，只从"指令 - 实测转速"残差里判失效，
                % 判死后按完全失效分配（r9 研究：延迟约 4 ms，性能接近先知先觉）。
                actC = ones(4, 1);
                actC(det.dead) = 0;
            else
                actC = actR;
                % faultHard：检测到降效就当作完全失效处理。降效 35% 时伪逆会
                % 在剩余三台上解出一个带偏置的最小范数解，反而把偏航残差做成
                % 持续的姿态偏置（r8 里 drift 4.8 m，比按完全失效处理差一倍）。
                if isfield(s, 'faultHard') && s.faultHard
                    actC = (actR > 0.5) .* actR;
                end
            end
            if isfield(s, 'alloc'), mAlloc = s.alloc; else, mAlloc = 'pinv'; end
            [cmd, st] = uav.ctrl.tilt_cascade(xCtrl, rNow, st, p, c, s.dt*s.ctrlDiv, actC, mAlloc);
        end
        cmd = cmd(:);
        % 差动短舱偏航微调（REPORT_r11）：四台短舱角不再完全一致，
        % 按 beta_i = beta + trim_i 产生可控偏航力矩。beta=90° 时 cos(beta)=0，
        % 没有权限，直接跳过；trim 也只在活着（act>0）的电机上加。
        betaTrim = zeros(4, 1);
        if isfield(s, 'yawTrim') && s.yawTrim.enable
            bNow  = x(14);
            cbNow = cos(bNow);
            if abs(cbNow) > 0.15
                eulNow = uav.plant.quat2euler(x(7:10));
                psiErr = uav.ctrl.wrap_pi(rNow.psi - eulNow(3));
                uYaw   = s.yawTrim.kp * psiErr - s.yawTrim.kd * x(12);
                Tnow   = p.kt * x(15:18).^2;
                % 三约束联解：不产生侧向力、不产生俯仰力矩、只出偏航力矩。
                % 单轴差动在单桨失效时会把偏航需求漏成俯仰（rx·ry 之和不再为零），
                % 实测只做 Σry 反解时漂移从 38.6 涨到 66.5 m。
                alive = find(actR > 0.5);
                if numel(alive) >= 2
                    A = [ Tnow(alive)'; ...
                          p.rx(alive)' .* Tnow(alive)' * cbNow; ...
                         -p.ry(alive)' .* Tnow(alive)' * cbNow ];
                    b = [0; 0; uYaw];
                    tAlive = lsqminnorm(A, b);
                    tAlive = max(-s.yawTrim.maxTrim, min(s.yawTrim.maxTrim, tAlive));
                    betaTrim(alive) = tAlive;
                end
            end
        end
        if isfield(s, 'detect') && s.detect
            [det, newIdx] = uav.est.rotor_fault_detect(cmd(1:4), x(15:18), det);
            if ~isempty(newIdx)
                tDetLog(end+1, 1) = t; %#ok<AGROW>
                iDetLog(end+1, 1) = newIdx(1); %#ok<AGROW>
            end
            if ~isempty(newIdx) && isfield(s, 'logFault') && s.logFault
                fprintf('  [detect] t=%.3f s 判定电机 %s 失效\n', t, mat2str(newIdx'));
            end
        end
    end

    % 记录 t 时刻状态（积分之前），与 Simulink 积分器输出对齐
    if mod(kk-1, s.logDiv) == 0
        j = j + 1;
        tLog(j) = t;
        xLog(j, :) = x';
        pLog(j, :) = x(1:3)';
        vLog(j, :) = x(4:6)';
        eLog(j, :) = uav.plant.quat2euler(x(7:10))';
        wLog(j, :) = x(11:13)';
        cLog(j, :) = cmd';
        [FbNow, ~] = uav.plant.tilt_wrench(x, sqrt(actR).*x(15:18), x(19:21), x(14), p);
        fLog(j, :) = (FbNow / p.mass)';
        rLog(j, :) = [rNow.p(:); rNow.psi; rNow.h; rNow.V]';
    end

    if isfield(s, 'windFn')
        Wned = s.windFn(t);
        WbW  = uav.plant.q2dcm(x(7:10))' * Wned(:);
    else
        WbW = zeros(3, 1);
    end
    k1 = deriv(x, cmd, p, actR, betaTrim, WbW);
    k2 = deriv(x + 0.5*dt*k1, cmd, p, actR, betaTrim, WbW);
    k3 = deriv(x + 0.5*dt*k2, cmd, p, actR, betaTrim, WbW);
    k4 = deriv(x + dt*k3, cmd, p, actR, betaTrim, WbW);
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(7:10) = x(7:10)/norm(x(7:10));
    x(15:18) = max(0, min(p.wMax, x(15:18)));
end

out.t    = tLog(1:j);
out.x    = xLog(1:j, :);
out.p    = pLog(1:j, :);
out.v    = vLog(1:j, :);
out.eul  = eLog(1:j, :);
out.w    = wLog(1:j, :);
out.cmd  = cLog(1:j, :);
 out.f    = fLog(1:j, :);
out.ref  = rLog(1:j, :);
out.beta = xLog(1:j, 14);
out.om   = xLog(1:j, 15:18);
out.det  = struct('t', tDetLog, 'idx', iDetLog);
out.prm  = p;
end

function xd = deriv(x, cmd, p, actR, betaTrim, WbW)
om   = sqrt(actR) .* x(15:18);   % 降效电机：转速不变，推力按 sqrt(act) 缩
act  = x(19:21);
beta = x(14);
if nargin < 5 || isempty(betaTrim), betaTrim = zeros(4, 1); end

if nargin < 6 || isempty(WbW), WbW = zeros(3, 1); end
[Fb, Mb] = uav.plant.tilt_wrench(x, om, act, beta + betaTrim(:), p, WbW);
xd13 = uav.plant.rigid_body(x(1:13), Fb, Mb, p);

betaDot = (cmd(8) - beta) / p.tauBeta;
betaDot = max(-p.betaRate, min(p.betaRate, betaDot));

omDot  = (cmd(1:4) - om) / p.tauM;
actDot = (cmd(5:7) - act) / p.tauA;

xd = [xd13; betaDot; omDot; actDot];
end

function r = eval_ref(ref, t)
if isa(ref, 'function_handle')
    r = ref(t);
else
    r = ref;
end
end
