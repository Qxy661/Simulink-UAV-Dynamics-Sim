function X = fw_sim()
%FW_SIM  固定翼闭环仿真（C 代码生成入口）。
%   X = uav.codegen.fw_sim()
%   与 uav.sim.run_fw 的数值完全一致：RK4 @1 ms、控制器 100 Hz、
%   舵机与油门一阶滞后精确离散、记录积分前的状态。
%   任务：100 m 平飞起，t=5 s 爬升 30 m，t=20 s 右转 60°，t=35 s 加速到 22 m/s。
%   返回 6000 x 13 状态序列，采样间隔 10 ms，总时长 60 s。
%
%#codegen

p = uav.cfg.fw();
c = uav.cfg.fwctrl();

h0 = 100;
V0 = 18;

dt      = 1e-3;
ctrlDiv = 10;
logDiv  = 10;
nStep   = 60000;
nLog    = nStep/logDiv;

tr  = uav.plant.fw_trim(p, V0);
q0  = uav.plant.euler2quat(0, tr.alpha, 0);
Vb0 = V0*[cos(tr.alpha); 0; sin(tr.alpha)];

X   = zeros(nLog, 13);
x   = [0; 0; -h0; Vb0; q0; 0; 0; 0];
act = [tr.thr; tr.de; 0; 0];
cmd = act;
st  = struct('iV', 0);
j   = 0;

for k = 1:nStep
    t = (k - 1)*dt;

    if mod(k - 1, ctrlDiv) == 0
        r = uav.env.fw_ref(t, h0, V0);
        [cmd, st] = uav.ctrl.fw_cascade(x, r, st, p, c, dt*ctrlDiv);
    end

    act = uav.plant.actuator_step(cmd, act, p, dt);

    if mod(k - 1, logDiv) == 0
        j = j + 1;
        X(j, :) = x';
    end

    k1 = deriv(x, act, p);
    k2 = deriv(x + 0.5*dt*k1, act, p);
    k3 = deriv(x + 0.5*dt*k2, act, p);
    k4 = deriv(x + dt*k3, act, p);

    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(7:10) = x(7:10)/norm(x(7:10));
end
end

function xd = deriv(x, act, p)
Vb = x(4:6);
w  = x(11:13);
[Fb, Mb] = uav.plant.fw_aero(Vb, w, act, p);
xd = uav.plant.rigid_body(x, Fb, Mb, p);
end
