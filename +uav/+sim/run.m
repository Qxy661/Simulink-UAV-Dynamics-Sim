function out = run(p, c, s, ref, x0)
%RUN  四旋翼闭环仿真。
%   out = uav.sim.run(p, c, s, ref, x0)
%   ref 为参考结构体或 @(t) 返回参考结构体的函数句柄。
%   x0 省略时按参考初始位置悬停起飞。
%   积分用 RK4（dt），控制器每 ctrlDiv 步更新一次。

if nargin < 5 || isempty(x0)
    r0 = eval_ref(ref, 0);
    x0 = [r0.p; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
end

dt   = s.dt;
N    = round(s.tEnd/dt);
nLog = floor(N/s.logDiv) + 1;

tLog = zeros(nLog, 1);
xLog = zeros(nLog, 13);
pLog = zeros(nLog, 3);
vLog = zeros(nLog, 3);
eLog = zeros(nLog, 3);
wLog = zeros(nLog, 4);
uLog = zeros(nLog, 4);
rLog = zeros(nLog, 3);
satLog = false(nLog, 1);

x = x0(:);
w = repmat(p.hoverW, 4, 1);
st = struct('ip', [0; 0; 0], 'ia', [0; 0; 0]);
wcmd = w;
Tcmd = 0;
Mcmd = [0; 0; 0];
sat  = false;
rNow = eval_ref(ref, 0);

j = 0;
for k = 1:N
    t = (k-1)*dt;

    if mod(k-1, s.ctrlDiv) == 0
        rNow = eval_ref(ref, t);
        [Tcmd, Mcmd, st] = uav.ctrl.cascade(x, rNow, p, c, st, s.dt*s.ctrlDiv);
        [wcmd, sat] = uav.plant.mixer([Tcmd; Mcmd], p);
    end

    k1 = deriv(x, w, p);
    k2 = deriv(x + 0.5*dt*k1, w, p);
    k3 = deriv(x + 0.5*dt*k2, w, p);
    k4 = deriv(x + dt*k3, w, p);
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(7:10) = x(7:10)/norm(x(7:10));

    w = uav.plant.motor_step(w, wcmd, p, dt);

    if mod(k-1, s.logDiv) == 0
        j = j + 1;
        tLog(j) = t;
        xLog(j, :) = x';
        pLog(j, :) = x(1:3)';
        vLog(j, :) = x(4:6)';
        eLog(j, :) = uav.plant.quat2euler(x(7:10))';
        wLog(j, :) = w';
        uLog(j, :) = [Tcmd, Mcmd'];
        rLog(j, :) = rNow.p';
        satLog(j) = sat;
    end
end

out.t    = tLog(1:j);
out.x    = xLog(1:j, :);
out.p    = pLog(1:j, :);
out.v    = vLog(1:j, :);
out.eul  = eLog(1:j, :);
out.w    = wLog(1:j, :);
out.u    = uLog(1:j, :);
out.ref  = rLog(1:j, :);
out.sat  = satLog(1:j);
out.prm  = p;
end

function xd = deriv(x, w, p)
Vb = x(4:6);
[Fb, Mb] = uav.plant.quad_wrench(w, Vb, p);
xd = uav.plant.rigid_body(x, Fb, Mb, p);
end

function r = eval_ref(ref, t)
if isa(ref, 'function_handle')
    r = ref(t);
else
    r = ref;
end
end
