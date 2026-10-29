function [Fb, Mb] = tilt_wrench(x, om, act, beta, p)
%TILT_WRENCH  倾转四旋翼的机体力与力矩。
%   x    : 21 维状态，只用 4:6（机体速度）与 11:13（角速度）
%   om   : 4x1 旋翼转速 (rad/s)
%   act  : 3x1 舵面 [de; da; dr] (rad)
%   beta : 短舱倾角 (rad)，0 推力朝上，pi/2 推力朝前
%
%   旋翼推力方向 d = [sin(beta); 0; -cos(beta)]（机体系，z 向下）。
%   取虚拟控制量
%       U1 = sum(Ti)
%       U2 = -sum(yi*Ti)     横向差动（滚转）
%       U3 =  sum(xi*Ti)     纵向差动（俯仰）
%       U4 =  sum(si*Ti)     反扭矩差动（偏航）
%   则旋翼力偶为
%       Mx = cos(beta)*U2 - kq*sin(beta)*U4
%       My = cos(beta)*U3
%       Mz = sin(beta)*U2 + kq*cos(beta)*U4
%   在 beta=0 时退化为四旋翼的滚转/俯仰差动 + 反扭矩偏航；beta=90 度时推力不再
%   产生滚转，只能靠反扭矩，滚转权限交给副翼——这正是过渡段要做权限混合的原因。

Vb = x(4:6);
w  = x(11:13);

[Fa, Ma] = uav.plant.fw_aero(Vb, w, [0; act(:)], p);

T  = p.kt * om(:).^2;
U1 = sum(T);
U2 = -sum(p.ry(:) .* T);
U3 = sum(p.rx(:) .* T);
U4 = sum(p.spin(:) .* T);

d  = [sin(beta); 0; -cos(beta)];
Fr = d * U1;

Mr = [ cos(beta)*U2 - p.kq*sin(beta)*U4;
       cos(beta)*U3;
       sin(beta)*U2 + p.kq*cos(beta)*U4 ];

Fb = Fa + Fr;
Mb = Ma + Mr;
end
