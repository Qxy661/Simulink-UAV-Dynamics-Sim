function ref = hover(pn, pe, pd, psi)
%HOVER  定点悬停参考。
%   pd 为 NED 高度（向下为正），5 m 高度写作 -5。

ref.p   = [pn; pe; pd];
ref.v   = [0; 0; 0];
ref.a   = [0; 0; 0];
ref.psi = psi;
end
