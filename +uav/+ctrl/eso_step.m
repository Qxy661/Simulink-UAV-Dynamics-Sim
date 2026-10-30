function [st, dHat] = eso_step(st, pMeas, aCmd, dt)
%ESO_STEP  推进一步三轴线性 ESO，返回扰动加速度估计。
%   pMeas 为位置量测（NED，3x1），aCmd 为控制器已经给出的期望加速度
%   （NED，3x1），不含扰动补偿。dHat 即 z3，用来做前馈补偿：
%       aCmd_total = aCmd - dHat
%   离散用前向欧拉，dt 取控制周期。

wo = st.wo;

if ~st.init
    st.z(:, 1) = pMeas(:);
    st.init = true;
end

e1 = pMeas(:) - st.z(:, 1);
dz1 = st.z(:, 2) + 3 * wo * e1;
dz2 = st.z(:, 3) + aCmd(:) + 3 * wo^2 * e1;
dz3 = wo^3 * e1;

st.z(:, 1) = st.z(:, 1) + dt * dz1;
st.z(:, 2) = st.z(:, 2) + dt * dz2;
st.z(:, 3) = st.z(:, 3) + dt * dz3;

dHat = st.z(:, 3);
end
