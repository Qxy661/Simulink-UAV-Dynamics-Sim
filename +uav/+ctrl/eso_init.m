function st = eso_init(wo)
%ESO_INIT  三轴线性扩张状态观测器（位置型）。
%   st = eso_init(wo) 返回结构体，wo 为观测器带宽（rad/s）。
%   每轴三阶：z1≈位置，z2≈速度，z3≈扰动加速度。状态初值由第一次
%   调用补齐，这里只置零与标志位。

st.z = zeros(3, 3);        % 三轴 × 三阶
st.wo = wo;
st.init = false;
end
