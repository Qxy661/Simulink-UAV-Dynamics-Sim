/*
 * File: tilt_sim.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-31 05:20:27
 */

/* Include Files */
#include "tilt_sim.h"
#include "rt_nonfinite.h"
#include "tilt_cascade.h"
#include "tilt_sim_data.h"
#include "tilt_sim_initialize.h"
#include "tilt_sim_internal_types.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Function Declarations */
static void deriv(const double x[21], const double cmd[8], double xd[21]);

/* Function Definitions */
/*
 * Arguments    : const double x[21]
 *                const double cmd[8]
 *                double xd[21]
 * Return Type  : void
 */
static void deriv(const double x[21], const double cmd[8], double xd[21])
{
  static const double a[9] = {0.9, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 1.6};
  static const double dv[4] = {0.9, 0.9, -0.9, -0.9};
  static const double dv1[4] = {0.4, -0.4, -0.4, 0.4};
  __m128d r;
  __m128d r1;
  double R[9];
  double T[4];
  double b_x[4];
  double B[3];
  double Vb[3];
  double c_d_tmp[3];
  double c_x[3];
  double d_x[3];
  double D;
  double L;
  double R_tmp;
  double U1;
  double U4;
  double V;
  double Ve;
  double absxk;
  double alpha;
  double b_R_tmp;
  double b_d_tmp;
  double beta;
  double d_tmp;
  double qbar;
  double scale;
  double t;
  double y;
  int i;
  /* TILT_WRENCH  倾转四旋翼的机体力与力矩。 */
  /*    x    : 21 维状态，只用 4:6（机体速度）与 11:13（角速度） */
  /*    om   : 4x1 旋翼转速 (rad/s) */
  /*    act  : 3x1 舵面 [de; da; dr] (rad) */
  /*    beta : 短舱倾角 (rad)，0 推力朝上，pi/2 推力朝前 */
  /*  */
  /*    旋翼推力方向 d = [sin(beta); 0; -cos(beta)]（机体系，z 向下）。 */
  /*    取虚拟控制量 */
  /*        U1 = sum(Ti) */
  /*        U2 = -sum(yi*Ti)     横向差动（滚转） */
  /*        U3 =  sum(xi*Ti)     纵向差动（俯仰） */
  /*        U4 =  sum(si*Ti)     反扭矩差动（偏航） */
  /*    则旋翼力偶为 */
  /*        Mx = cos(beta)*U2 - kq*sin(beta)*U4 */
  /*        My = cos(beta)*U3 */
  /*        Mz = sin(beta)*U2 + kq*cos(beta)*U4 */
  /*    在 beta=0 时退化为四旋翼的滚转/俯仰差动 + 反扭矩偏航；beta=90
   * 度时推力不再 */
  /*    产生滚转，只能靠反扭矩，滚转权限交给副翼——这正是过渡段要做权限混合的原因。
   */
  /* FW_AERO  固定翼气动力与力矩。 */
  /*    Vb  : 机体系速度 [u; v; w] (m/s) */
  /*    w   : 机体系角速度 [p; q; r] (rad/s) */
  /*    act : 执行机构 [throttle; de; da; dr] */
  /*          throttle 0..1，舵面 rad，正 de 为抬头方向的定义见 fw_aero 文档 */
  /*    Fb  : 机体系力 (N)，包含推力、升力、阻力、侧力 */
  /*    Mb  : 机体力矩 (N*m) */
  /*  */
  /*    气动角：alpha = atan2(w, u)，beta = asin(v/V)。 */
  /*    升力沿稳定性 z 轴负方向（机腹向上为负），阻力沿 -x，再按 alpha
   * 转到机体轴。 */
  /*  风：气动力量按相对气流算（Wb 为机体系风速；NED 风速要先转到机体） */
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[3]);
  if (absxk > 3.3121686421112381E-170) {
    V = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    V = t * t;
  }
  absxk = fabs(x[4]);
  if (absxk > scale) {
    t = scale / absxk;
    V = V * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    V += t * t;
  }
  absxk = fabs(x[5]);
  if (absxk > scale) {
    t = scale / absxk;
    V = V * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    V += t * t;
  }
  V = scale * sqrt(V);
  if (V >= 1.0) {
    Ve = V;
  } else {
    Ve = 1.0;
  }
  /*  低速时避免除零，只在悬停/起飞段生效 */
  /*  线性气动只在失速前有效：悬停遇侧风时相对气流可以接近垂直于机身， */
  /*  atan2(0,-2) 这种退化情形会给出 180°，线性 CL/CY 会算出几十牛的气动力 */
  /*  把整机掀翻（r17 实测 6 m/s 侧风滚转 141°）。这里用 asin 速度分量比， */
  /*  并把两个角都截到 ±19.5°——相当于"失速后气动力不再增长"的最粗模型。 */
  t = x[5] / Ve;
  if ((t >= 0.34) || rtIsNaN(t)) {
    t = 0.34;
  }
  if (t <= -0.34) {
    t = -0.34;
  }
  alpha = asin(t);
  t = x[4] / Ve;
  if ((t >= 0.34) || rtIsNaN(t)) {
    t = 0.34;
  }
  if (t <= -0.34) {
    t = -0.34;
  }
  beta = asin(t);
  qbar = 0.6125 * (V * V);
  V = ((5.0 * alpha + 0.45) + 3.9 * x[11] * 0.34 / (2.0 * Ve)) + 0.4 * x[18];
  D = qbar * 1.6 * (0.045 * (V * V) + 0.025);
  L = qbar * 1.6 * V;
  /*  稳定性轴到机体轴 */
  r = _mm_loadu_pd(&x[14]);
  r1 = _mm_set1_pd(2.6E-5);
  _mm_storeu_pd(&T[0], _mm_mul_pd(r1, _mm_mul_pd(r, r)));
  r = _mm_loadu_pd(&x[16]);
  _mm_storeu_pd(&T[2], _mm_mul_pd(r1, _mm_mul_pd(r, r)));
  U1 = ((T[0] + T[1]) + T[2]) + T[3];
  r = _mm_loadu_pd(&T[0]);
  _mm_storeu_pd(&b_x[0], _mm_mul_pd(_mm_loadu_pd(&dv[0]), r));
  r = _mm_loadu_pd(&T[2]);
  _mm_storeu_pd(&b_x[2], _mm_mul_pd(_mm_loadu_pd(&dv[2]), r));
  y = ((b_x[0] + b_x[1]) + b_x[2]) + b_x[3];
  r = _mm_loadu_pd(&T[0]);
  _mm_storeu_pd(&b_x[0], _mm_mul_pd(_mm_loadu_pd(&dv1[0]), r));
  r = _mm_loadu_pd(&T[2]);
  _mm_storeu_pd(&b_x[2], _mm_mul_pd(_mm_loadu_pd(&dv1[2]), r));
  T[1] = -T[1];
  T[3] = -T[3];
  U4 = ((T[0] + T[1]) + T[2]) + T[3];
  /*  短舱角可以是标量（四台同角）或 4x1（差动微调，见 REPORT_r11） */
  d_tmp = cos(x[13]);
  b_d_tmp = sin(x[13]);
  /* RIGID_BODY  刚体六自由度方程（机体 FRD，世界 NED）。 */
  /*    状态 x = [pn;pe;pd; u;v;w; qw;qx;qy;qz; p;q;r]。 */
  /*    Fb/Mb 为机体系合力与合力矩（不含重力）。 */
  /* Q2DCM  四元数转旋转矩阵（机体系 -> NED）。 */
  /*    q = [w; x; y; z]，要求单位化；非单位输入按单位处理。 */
  /*    对应 ZYX 欧拉角：R = Rz(psi)*Ry(theta)*Rx(phi)。 */
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[6]);
  if (absxk > 3.3121686421112381E-170) {
    V = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    V = t * t;
  }
  absxk = fabs(x[7]);
  if (absxk > scale) {
    t = scale / absxk;
    V = V * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    V += t * t;
  }
  absxk = fabs(x[8]);
  if (absxk > scale) {
    t = scale / absxk;
    V = V * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    V += t * t;
  }
  absxk = fabs(x[9]);
  if (absxk > scale) {
    t = scale / absxk;
    V = V * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    V += t * t;
  }
  V = scale * sqrt(V);
  if (V < 1.0E-12) {
    T[0] = 1.0;
    T[1] = 0.0;
    T[2] = 0.0;
    T[3] = 0.0;
  } else {
    r = _mm_set1_pd(V);
    _mm_storeu_pd(&T[0], _mm_div_pd(_mm_loadu_pd(&x[6]), r));
    _mm_storeu_pd(&T[2], _mm_div_pd(_mm_loadu_pd(&x[8]), r));
  }
  V = T[3] * T[3];
  scale = T[2] * T[2];
  R[0] = 1.0 - 2.0 * (scale + V);
  absxk = T[1] * T[2];
  t = T[0] * T[3];
  R[3] = 2.0 * (absxk - t);
  R_tmp = T[1] * T[3];
  b_R_tmp = T[0] * T[2];
  R[6] = 2.0 * (R_tmp + b_R_tmp);
  R[1] = 2.0 * (absxk + t);
  absxk = T[1] * T[1];
  R[4] = 1.0 - 2.0 * (absxk + V);
  V = T[2] * T[3];
  t = T[0] * T[1];
  R[7] = 2.0 * (V - t);
  R[2] = 2.0 * (R_tmp - b_R_tmp);
  R[5] = 2.0 * (V + t);
  R[8] = 1.0 - 2.0 * (absxk + scale);
  T[0] = 0.0;
  /* QMUL  Hamilton 四元数乘法 a (x) b。 */
  /*    与 [w;x;y;z] 排列一致，用于 qdot = 0.5*q (x) [0;omega]。 */
  t = (cmd[7] - x[13]) / 0.15;
  if ((t >= 0.52359877559829882) || rtIsNaN(t)) {
    t = 0.52359877559829882;
  }
  c_d_tmp[0] = b_d_tmp * U1;
  c_d_tmp[1] = 0.0 * U1;
  c_d_tmp[2] = -d_tmp * U1;
  c_x[0] = x[5] * x[11] - x[4] * x[12];
  c_x[1] = x[3] * x[12] - x[5] * x[10];
  c_x[2] = x[4] * x[10] - x[3] * x[11];
  absxk = 0.0;
  for (i = 0; i < 3; i++) {
    V = x[i + 10];
    T[i + 1] = V;
    Vb[i] = (a[i] * x[10] + a[i + 3] * x[11]) + a[i + 6] * x[12];
    absxk += x[i + 7] * V;
  }
  scale = qbar * 1.6 * 3.6;
  B[0] = (scale * (((-0.09 * beta + -0.5 * x[10] * 3.6 / (2.0 * Ve)) +
                    0.08 * x[12] * 3.6 / (2.0 * Ve)) +
                   0.18 * x[19]) +
          (d_tmp * -y - 0.019230769230769232 * b_d_tmp * U4)) -
         (Vb[2] * x[11] - Vb[1] * x[12]);
  B[1] = (qbar * 1.6 * 0.34 *
              (((-0.8 * alpha + 0.05) + -12.0 * x[11] * 0.34 / (2.0 * Ve)) +
               -1.2 * x[18]) +
          d_tmp * (((b_x[0] + b_x[1]) + b_x[2]) + b_x[3])) -
         (Vb[0] * x[12] - Vb[2] * x[10]);
  B[2] = (scale * (((0.06 * beta + -0.03 * x[10] * 3.6 / (2.0 * Ve)) +
                    -0.12 * x[12] * 3.6 / (2.0 * Ve)) +
                   0.09 * x[20]) +
          (b_d_tmp * -y + 0.019230769230769232 * d_tmp * U4)) -
         (Vb[1] * x[10] - Vb[0] * x[11]);
  Vb[1] = B[1] - B[0] * 0.0;
  Vb[2] = (B[2] - B[0] * 0.0) - Vb[1] * 0.0;
  Vb[2] /= 1.6;
  Vb[0] = B[0] - Vb[2] * 0.0;
  Vb[1] -= Vb[2] * 0.0;
  Vb[1] /= 1.2;
  Vb[0] -= Vb[1] * 0.0;
  Vb[0] /= 0.9;
  scale = sin(alpha);
  V = cos(alpha);
  B[0] = -D * V + L * scale;
  B[1] = qbar * 1.6 * (-0.6 * beta + 0.19 * x[20]);
  B[2] = -D * scale - L * V;
  d_x[0] = x[8] * x[12] - x[9] * x[11];
  d_x[1] = x[9] * x[10] - x[7] * x[12];
  d_x[2] = x[7] * x[11] - x[8] * x[10];
  xd[6] = 0.5 * (x[6] * 0.0 - absxk);
  for (i = 0; i < 3; i++) {
    xd[i] = (R[i] * x[3] + R[i + 3] * x[4]) + R[i + 6] * x[5];
    xd[i + 3] = ((B[i] + c_d_tmp[i]) / 8.0 - c_x[i]) +
                ((R[3 * i] * 0.0 + R[3 * i + 1] * 0.0) + R[3 * i + 2] * 9.81);
    xd[i + 7] = 0.5 * ((x[6] * T[i + 1] + 0.0 * x[i + 7]) + d_x[i]);
    xd[i + 10] = Vb[i];
  }
  if (t <= -0.52359877559829882) {
    xd[13] = -0.52359877559829882;
  } else {
    xd[13] = t;
  }
  r = _mm_set1_pd(0.02);
  _mm_storeu_pd(
      &xd[14],
      _mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&cmd[0]), _mm_loadu_pd(&x[14])), r));
  _mm_storeu_pd(
      &xd[16],
      _mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&cmd[2]), _mm_loadu_pd(&x[16])), r));
  _mm_storeu_pd(&xd[18], _mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&cmd[4]),
                                               _mm_loadu_pd(&x[18])),
                                    _mm_set1_pd(0.05)));
  xd[20] = (cmd[6] - x[20]) / 0.05;
}

/*
 * TILT_SIM  倾转四旋翼闭环仿真（C 代码生成入口）。
 *    X = uav.codegen.tilt_sim()
 *    与 uav.sim.run_tilt 的数值完全一致：RK4 @1 ms、控制器 100 Hz、
 *    短舱/转速/舵面一阶滞后在导数里精确积分、记录积分前的状态。
 *    任务：20 m 悬停 5 s，随后沿正北加速到 10 m/s 并进入过渡段。
 *    返回 4500 x 21 状态序列，采样间隔 10 ms，总时长 45 s。
 *
 *
 * Arguments    : double X[94500]
 * Return Type  : void
 */
void tilt_sim(double X[94500])
{
  static const double dv[21] = {0.0,
                                0.0,
                                -20.0,
                                0.0,
                                0.0,
                                0.0,
                                1.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                868.686010371633,
                                868.686010371633,
                                868.686010371633,
                                868.686010371633,
                                0.0,
                                0.0,
                                0.0};
  static const double dv1[8] = {868.686010371633,
                                868.686010371633,
                                868.686010371633,
                                868.686010371633,
                                0.0,
                                0.0,
                                0.0,
                                0.0};
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r3;
  __m128d r4;
  __m128d r5;
  struct_T st;
  double b_x[21];
  double dv2[21];
  double k1[21];
  double k2[21];
  double k3[21];
  double x[21];
  double cmd[8];
  double r_a[3];
  double r_p[3];
  double r_v[3];
  double V;
  double aN;
  double pN;
  double t;
  int i;
  int j;
  int k;
  if (!isInitialized_tilt_sim) {
    tilt_sim_initialize();
  }
  memset(&X[0], 0, 94500U * sizeof(double));
  memcpy(&x[0], &dv[0], 21U * sizeof(double));
  memcpy(&cmd[0], &dv1[0], 8U * sizeof(double));
  st.ip[0] = 0.0;
  st.ia[0] = 0.0;
  st.ip[1] = 0.0;
  st.ia[1] = 0.0;
  st.ip[2] = 0.0;
  st.ia[2] = 0.0;
  st.iH = 0.0;
  j = -1;
  r = _mm_set1_pd(2.0);
  for (k = 0; k < 45000; k++) {
    t = (((double)k + 1.0) - 1.0) * 0.001;
    if (k == 0) {
      i = 0;
    } else {
      i = (int)fmod(((double)k + 1.0) - 1.0, 10.0);
    }
    if (i == 0.0) {
      /* TILT_REF  倾转四旋翼的任务参考（解析式，便于在图表里内联）。 */
      /*    0–5 s    20 m 悬停 */
      /*    5–20 s   沿正北加速到 10 m/s，高度保持 20 m */
      /*    20–45 s  巡航 */
      /*    45–60 s  减速到悬停 */
      /*    60 s 后  20 m 悬停 */
      /*  */
      /*    高度在过渡段保持不变。早期版本让加速与爬升同时进行（20→40 m）， */
      /*    纵向需求把推力方向拉得几乎竖直，机体被迫低头不够，实测卡在 32° */
      /*    爬升、机体反而抬头 7.6°，速度停在 4 m/s。过渡段先做平飞加速， */
      /*    把纵向与横向耦合分开，等机翼接管后再做高度机动。 */
      /*  */
      /*    返回 struct：p（NED 位置）、v、a、psi、h、V。 */
      /*    加速度用余弦包络的一阶导数，速度与位置用其解析积分，避免数值积分误差。
       */
      /*  m/s，巡航速度；β 到 90°（见 docs/tiltrotor.md） */
      /*  m/s，减速后保留的低速巡航 */
      /*  s，加/减速时长 */
      /*  加速开始 */
      /*  20 */
      /*  减速开始 */
      /*  60 */
      /*  m，全程高度 */
      /*  加速段位移 132 m */
      /*  巡航结束位置 */
      /*  ---- 速度与加速度 ---- */
      if (t <= 5.0) {
        V = 0.0;
        aN = 0.0;
        pN = 0.0;
      } else if (t <= 20.0) {
        pN = 3.1415926535897931 * (t - 5.0) / 15.0;
        V = 11.0 * (1.0 - cos(pN));
        pN = sin(pN);
        aN = 2.3038346126325147 * pN;
        pN = 11.0 * ((t - 5.0) - 4.7746482927568605 * pN);
      } else {
        /*  加速段结束后保持 10 m/s 巡航。任务只覆盖"悬停 → 转捩 → 巡航"， */
        /*  不做减速回悬停：减速段的姿态混合会削弱倾角需求，实测位置滞后 */
        /*  25 m 且 β 收回到 8° 后机体继续漂移，见 docs/tiltrotor.md 未解决项。
         */
        V = 22.0;
        aN = 0.0;
        pN = 22.0 * (t - 20.0) + 165.0;
      }
      r_p[0] = pN;
      r_p[1] = 0.0;
      r_p[2] = -20.0;
      r_v[0] = V;
      r_v[1] = 0.0;
      r_v[2] = 0.0;
      r_a[0] = aN;
      r_a[1] = 0.0;
      r_a[2] = 0.0;
      tilt_cascade(x, r_p, r_v, r_a, V, &st, cmd);
      j++;
      for (i = 0; i < 21; i++) {
        X[j + 4500 * i] = x[i];
      }
    }
    deriv(x, cmd, k1);
    for (i = 0; i <= 18; i += 2) {
      r1 = _mm_loadu_pd(&k1[i]);
      r2 = _mm_loadu_pd(&x[i]);
      _mm_storeu_pd(&b_x[i],
                    _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(0.0005), r1)));
    }
    b_x[20] = x[20] + 0.0005 * k1[20];
    deriv(b_x, cmd, k2);
    for (i = 0; i <= 18; i += 2) {
      r1 = _mm_loadu_pd(&k2[i]);
      r2 = _mm_loadu_pd(&x[i]);
      _mm_storeu_pd(&b_x[i],
                    _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(0.0005), r1)));
    }
    b_x[20] = x[20] + 0.0005 * k2[20];
    deriv(b_x, cmd, k3);
    for (i = 0; i <= 18; i += 2) {
      r1 = _mm_loadu_pd(&k3[i]);
      r2 = _mm_loadu_pd(&x[i]);
      _mm_storeu_pd(&b_x[i],
                    _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(0.001), r1)));
    }
    b_x[20] = x[20] + 0.001 * k3[20];
    deriv(b_x, cmd, dv2);
    for (i = 0; i <= 18; i += 2) {
      r1 = _mm_loadu_pd(&k2[i]);
      r2 = _mm_loadu_pd(&k1[i]);
      r3 = _mm_loadu_pd(&k3[i]);
      r4 = _mm_loadu_pd(&dv2[i]);
      r5 = _mm_loadu_pd(&x[i]);
      _mm_storeu_pd(
          &x[i],
          _mm_add_pd(
              r5, _mm_mul_pd(
                      _mm_set1_pd(0.00016666666666666666),
                      _mm_add_pd(_mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r, r1)),
                                            _mm_mul_pd(r, r3)),
                                 r4))));
    }
    x[20] += 0.00016666666666666666 *
             (((k1[20] + 2.0 * k2[20]) + 2.0 * k3[20]) + dv2[20]);
    pN = 3.3121686421112381E-170;
    aN = fabs(x[6]);
    if (aN > 3.3121686421112381E-170) {
      V = 1.0;
      pN = aN;
    } else {
      t = aN / 3.3121686421112381E-170;
      V = t * t;
    }
    aN = fabs(x[7]);
    if (aN > pN) {
      t = pN / aN;
      V = V * t * t + 1.0;
      pN = aN;
    } else {
      t = aN / pN;
      V += t * t;
    }
    aN = fabs(x[8]);
    if (aN > pN) {
      t = pN / aN;
      V = V * t * t + 1.0;
      pN = aN;
    } else {
      t = aN / pN;
      V += t * t;
    }
    aN = fabs(x[9]);
    if (aN > pN) {
      t = pN / aN;
      V = V * t * t + 1.0;
      pN = aN;
    } else {
      t = aN / pN;
      V += t * t;
    }
    V = pN * sqrt(V);
    x[6] /= V;
    if ((x[14] >= 1100.0) || rtIsNaN(x[14])) {
      pN = 1100.0;
    } else {
      pN = x[14];
    }
    if (pN <= 0.0) {
      x[14] = 0.0;
    } else {
      x[14] = pN;
    }
    x[7] /= V;
    if ((x[15] >= 1100.0) || rtIsNaN(x[15])) {
      pN = 1100.0;
    } else {
      pN = x[15];
    }
    if (pN <= 0.0) {
      x[15] = 0.0;
    } else {
      x[15] = pN;
    }
    x[8] /= V;
    if ((x[16] >= 1100.0) || rtIsNaN(x[16])) {
      pN = 1100.0;
    } else {
      pN = x[16];
    }
    if (pN <= 0.0) {
      x[16] = 0.0;
    } else {
      x[16] = pN;
    }
    x[9] /= V;
    if ((x[17] >= 1100.0) || rtIsNaN(x[17])) {
      pN = 1100.0;
    } else {
      pN = x[17];
    }
    if (pN <= 0.0) {
      x[17] = 0.0;
    } else {
      x[17] = pN;
    }
  }
}

/*
 * File trailer for tilt_sim.c
 *
 * [EOF]
 */
