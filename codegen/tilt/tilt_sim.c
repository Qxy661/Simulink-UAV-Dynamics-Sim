/*
 * File: tilt_sim.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-29 21:15:18
 */

/* Include Files */
#include "tilt_sim.h"
#include "mod.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "tilt_sim_data.h"
#include "tilt_sim_initialize.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Function Declarations */
static void deriv(const double x[21], const double cmd[8], double xd[21]);

static double rt_atan2d_snf(double u0, double u1);

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
  double c_d_tmp[3];
  double c_x[3];
  double d_x[3];
  double wd[3];
  double D;
  double L;
  double R_tmp;
  double U1;
  double U4;
  double V;
  double Ve;
  double Xa_tmp;
  double absxk;
  double alpha;
  double b_R_tmp;
  double b_Xa_tmp;
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
  alpha = rt_atan2d_snf(x[5], x[3]);
  absxk = x[4] / Ve;
  if ((absxk >= 1.0) || rtIsNaN(absxk)) {
    absxk = 1.0;
  }
  if (absxk <= -1.0) {
    absxk = -1.0;
  }
  beta = asin(absxk);
  qbar = 0.6125 * (V * V);
  V = ((5.0 * alpha + 0.45) + 3.9 * x[11] * 0.34 / (2.0 * Ve)) + 0.4 * x[18];
  D = qbar * 1.6 * (0.045 * (V * V) + 0.025);
  L = qbar * 1.6 * V;
  /*  稳定性轴到机体轴 */
  Xa_tmp = sin(alpha);
  b_Xa_tmp = cos(alpha);
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
  absxk = (cmd[7] - x[13]) / 0.15;
  if ((absxk >= 0.52359877559829882) || rtIsNaN(absxk)) {
    absxk = 0.52359877559829882;
  }
  c_d_tmp[0] = b_d_tmp * U1;
  c_d_tmp[1] = 0.0 * U1;
  c_d_tmp[2] = -d_tmp * U1;
  c_x[0] = x[5] * x[11] - x[4] * x[12];
  c_x[1] = x[3] * x[12] - x[5] * x[10];
  c_x[2] = x[4] * x[10] - x[3] * x[11];
  scale = 0.0;
  for (i = 0; i < 3; i++) {
    V = x[i + 10];
    T[i + 1] = V;
    wd[i] = (a[i] * x[10] + a[i + 3] * x[11]) + a[i + 6] * x[12];
    scale += x[i + 7] * V;
  }
  V = qbar * 1.6 * 3.6;
  B[0] = (V * (((-0.09 * beta + -0.5 * x[10] * 3.6 / (2.0 * Ve)) +
                0.08 * x[12] * 3.6 / (2.0 * Ve)) +
               0.18 * x[19]) +
          (d_tmp * -y - 0.019230769230769232 * b_d_tmp * U4)) -
         (wd[2] * x[11] - wd[1] * x[12]);
  B[1] = (qbar * 1.6 * 0.34 *
              (((-0.8 * alpha + 0.05) + -12.0 * x[11] * 0.34 / (2.0 * Ve)) +
               -1.2 * x[18]) +
          d_tmp * (((b_x[0] + b_x[1]) + b_x[2]) + b_x[3])) -
         (wd[0] * x[12] - wd[2] * x[10]);
  B[2] = (V * (((0.06 * beta + -0.03 * x[10] * 3.6 / (2.0 * Ve)) +
                -0.12 * x[12] * 3.6 / (2.0 * Ve)) +
               0.09 * x[20]) +
          (b_d_tmp * -y + 0.019230769230769232 * d_tmp * U4)) -
         (wd[1] * x[10] - wd[0] * x[11]);
  wd[1] = B[1] - B[0] * 0.0;
  wd[2] = (B[2] - B[0] * 0.0) - wd[1] * 0.0;
  wd[2] /= 1.6;
  wd[0] = B[0] - wd[2] * 0.0;
  wd[1] -= wd[2] * 0.0;
  wd[1] /= 1.2;
  wd[0] -= wd[1] * 0.0;
  wd[0] /= 0.9;
  B[0] = -D * b_Xa_tmp + L * Xa_tmp;
  B[1] = qbar * 1.6 * (-0.6 * beta + 0.19 * x[20]);
  B[2] = -D * Xa_tmp - L * b_Xa_tmp;
  d_x[0] = x[8] * x[12] - x[9] * x[11];
  d_x[1] = x[9] * x[10] - x[7] * x[12];
  d_x[2] = x[7] * x[11] - x[8] * x[10];
  xd[6] = 0.5 * (x[6] * 0.0 - scale);
  for (i = 0; i < 3; i++) {
    xd[i] = (R[i] * x[3] + R[i + 3] * x[4]) + R[i + 6] * x[5];
    xd[i + 3] = ((B[i] + c_d_tmp[i]) / 8.0 - c_x[i]) +
                ((R[3 * i] * 0.0 + R[3 * i + 1] * 0.0) + R[3 * i + 2] * 9.81);
    xd[i + 7] = 0.5 * ((x[6] * T[i + 1] + 0.0 * x[i + 7]) + d_x[i]);
    xd[i + 10] = wd[i];
  }
  if (absxk <= -0.52359877559829882) {
    xd[13] = -0.52359877559829882;
  } else {
    xd[13] = absxk;
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
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int i;
  int i1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = atan2(i, i1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }
  return y;
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
  static const double dv3[3] = {0.0, 0.0, 9.81};
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r3;
  __m128d r4;
  __m128d r5;
  double b_x[21];
  double dv2[21];
  double k1[21];
  double k2[21];
  double k3[21];
  double x[21];
  double R[9];
  double cmd[8];
  double T[4];
  double Frot[3];
  double Vn[3];
  double a[3];
  double b_D[3];
  double ep[3];
  double r_v[3];
  double D;
  double V;
  double V_tmp;
  double aN;
  double att_idx_0;
  double att_idx_1;
  double att_idx_2;
  double b_k;
  double cb;
  double n;
  double ny;
  double pN;
  double sb;
  double st_ia_idx_0;
  double st_ia_idx_1;
  double st_ia_idx_2;
  double st_ip_idx_0;
  double st_ip_idx_1;
  double st_ip_idx_2;
  double t;
  double u1;
  int i;
  int j;
  int k;
  if (!isInitialized_tilt_sim) {
    tilt_sim_initialize();
  }
  memset(&X[0], 0, 94500U * sizeof(double));
  memcpy(&x[0], &dv[0], 21U * sizeof(double));
  memcpy(&cmd[0], &dv1[0], 8U * sizeof(double));
  st_ip_idx_0 = 0.0;
  st_ia_idx_0 = 0.0;
  st_ip_idx_1 = 0.0;
  st_ia_idx_1 = 0.0;
  st_ip_idx_2 = 0.0;
  st_ia_idx_2 = 0.0;
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
      /*  m/s，稳定转捩走廊上界（见 docs/tiltrotor.md） */
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
        V_tmp = 3.1415926535897931 * (t - 5.0) / 15.0;
        V = 5.0 * (1.0 - cos(V_tmp));
        n = sin(V_tmp);
        aN = 1.0471975511965976 * n;
        pN = 5.0 * ((t - 5.0) - 4.7746482927568605 * n);
      } else {
        /*  加速段结束后保持 10 m/s 巡航。任务只覆盖"悬停 → 转捩 → 巡航"， */
        /*  不做减速回悬停：减速段的姿态混合会削弱倾角需求，实测位置滞后 */
        /*  25 m 且 β 收回到 8° 后机体继续漂移，见 docs/tiltrotor.md 未解决项。
         */
        V = 10.0;
        aN = 0.0;
        pN = 10.0 * (t - 20.0) + 75.0;
      }
      r_v[0] = V;
      r_v[1] = 0.0;
      r_v[2] = 0.0;
      /* TILT_CASCADE  倾转四旋翼统一级联控制（空速调度 + 权限分配）。 */
      /*    x   : 21 维状态，前 13 维与四旋翼/固定翼一致，14 短舱角，15:18
       * 旋翼转速，19:21 舵面 */
      /*    ref : 结构体，字段 p（NED 位置）、v、a、psi、h、V */
      /*    cmd : [om(4); de; da; dr; betaCmd]，8 维 */
      /*  */
      /*    位置环先算期望加速度 a，再由推力需求 Tn=m*(a-g) 得到旋翼要补的力 */
      /*    Frot = R'*Tn - Fa。姿态指令按空速权重 k 在两支之间线性混合： */
      /*      k=0 四旋翼支：机体 -z 轴对准 Tn； */
      /*      k=1 固定翼支：俯仰维持配平迎角 + 高度/速度反馈，升力由机翼出。 */
      /*    硬性按约束 R*d(beta)=n 反解姿态在 cb≈0.3 处会跳变，实测把俯仰打到
       * -17°， */
      /*    机翼迎角反号，越低头越掉高，所以改成混合而不是解约束。 */
      /*  */
      /*    U1 用投影而不是模长：U1=max(d'*Frot,0)，d=[sin b;0;-cos b]。模长会把
       */
      /*    横向分量也算成推力，β=90° 时推力全给前向，转速顶到限幅收不回来。 */
      /* Q2DCM  四元数转旋转矩阵（机体系 -> NED）。 */
      /*    q = [w; x; y; z]，要求单位化；非单位输入按单位处理。 */
      /*    对应 ZYX 欧拉角：R = Rz(psi)*Ry(theta)*Rx(phi)。 */
      n = b_norm(&x[6]);
      if (n < 1.0E-12) {
        T[0] = 1.0;
        T[1] = 0.0;
        T[2] = 0.0;
        T[3] = 0.0;
      } else {
        r1 = _mm_loadu_pd(&x[6]);
        r2 = _mm_set1_pd(n);
        _mm_storeu_pd(&T[0], _mm_div_pd(r1, r2));
        r1 = _mm_loadu_pd(&x[8]);
        _mm_storeu_pd(&T[2], _mm_div_pd(r1, r2));
      }
      n = T[3] * T[3];
      t = T[2] * T[2];
      R[0] = 1.0 - 2.0 * (t + n);
      sb = T[1] * T[2];
      ny = T[0] * T[3];
      R[3] = 2.0 * (sb - ny);
      D = T[1] * T[3];
      cb = T[0] * T[2];
      R[6] = 2.0 * (D + cb);
      R[1] = 2.0 * (sb + ny);
      sb = T[1] * T[1];
      R[4] = 1.0 - 2.0 * (sb + n);
      n = T[2] * T[3];
      ny = T[0] * T[1];
      R[7] = 2.0 * (n - ny);
      R[2] = 2.0 * (D - cb);
      R[5] = 2.0 * (n + ny);
      R[8] = 1.0 - 2.0 * (sb + t);
      /* QUAT2EULER  四元数转 ZYX 欧拉角 [phi; theta; psi]，单位 rad。 */
      /*    输入四元数为机体系 -> NED，奇异点 theta = +-pi/2 处 theta 被截断。
       */
      n = b_norm(&x[6]);
      if (n < 1.0E-12) {
        att_idx_0 = 0.0;
        att_idx_1 = 0.0;
        att_idx_2 = 0.0;
      } else {
        r1 = _mm_loadu_pd(&x[6]);
        r2 = _mm_set1_pd(n);
        _mm_storeu_pd(&T[0], _mm_div_pd(r1, r2));
        r1 = _mm_loadu_pd(&x[8]);
        _mm_storeu_pd(&T[2], _mm_div_pd(r1, r2));
        u1 = 2.0 * (T[0] * T[2] - T[1] * T[3]);
        if ((u1 >= 1.0) || rtIsNaN(u1)) {
          u1 = 1.0;
        }
        n = T[2] * T[2];
        att_idx_0 = rt_atan2d_snf(2.0 * (T[0] * T[1] + T[2] * T[3]),
                                  1.0 - 2.0 * (T[1] * T[1] + n));
        if (u1 <= -1.0) {
          u1 = -1.0;
        }
        att_idx_1 = asin(u1);
        att_idx_2 = rt_atan2d_snf(2.0 * (T[0] * T[3] + T[1] * T[2]),
                                  1.0 - 2.0 * (n + T[3] * T[3]));
      }
      D = x[3];
      n = x[4];
      t = x[5];
      r1 = _mm_loadu_pd(&R[0]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(D));
      r2 = _mm_loadu_pd(&R[3]);
      r2 = _mm_mul_pd(r2, _mm_set1_pd(n));
      r1 = _mm_add_pd(r1, r2);
      r2 = _mm_loadu_pd(&R[6]);
      r2 = _mm_mul_pd(r2, _mm_set1_pd(t));
      r1 = _mm_add_pd(r1, r2);
      _mm_storeu_pd(&Vn[0], r1);
      Vn[2] = (R[2] * D + R[5] * n) + R[8] * t;
      V_tmp = c_norm(&x[3]);
      sb = 0.6125 * (V_tmp * V_tmp);
      /*  转捩调度用参考空速而不是实测空速：实测空速低于计划时，若按它算 k， */
      /*  k 会回落，短舱跟着往回收，机体在过渡段来回摆。用 ref.V 做开环调度， */
      /*  短舱按任务计划单调前倾，实测速度只作为内环的被控量。 */
      b_k = (V - 3.0) / 11.0;
      if (b_k <= 0.0) {
        b_k = 0.0;
      }
      /*  ---- 位置环 ---- */
      ep[0] = pN - x[0];
      D = st_ip_idx_0 + ep[0] * 0.01;
      if ((D >= 0.0) || rtIsNaN(D)) {
        D = 0.0;
      }
      if (D <= -0.0) {
        st_ip_idx_0 = -0.0;
      } else {
        st_ip_idx_0 = D;
      }
      D = st_ip_idx_1 + (0.0 - x[1]) * 0.01;
      if ((D >= 0.0) || rtIsNaN(D)) {
        D = 0.0;
      }
      if (D <= -0.0) {
        st_ip_idx_1 = -0.0;
      } else {
        st_ip_idx_1 = D;
      }
      D = st_ip_idx_2 + (-20.0 - x[2]) * 0.01;
      if ((D >= 1.2) || rtIsNaN(D)) {
        D = 1.2;
      }
      if (D <= -1.2) {
        st_ip_idx_2 = -1.2;
      } else {
        st_ip_idx_2 = D;
      }
      a[0] = ((ep[0] + 1.8 * (V - Vn[0])) + aN) + 0.0 * st_ip_idx_0;
      a[1] = ((0.0 - x[1]) + 1.8 * (0.0 - Vn[1])) + 0.0 * st_ip_idx_1;
      a[2] = (1.6 * (-20.0 - x[2]) + 2.2 * (0.0 - Vn[2])) + 0.8 * st_ip_idx_2;
      n = d_norm(&a[0]);
      if (n > 2.5) {
        r1 = _mm_loadu_pd(&a[0]);
        _mm_storeu_pd(&a[0], _mm_mul_pd(r1, _mm_set1_pd(2.5 / n)));
      }
      if ((a[2] >= 4.0) || rtIsNaN(a[2])) {
        u1 = 4.0;
      } else {
        u1 = a[2];
      }
      if (u1 <= -4.0) {
        a[2] = -4.0;
      } else {
        a[2] = u1;
      }
      /*  限速：位置误差大时位置环会一直给满加速度，速度一路超。超过参考两米每秒
       */
      /*  就把沿速度方向的加速度压成"减速"，让速度回到参考附近。 */
      n = c_norm(r_v);
      if (V_tmp > n + 2.0) {
        D = Vn[0] / V_tmp;
        Vn[0] = D;
        t = a[0] * D;
        D = Vn[1] / V_tmp;
        Vn[1] = D;
        t += a[1] * D;
        D = Vn[2] / V_tmp;
        Vn[2] = D;
        t += a[2] * D;
        n = -((V_tmp - n) - 2.0);
        if (t > n) {
          n -= t;
          r1 = _mm_loadu_pd(&Vn[0]);
          r2 = _mm_loadu_pd(&a[0]);
          _mm_storeu_pd(&a[0], _mm_add_pd(r2, _mm_mul_pd(r1, _mm_set1_pd(n))));
          a[2] += D * n;
        }
      }
      /*  ---- 需要的合力与旋翼份额 ---- */
      /*  NED，非重力合力（悬停时 [0;0;-mg]） */
      r1 = _mm_loadu_pd(&a[0]);
      _mm_storeu_pd(&a[0], _mm_mul_pd(_mm_set1_pd(8.0),
                                      _mm_sub_pd(r1, _mm_loadu_pd(&dv3[0]))));
      r1 = _mm_loadu_pd(&x[18]);
      _mm_storeu_pd(&T[1], r1);
      a[2] = 8.0 * (a[2] - 9.81);
      /* FW_AERO  固定翼气动力与力矩。 */
      /*    Vb  : 机体系速度 [u; v; w] (m/s) */
      /*    w   : 机体系角速度 [p; q; r] (rad/s) */
      /*    act : 执行机构 [throttle; de; da; dr] */
      /*          throttle 0..1，舵面 rad，正 de 为抬头方向的定义见 fw_aero 文档
       */
      /*    Fb  : 机体系力 (N)，包含推力、升力、阻力、侧力 */
      /*    Mb  : 机体力矩 (N*m) */
      /*  */
      /*    气动角：alpha = atan2(w, u)，beta = asin(v/V)。 */
      /*    升力沿稳定性 z 轴负方向（机腹向上为负），阻力沿 -x，再按 alpha
       * 转到机体轴。 */
      if (V_tmp >= 1.0) {
        n = V_tmp;
      } else {
        n = 1.0;
      }
      /*  低速时避免除零，只在悬停/起飞段生效 */
      cb = rt_atan2d_snf(x[5], x[3]);
      u1 = x[4] / n;
      if ((u1 >= 1.0) || rtIsNaN(u1)) {
        u1 = 1.0;
      }
      n = ((5.0 * cb + 0.45) + 3.9 * x[11] * 0.34 / (2.0 * n)) + 0.4 * T[1];
      D = sb * 1.6 * (0.045 * (n * n) + 0.025);
      n *= sb * 1.6;
      /*  稳定性轴到机体轴 */
      t = sin(cb);
      ny = cos(cb);
      b_D[0] = -D * ny + n * t;
      if (u1 <= -1.0) {
        u1 = -1.0;
      }
      b_D[1] = sb * 1.6 * (-0.6 * asin(u1) + 0.19 * x[20]);
      b_D[2] = -D * t - n * ny;
      D = a[0];
      n = a[1];
      t = a[2];
      for (i = 0; i < 3; i++) {
        Frot[i] =
            ((R[3 * i] * D + R[3 * i + 1] * n) + R[3 * i + 2] * t) - b_D[i];
      }
      if (c_norm(Frot) < 1.0E-6) {
        Frot[0] = 0.0;
        Frot[1] = 0.0;
        Frot[2] = -78.48;
      }
      /*  ---- 姿态指令：四旋翼支与固定翼支按 k 混合 ---- */
      if (sb >= 1.0) {
        pN = sb;
      } else {
        pN = 1.0;
      }
      u1 = (78.48 / (pN * 1.6) - 0.45) / 5.0;
      /*  悬停时不能用机翼配平迎角 */
      /*  四旋翼支：机体 -z 轴对准 Tn */
      n = c_norm(a);
      if (!(n >= 1.0E-9)) {
        n = 1.0E-9;
      }
      r1 = _mm_loadu_pd(&a[0]);
      _mm_storeu_pd(
          &a[0], _mm_div_pd(_mm_mul_pd(r1, _mm_set1_pd(-1.0)), _mm_set1_pd(n)));
      a[2] = -a[2] / n;
      Vn[0] = cos(att_idx_2);
      Vn[1] = sin(att_idx_2);
      ep[0] = a[1] * 0.0 - Vn[1] * a[2];
      ep[1] = Vn[0] * a[2] - a[0] * 0.0;
      ep[2] = a[0] * Vn[1] - Vn[0] * a[1];
      ny = c_norm(ep);
      if (ny < 1.0E-6) {
        ep[0] = a[1] * 0.0 - 0.0 * a[2];
        ep[1] = a[2] - a[0] * 0.0;
        ep[2] = a[0] * 0.0 - a[1];
        ny = c_norm(ep);
      }
      r1 = _mm_loadu_pd(&ep[0]);
      r1 = _mm_div_pd(r1, _mm_set1_pd(ny));
      _mm_storeu_pd(&ep[0], r1);
      _mm_storeu_pd(&R[3], r1);
      r1 = _mm_loadu_pd(&a[0]);
      _mm_storeu_pd(&R[6], r1);
      R[2] = ep[0] * a[1] - a[0] * ep[1];
      if ((R[2] >= 1.0) || rtIsNaN(R[2])) {
        sb = 1.0;
      } else {
        sb = R[2];
      }
      /*  固定翼支：配平迎角 + 高度与空速反馈 */
      if (u1 >= 0.3) {
        u1 = 0.3;
      }
      u1 = ((b_k * u1 + 0.03 * (20.0 - (-x[2]))) -
            0.04 * (V_tmp * sin(att_idx_1 - cb))) +
           0.03 * (V_tmp - V);
      if ((u1 >= 0.3) || rtIsNaN(u1)) {
        u1 = 0.3;
      }
      /* WRAP_PI  角度归一化到 (-pi, pi]。 */
      t = b_mod((0.0 - att_idx_2) + 3.1415926535897931);
      n = t - 3.1415926535897931;
      if (t - 3.1415926535897931 <= -3.1415926535897931) {
        n = 3.1415926535897931;
      }
      n *= 0.8;
      if ((n >= 0.45) || rtIsNaN(n)) {
        n = 0.45;
      }
      /*  ---- 姿态环 ---- */
      /* WRAP_PI  角度归一化到 (-pi, pi]。 */
      if (n <= -0.45) {
        n = -0.45;
      }
      Vn[0] = (((1.0 - b_k) * rt_atan2d_snf(ep[2] / ny, a[2]) + b_k * n) -
               att_idx_0) +
              3.1415926535897931;
      ep[0] = b_mod(Vn[0]) - 3.1415926535897931;
      if (sb <= -1.0) {
        sb = -1.0;
      }
      if (u1 <= -0.3) {
        u1 = -0.3;
      }
      ep[1] = b_mod((((1.0 - b_k) * -asin(sb) + b_k * u1) - att_idx_1) +
                    3.1415926535897931) -
              3.1415926535897931;
      ep[2] = t - 3.1415926535897931;
      if (ep[0] <= -3.1415926535897931) {
        ep[0] = 3.1415926535897931;
      }
      if (ep[1] <= -3.1415926535897931) {
        ep[1] = 3.1415926535897931;
      }
      if (t - 3.1415926535897931 <= -3.1415926535897931) {
        ep[2] = 3.1415926535897931;
      }
      /*  ---- 分配：旋翼份额与舵面份额 ---- */
      sb = sin(x[13]);
      cb = cos(x[13]);
      D = st_ia_idx_0 + ep[0] * 0.01;
      if ((D >= 0.0) || rtIsNaN(D)) {
        D = 0.0;
      }
      if (D <= -0.0) {
        D = -0.0;
      }
      st_ia_idx_0 = D;
      D = (6.0 * ep[0] + 0.0 * D) - x[10];
      if ((D >= 2.5) || rtIsNaN(D)) {
        D = 2.5;
      }
      if (D <= -2.5) {
        ep[0] = -2.5;
      } else {
        ep[0] = D;
      }
      D = st_ia_idx_1 + ep[1] * 0.01;
      if ((D >= 0.0) || rtIsNaN(D)) {
        D = 0.0;
      }
      if (D <= -0.0) {
        D = -0.0;
      }
      st_ia_idx_1 = D;
      D = (6.0 * ep[1] + 0.0 * D) - x[11];
      if ((D >= 2.5) || rtIsNaN(D)) {
        D = 2.5;
      }
      if (D <= -2.5) {
        ep[1] = -2.5;
      } else {
        ep[1] = D;
      }
      D = st_ia_idx_2 + ep[2] * 0.01;
      if ((D >= 0.35) || rtIsNaN(D)) {
        D = 0.35;
      }
      if (D <= -0.35) {
        D = -0.35;
      }
      st_ia_idx_2 = D;
      D = (3.0 * ep[2] + 0.6 * D) - 0.8 * x[12];
      if ((D >= 1.0) || rtIsNaN(D)) {
        D = 1.0;
      }
      if (D <= -1.0) {
        ep[2] = -1.0;
      } else {
        ep[2] = D;
      }
      n = (sb * Frot[0] + 0.0 * Frot[1]) + -cb * Frot[2];
      if ((n <= 0.0) || rtIsNaN(n)) {
        n = 0.0;
      }
      u1 = (1.0 - b_k) * c_norm(Frot) + b_k * n;
      if ((u1 >= 125.83999999999999) || rtIsNaN(u1)) {
        u1 = 125.83999999999999;
      }
      if (u1 <= 0.0) {
        ny = 0.0;
      } else {
        ny = u1;
      }
      n = (1.0 - b_k) * ep[0];
      t = (1.0 - b_k) * ep[2];
      D = (-sb * n + cb * t) / 0.019230769230769232;
      /* TILT_MIX  虚拟控制量 -> 四个旋翼转速。 */
      /*    解 [U1;U2;U3;U4] = M * [T1;T2;T3;T4] */
      /*        M = [ 1    1    1    1 ; */
      /*             -aY  -aY   aY   aY ; */
      /*              aX  -aX  -aX   aX ; */
      /*              1   -1    1   -1 ] */
      /*    逆矩阵按显式算式展开，便于在 Simulink 图表里内联。 */
      /*    sat 为 1 表示有旋翼推力被截断到 [0, wMax]（或出现过负推力需求）。 */
      t = 0.25 * (cb * n + sb * t) / 0.9;
      sb = 0.25 * ny - t;
      if (!(cb >= 0.2)) {
        cb = 0.2;
      }
      n = 0.25 * ((1.0 - b_k) * ep[1] / cb) / 0.4;
      T[0] = (sb + n) + 0.25 * D;
      T[1] = (sb - n) - 0.25 * D;
      sb = 0.25 * ny + t;
      T[2] = (sb - n) + 0.25 * D;
      T[3] = (sb + n) - 0.25 * D;
      n = pN * 1.6 * 3.6;
      u1 = (1.0 - (1.0 - b_k)) * ep[0] / (n * 0.18);
      sb = (1.0 - (1.0 - b_k)) * ep[1] / (pN * 1.6 * 0.34 * -1.2);
      n = (1.0 - (1.0 - b_k)) * ep[2] / (n * 0.09);
      if ((sb >= 0.43633231299858238) || rtIsNaN(sb)) {
        sb = 0.43633231299858238;
      }
      if ((u1 >= 0.3490658503988659) || rtIsNaN(u1)) {
        u1 = 0.3490658503988659;
      }
      if ((n >= 0.43633231299858238) || rtIsNaN(n)) {
        n = 0.43633231299858238;
      }
      /*  早期这里写 st.dbg = [a(:); phiD; thetaD; U1; k] 用于调参， */
      /*  代码生成不允许在结构体被读取后再加字段，已删除；需要诊断时临时加回来。
       */
      if (T[0] >= 0.0) {
        t = T[0];
      } else {
        t = 0.0;
      }
      D = sqrt(t / 2.6E-5);
      if (D <= 1100.0) {
        cmd[0] = D;
      } else {
        cmd[0] = 1100.0;
      }
      if (T[1] >= 0.0) {
        t = T[1];
      } else {
        t = 0.0;
      }
      D = sqrt(t / 2.6E-5);
      if (D <= 1100.0) {
        cmd[1] = D;
      } else {
        cmd[1] = 1100.0;
      }
      if (T[2] >= 0.0) {
        t = T[2];
      } else {
        t = 0.0;
      }
      D = sqrt(t / 2.6E-5);
      if (D <= 1100.0) {
        cmd[2] = D;
      } else {
        cmd[2] = 1100.0;
      }
      if (T[3] >= 0.0) {
        t = T[3];
      } else {
        t = 0.0;
      }
      D = sqrt(t / 2.6E-5);
      if (D <= 1100.0) {
        cmd[3] = D;
      } else {
        cmd[3] = 1100.0;
      }
      if (sb <= -0.43633231299858238) {
        cmd[4] = -0.43633231299858238;
      } else {
        cmd[4] = sb;
      }
      if (u1 <= -0.3490658503988659) {
        cmd[5] = -0.3490658503988659;
      } else {
        cmd[5] = u1;
      }
      if (n <= -0.43633231299858238) {
        cmd[6] = -0.43633231299858238;
      } else {
        cmd[6] = n;
      }
      cmd[7] = 1.5707963267948966 * b_k;
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
    D = b_norm(&x[6]);
    x[6] /= D;
    if ((x[14] >= 1100.0) || rtIsNaN(x[14])) {
      n = 1100.0;
    } else {
      n = x[14];
    }
    if (n <= 0.0) {
      x[14] = 0.0;
    } else {
      x[14] = n;
    }
    x[7] /= D;
    if ((x[15] >= 1100.0) || rtIsNaN(x[15])) {
      n = 1100.0;
    } else {
      n = x[15];
    }
    if (n <= 0.0) {
      x[15] = 0.0;
    } else {
      x[15] = n;
    }
    x[8] /= D;
    if ((x[16] >= 1100.0) || rtIsNaN(x[16])) {
      n = 1100.0;
    } else {
      n = x[16];
    }
    if (n <= 0.0) {
      x[16] = 0.0;
    } else {
      x[16] = n;
    }
    x[9] /= D;
    if ((x[17] >= 1100.0) || rtIsNaN(x[17])) {
      n = 1100.0;
    } else {
      n = x[17];
    }
    if (n <= 0.0) {
      x[17] = 0.0;
    } else {
      x[17] = n;
    }
  }
}

/*
 * File trailer for tilt_sim.c
 *
 * [EOF]
 */
