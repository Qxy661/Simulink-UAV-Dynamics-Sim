/*
 * File: quad_sim.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-29 21:03:40
 */

/* Include Files */
#include "quad_sim.h"
#include "norm.h"
#include "quad_sim_data.h"
#include "quad_sim_initialize.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Function Declarations */
static void deriv(const double x[13], const double w[4], double xd[13]);

static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : const double x[13]
 *                const double w[4]
 *                double xd[13]
 * Return Type  : void
 */
static void deriv(const double x[13], const double w[4], double xd[13])
{
  static const double a[9] = {0.0035, 0.0, 0.0, 0.0,   0.0035,
                              0.0,    0.0, 0.0, 0.0065};
  __m128d r;
  __m128d r1;
  double R[9];
  double T[4];
  double q[4];
  double B[3];
  double dv[3];
  double wd[3];
  double y[3];
  double R_tmp;
  double absxk;
  double b_R_tmp;
  double n;
  double scale;
  double t;
  double y_tmp;
  int i;
  /* QUAD_WRENCH  旋翼转速到机体力/力矩。 */
  /*    旋翼推力沿机体 -z，阻力取线性近似（仅低速段有效）。 */
  /*    w 为 4x1 转速 rad/s。 */
  r = _mm_loadu_pd(&w[0]);
  r1 = _mm_set1_pd(1.5E-5);
  _mm_storeu_pd(&T[0], _mm_mul_pd(r1, _mm_mul_pd(r, r)));
  r = _mm_loadu_pd(&w[2]);
  _mm_storeu_pd(&T[2], _mm_mul_pd(r1, _mm_mul_pd(r, r)));
  y_tmp = T[0] + T[1];
  /* RIGID_BODY  刚体六自由度方程（机体 FRD，世界 NED）。 */
  /*    状态 x = [pn;pe;pd; u;v;w; qw;qx;qy;qz; p;q;r]。 */
  /*    Fb/Mb 为机体系合力与合力矩（不含重力）。 */
  /* Q2DCM  四元数转旋转矩阵（机体系 -> NED）。 */
  /*    q = [w; x; y; z]，要求单位化；非单位输入按单位处理。 */
  /*    对应 ZYX 欧拉角：R = Rz(psi)*Ry(theta)*Rx(phi)。 */
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[6]);
  if (absxk > 3.3121686421112381E-170) {
    n = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n = t * t;
  }
  absxk = fabs(x[7]);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  absxk = fabs(x[8]);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  absxk = fabs(x[9]);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  n = scale * sqrt(n);
  if (n < 1.0E-12) {
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
  } else {
    r = _mm_set1_pd(n);
    _mm_storeu_pd(&q[0], _mm_div_pd(_mm_loadu_pd(&x[6]), r));
    _mm_storeu_pd(&q[2], _mm_div_pd(_mm_loadu_pd(&x[8]), r));
  }
  scale = q[3] * q[3];
  absxk = q[2] * q[2];
  R[0] = 1.0 - 2.0 * (absxk + scale);
  t = q[1] * q[2];
  n = q[0] * q[3];
  R[3] = 2.0 * (t - n);
  R_tmp = q[1] * q[3];
  b_R_tmp = q[0] * q[2];
  R[6] = 2.0 * (R_tmp + b_R_tmp);
  R[1] = 2.0 * (t + n);
  t = q[1] * q[1];
  R[4] = 1.0 - 2.0 * (t + scale);
  scale = q[2] * q[3];
  n = q[0] * q[1];
  R[7] = 2.0 * (scale - n);
  R[2] = 2.0 * (R_tmp - b_R_tmp);
  R[5] = 2.0 * (scale + n);
  R[8] = 1.0 - 2.0 * (t + absxk);
  /* QMUL  Hamilton 四元数乘法 a (x) b。 */
  /*    与 [w;x;y;z] 排列一致，用于 qdot = 0.5*q (x) [0;omega]。 */
  dv[0] = (0.0 - 0.08 * x[3]) / 1.5;
  dv[1] = (0.0 - 0.08 * x[4]) / 1.5;
  dv[2] = (-((y_tmp + T[2]) + T[3]) - 0.08 * x[5]) / 1.5;
  absxk = 0.0;
  for (i = 0; i < 3; i++) {
    scale = x[i + 10];
    q[i + 1] = scale;
    wd[i] = (a[i] * x[10] + a[i + 3] * x[11]) + a[i + 6] * x[12];
    absxk += x[i + 7] * scale;
  }
  B[0] = (-0.15909902576697318 * y_tmp + 0.15909902576697318 * (T[2] + T[3])) -
         (wd[2] * x[11] - wd[1] * x[12]);
  scale = T[0] - T[1];
  B[1] = 0.15909902576697318 * ((scale - T[2]) + T[3]) -
         (wd[0] * x[12] - wd[2] * x[10]);
  B[2] = 0.013333333333333332 * ((scale + T[2]) - T[3]) -
         (wd[1] * x[10] - wd[0] * x[11]);
  wd[1] = B[1] - B[0] * 0.0;
  wd[2] = (B[2] - B[0] * 0.0) - wd[1] * 0.0;
  wd[2] /= 0.0065;
  wd[0] = B[0] - wd[2] * 0.0;
  wd[1] -= wd[2] * 0.0;
  wd[1] /= 0.0035;
  wd[0] -= wd[1] * 0.0;
  wd[0] /= 0.0035;
  B[0] = x[5] * x[11] - x[4] * x[12];
  B[1] = x[3] * x[12] - x[5] * x[10];
  B[2] = x[4] * x[10] - x[3] * x[11];
  y[0] = q[3] * x[8] - q[2] * x[9];
  y[1] = q[1] * x[9] - q[3] * x[7];
  y[2] = q[2] * x[7] - q[1] * x[8];
  xd[6] = 0.5 * (x[6] * 0.0 - absxk);
  for (i = 0; i < 3; i++) {
    xd[i] = (R[i] * x[3] + R[i + 3] * x[4]) + R[i + 6] * x[5];
    xd[i + 3] = (dv[i] - B[i]) +
                ((R[3 * i] * 0.0 + R[3 * i + 1] * 0.0) + R[3 * i + 2] * 9.81);
    xd[i + 7] = 0.5 * ((x[6] * x[i + 10] + 0.0 * x[i + 7]) + y[i]);
    xd[i + 10] = wd[i];
  }
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
 * QUAD_SIM  四旋翼闭环仿真主循环（C 代码生成入口）。
 *    X = uav.codegen.quad_sim(pTarget, psiTarget)
 *    与 uav.sim.run 的数值完全一致：RK4 @1 ms、控制器 100 Hz、
 *    电机一阶模型精确离散、日志记录积分前的状态。
 *    pTarget 为 3x1 NED 位置目标（t = 1 s 阶跃），psiTarget 为航向。
 *    返回 1401x13 状态序列，采样间隔 10 ms，总时长 14 s。
 *
 *
 * Arguments    : const double pTarget[3]
 *                double psiTarget
 *                double X[18200]
 * Return Type  : void
 */
void quad_sim(const double pTarget[3], double psiTarget, double X[18200])
{
  static const double b_a[16] = {16666.666666666668,
                                 16666.666666666664,
                                 16666.666666666664,
                                 16666.666666666668,
                                 -104756.56017578482,
                                 -104756.56017578482,
                                 104756.56017578482,
                                 104756.56017578482,
                                 104756.56017578482,
                                 -104756.56017578482,
                                 -104756.56017578482,
                                 104756.56017578482,
                                 1.25E+6,
                                 -1.25E+6,
                                 1.25E+6,
                                 -1.25E+6};
  static const double dv1[3] = {1.6, 1.6, 2.5};
  static const double dv2[3] = {2.2, 2.2, 3.0};
  static const double dv3[3] = {0.0, 0.0, 1.2};
  static const signed char iv[13] = {0, 0, -5, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r3;
  __m128d r4;
  __m128d r5;
  double b_x[13];
  double dv[13];
  double k1[13];
  double k2[13];
  double k3[13];
  double x[13];
  double Rd[9];
  double q[4];
  double w[4];
  double w2[4];
  double a[3];
  double ep[3];
  double r_p[3];
  double st_ip[3];
  double yb[3];
  double Rd_tmp;
  double absxk;
  double absxk_tmp;
  double b_Rd_tmp;
  double b_absxk_tmp;
  double c_absxk_tmp;
  double d;
  double d_absxk_tmp;
  double n;
  double scale;
  double st_ia_idx_0;
  double st_ia_idx_1;
  double st_ia_idx_2;
  double t;
  int b_i;
  int i;
  int j;
  int k;
  boolean_T exitg1;
  boolean_T rEQ0;
  if (!isInitialized_quad_sim) {
    quad_sim_initialize();
  }
  /*  0..13.99 s，每 10 ms 一个样本 */
  memset(&X[0], 0, 18200U * sizeof(double));
  for (i = 0; i < 13; i++) {
    x[i] = iv[i];
  }
  w[0] = 495.22722057657535;
  w[1] = 495.22722057657535;
  w[2] = 495.22722057657535;
  w[3] = 495.22722057657535;
  st_ip[0] = 0.0;
  st_ia_idx_0 = 0.0;
  st_ip[1] = 0.0;
  st_ia_idx_1 = 0.0;
  st_ip[2] = 0.0;
  st_ia_idx_2 = 0.0;
  w2[0] = 495.22722057657535;
  w2[1] = 495.22722057657535;
  w2[2] = 495.22722057657535;
  w2[3] = 495.22722057657535;
  j = -1;
  r = _mm_set1_pd(2.0);
  for (k = 0; k < 14000; k++) {
    if (k == 0) {
      b_i = 0;
    } else {
      b_i = (int)fmod(((double)k + 1.0) - 1.0, 10.0);
    }
    if (b_i == 0) {
      /* STEP  位置阶跃参考，t0 之前保持 p0，之后保持 p1。 */
      /*    p0/p1 为 NED 列向量；速度与加速度参考为零。 */
      if ((((double)k + 1.0) - 1.0) * 0.001 < 1.0) {
        /* HOVER  定点悬停参考。 */
        /*    pd 为 NED 高度（向下为正），5 m 高度写作 -5。 */
        r_p[0] = 0.0;
        r_p[1] = 0.0;
        r_p[2] = -5.0;
      } else {
        /* HOVER  定点悬停参考。 */
        /*    pd 为 NED 高度（向下为正），5 m 高度写作 -5。 */
        r_p[0] = pTarget[0];
        r_p[1] = pTarget[1];
        r_p[2] = pTarget[2];
      }
      /* CASCADE  位置-姿态级联控制器。 */
      /*    位置环在 NED 下算期望加速度，再反解推力方向与期望姿态； */
      /*    姿态环用欧拉角误差 + 机体角速度阻尼，输出机体力矩。 */
      /*    st 为控制器状态（积分项），由调用方持有，避免全局变量。 */
      /* Q2DCM  四元数转旋转矩阵（机体系 -> NED）。 */
      /*    q = [w; x; y; z]，要求单位化；非单位输入按单位处理。 */
      /*    对应 ZYX 欧拉角：R = Rz(psi)*Ry(theta)*Rx(phi)。 */
      scale = 3.3121686421112381E-170;
      absxk_tmp = fabs(x[6]);
      if (absxk_tmp > 3.3121686421112381E-170) {
        n = 1.0;
        scale = absxk_tmp;
      } else {
        t = absxk_tmp / 3.3121686421112381E-170;
        n = t * t;
      }
      b_absxk_tmp = fabs(x[7]);
      if (b_absxk_tmp > scale) {
        t = scale / b_absxk_tmp;
        n = n * t * t + 1.0;
        scale = b_absxk_tmp;
      } else {
        t = b_absxk_tmp / scale;
        n += t * t;
      }
      c_absxk_tmp = fabs(x[8]);
      if (c_absxk_tmp > scale) {
        t = scale / c_absxk_tmp;
        n = n * t * t + 1.0;
        scale = c_absxk_tmp;
      } else {
        t = c_absxk_tmp / scale;
        n += t * t;
      }
      d_absxk_tmp = fabs(x[9]);
      if (d_absxk_tmp > scale) {
        t = scale / d_absxk_tmp;
        n = n * t * t + 1.0;
        scale = d_absxk_tmp;
      } else {
        t = d_absxk_tmp / scale;
        n += t * t;
      }
      n = scale * sqrt(n);
      if (n < 1.0E-12) {
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
      } else {
        r1 = _mm_loadu_pd(&x[6]);
        r2 = _mm_set1_pd(n);
        _mm_storeu_pd(&q[0], _mm_div_pd(r1, r2));
        r1 = _mm_loadu_pd(&x[8]);
        _mm_storeu_pd(&q[2], _mm_div_pd(r1, r2));
      }
      /*  位置环 */
      scale = q[3] * q[3];
      absxk = q[2] * q[2];
      Rd[0] = 1.0 - 2.0 * (absxk + scale);
      n = q[1] * q[2];
      t = q[0] * q[3];
      Rd[3] = 2.0 * (n - t);
      Rd_tmp = q[1] * q[3];
      b_Rd_tmp = q[0] * q[2];
      Rd[6] = 2.0 * (Rd_tmp + b_Rd_tmp);
      Rd[1] = 2.0 * (n + t);
      n = q[1] * q[1];
      Rd[4] = 1.0 - 2.0 * (n + scale);
      scale = q[2] * q[3];
      t = q[0] * q[1];
      Rd[7] = 2.0 * (scale - t);
      Rd[2] = 2.0 * (Rd_tmp - b_Rd_tmp);
      Rd[5] = 2.0 * (scale + t);
      Rd[8] = 1.0 - 2.0 * (n + absxk);
      r1 = _mm_loadu_pd(&r_p[0]);
      r2 = _mm_loadu_pd(&x[0]);
      r1 = _mm_sub_pd(r1, r2);
      _mm_storeu_pd(&ep[0], r1);
      r2 = _mm_loadu_pd(&dv1[0]);
      r1 = _mm_mul_pd(r2, r1);
      r2 = _mm_loadu_pd(&Rd[0]);
      r2 = _mm_mul_pd(r2, _mm_set1_pd(x[3]));
      r3 = _mm_loadu_pd(&Rd[3]);
      r3 = _mm_mul_pd(r3, _mm_set1_pd(x[4]));
      r2 = _mm_add_pd(r2, r3);
      r3 = _mm_loadu_pd(&Rd[6]);
      r3 = _mm_mul_pd(r3, _mm_set1_pd(x[5]));
      r2 = _mm_add_pd(r2, r3);
      r2 = _mm_mul_pd(r2, _mm_set1_pd(-1.0));
      r3 = _mm_loadu_pd(&dv2[0]);
      r2 = _mm_mul_pd(r3, r2);
      r1 = _mm_add_pd(r1, r2);
      r2 = _mm_loadu_pd(&dv3[0]);
      r3 = _mm_loadu_pd(&st_ip[0]);
      r2 = _mm_mul_pd(r2, r3);
      r1 = _mm_add_pd(r1, r2);
      _mm_storeu_pd(&a[0], r1);
      d = r_p[2] - x[2];
      ep[2] = d;
      a[2] = (dv1[2] * d +
              dv2[2] * -((Rd[2] * x[3] + x[4] * Rd[5]) + x[5] * Rd[8])) +
             dv3[2] * st_ip[2];
      /*  倾角限幅：只压水平分量，保留高度通道 */
      scale = 3.3121686421112381E-170;
      absxk = fabs(a[0]);
      if (absxk > 3.3121686421112381E-170) {
        d = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        d = t * t;
      }
      absxk = fabs(a[1]);
      if (absxk > scale) {
        t = scale / absxk;
        d = d * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        d += t * t;
      }
      d = scale * sqrt(d);
      if (d > 6.8690359498372526) {
        r1 = _mm_loadu_pd(&a[0]);
        _mm_storeu_pd(&a[0],
                      _mm_mul_pd(r1, _mm_set1_pd(6.8690359498372526 / d)));
      }
      /*  推力方向：机体 -z 轴指向 (g_n - a)，即期望机体 z 轴为 z */
      d = st_ip[0] + ep[0] * 0.01;
      if (!(d >= 0.0)) {
        d = 0.0;
      }
      if (d <= 0.0) {
        st_ip[0] = d;
      } else {
        st_ip[0] = 0.0;
      }
      a[0] = 0.0 - a[0];
      d = st_ip[1] + ep[1] * 0.01;
      if (!(d >= 0.0)) {
        d = 0.0;
      }
      if (d <= 0.0) {
        st_ip[1] = d;
      } else {
        st_ip[1] = 0.0;
      }
      a[1] = 0.0 - a[1];
      d = st_ip[2] + ep[2] * 0.01;
      if (!(d >= -1.0)) {
        d = -1.0;
      }
      if (d <= 1.0) {
        st_ip[2] = d;
      } else {
        st_ip[2] = 1.0;
      }
      a[2] = 9.81 - a[2];
      n = b_norm(a);
      if (n < 1.0E-6) {
        a[0] = 0.0;
        a[1] = 0.0;
        a[2] = 1.0;
        n = 1.0;
      }
      r1 = _mm_loadu_pd(&a[0]);
      _mm_storeu_pd(&a[0], _mm_div_pd(r1, _mm_set1_pd(n)));
      a[2] /= n;
      b_Rd_tmp = 1.5 * n;
      if (!(b_Rd_tmp >= 0.0)) {
        b_Rd_tmp = 0.0;
      }
      /*  由 z 轴与航向构造期望姿态，避免大航向下欧拉角反解耦合 */
      ep[0] = cos(psiTarget);
      ep[1] = sin(psiTarget);
      yb[0] = a[1] * 0.0 - ep[1] * a[2];
      yb[1] = ep[0] * a[2] - a[0] * 0.0;
      yb[2] = a[0] * ep[1] - ep[0] * a[1];
      scale = b_norm(yb);
      if (scale < 1.0E-6) {
        yb[0] = a[1] * 0.0 - a[2] * 0.0;
        yb[1] = a[2] - a[0] * 0.0;
        yb[2] = a[0] * 0.0 - a[1];
        scale = b_norm(yb);
      }
      r1 = _mm_loadu_pd(&yb[0]);
      r1 = _mm_div_pd(r1, _mm_set1_pd(scale));
      _mm_storeu_pd(&yb[0], r1);
      _mm_storeu_pd(&Rd[3], r1);
      r1 = _mm_loadu_pd(&a[0]);
      _mm_storeu_pd(&Rd[6], r1);
      d = yb[2] / scale;
      Rd[0] = yb[1] * a[2] - a[1] * d;
      Rd[1] = a[0] * d - yb[0] * a[2];
      Rd[2] = yb[0] * a[1] - a[0] * yb[1];
      if ((Rd[2] >= 1.0) || rtIsNaN(Rd[2])) {
        Rd_tmp = 1.0;
      } else {
        Rd_tmp = Rd[2];
      }
      /*  姿态环 */
      /* QUAT2EULER  四元数转 ZYX 欧拉角 [phi; theta; psi]，单位 rad。 */
      /*    输入四元数为机体系 -> NED，奇异点 theta = +-pi/2 处 theta 被截断。
       */
      scale = 3.3121686421112381E-170;
      if (absxk_tmp > 3.3121686421112381E-170) {
        n = 1.0;
        scale = absxk_tmp;
      } else {
        t = absxk_tmp / 3.3121686421112381E-170;
        n = t * t;
      }
      if (b_absxk_tmp > scale) {
        t = scale / b_absxk_tmp;
        n = n * t * t + 1.0;
        scale = b_absxk_tmp;
      } else {
        t = b_absxk_tmp / scale;
        n += t * t;
      }
      if (c_absxk_tmp > scale) {
        t = scale / c_absxk_tmp;
        n = n * t * t + 1.0;
        scale = c_absxk_tmp;
      } else {
        t = c_absxk_tmp / scale;
        n += t * t;
      }
      if (d_absxk_tmp > scale) {
        t = scale / d_absxk_tmp;
        n = n * t * t + 1.0;
        scale = d_absxk_tmp;
      } else {
        t = d_absxk_tmp / scale;
        n += t * t;
      }
      n = scale * sqrt(n);
      if (n < 1.0E-12) {
        ep[0] = 0.0;
        ep[1] = 0.0;
        ep[2] = 0.0;
      } else {
        r1 = _mm_loadu_pd(&x[6]);
        r2 = _mm_set1_pd(n);
        _mm_storeu_pd(&q[0], _mm_div_pd(r1, r2));
        r1 = _mm_loadu_pd(&x[8]);
        _mm_storeu_pd(&q[2], _mm_div_pd(r1, r2));
        scale = 2.0 * (q[0] * q[2] - q[1] * q[3]);
        if ((scale >= 1.0) || rtIsNaN(scale)) {
          scale = 1.0;
        }
        absxk = q[2] * q[2];
        ep[0] = rt_atan2d_snf(2.0 * (q[0] * q[1] + q[2] * q[3]),
                              1.0 - 2.0 * (q[1] * q[1] + absxk));
        if (scale <= -1.0) {
          scale = -1.0;
        }
        ep[1] = asin(scale);
        ep[2] = rt_atan2d_snf(2.0 * (q[0] * q[3] + q[1] * q[2]),
                              1.0 - 2.0 * (absxk + q[3] * q[3]));
      }
      /* WRAP_PI  角度归一化到 (-pi, pi]。 */
      yb[0] = (rt_atan2d_snf(d, a[2]) - ep[0]) + 3.1415926535897931;
      if (Rd_tmp <= -1.0) {
        Rd_tmp = -1.0;
      }
      yb[1] = (-asin(Rd_tmp) - ep[1]) + 3.1415926535897931;
      yb[2] = (rt_atan2d_snf(Rd[1], Rd[0]) - ep[2]) + 3.1415926535897931;
      for (i = 0; i < 3; i++) {
        d = yb[i];
        if (rtIsNaN(d) || rtIsInf(d)) {
          n = rtNaN;
        } else if (d == 0.0) {
          n = 0.0;
        } else {
          n = fmod(d, 6.2831853071795862);
          rEQ0 = (n == 0.0);
          if (!rEQ0) {
            scale = fabs(d / 6.2831853071795862);
            rEQ0 = !(fabs(scale - floor(scale + 0.5)) >
                     2.2204460492503131E-16 * scale);
          }
          if (rEQ0) {
            n = 0.0;
          } else if (d < 0.0) {
            n += 6.2831853071795862;
          }
        }
        a[i] = n - 3.1415926535897931;
      }
      if (a[0] <= -3.1415926535897931) {
        a[0] = 3.1415926535897931;
      }
      if (a[1] <= -3.1415926535897931) {
        a[1] = 3.1415926535897931;
      }
      if (a[2] <= -3.1415926535897931) {
        a[2] = 3.1415926535897931;
      }
      /* MIXER  力/力矩指令分配到四个旋翼。 */
      /*    输入 TM = [T; Mx; My; Mz]，输出目标转速 wcmd 与饱和标志。 */
      /*    先解 w^2，再对负值与超限做截断；截断后的真实力/力矩由 quad_wrench
       * 重新计算。 */
      d = st_ia_idx_0 + a[0] * 0.01;
      if (!(d >= -0.2)) {
        d = -0.2;
      }
      if (!(d <= 0.2)) {
        d = 0.2;
      }
      st_ia_idx_0 = d;
      d = (6.0 * a[0] + 0.0 * d) - 0.35 * x[10];
      if (!(d >= -3.0)) {
        d = -3.0;
      }
      if (d <= 3.0) {
        w2[1] = d;
      } else {
        w2[1] = 3.0;
      }
      d = st_ia_idx_1 + a[1] * 0.01;
      if (!(d >= -0.2)) {
        d = -0.2;
      }
      if (!(d <= 0.2)) {
        d = 0.2;
      }
      st_ia_idx_1 = d;
      d = (6.0 * a[1] + 0.0 * d) - 0.35 * x[11];
      if (!(d >= -3.0)) {
        d = -3.0;
      }
      if (d <= 3.0) {
        w2[2] = d;
      } else {
        w2[2] = 3.0;
      }
      d = st_ia_idx_2 + a[2] * 0.01;
      if (!(d >= -0.2)) {
        d = -0.2;
      }
      if (!(d <= 0.2)) {
        d = 0.2;
      }
      st_ia_idx_2 = d;
      d = (3.0 * a[2] + 0.15 * d) - 0.3 * x[12];
      if (!(d >= -1.5)) {
        d = -1.5;
      }
      if (!(d <= 1.5)) {
        d = 1.5;
      }
      if (b_Rd_tmp <= 40.0) {
        n = b_Rd_tmp;
      } else {
        n = 40.0;
      }
      scale = w2[1];
      absxk = w2[2];
      for (i = 0; i <= 2; i += 2) {
        r1 = _mm_loadu_pd(&b_a[i]);
        r1 = _mm_mul_pd(r1, _mm_set1_pd(n));
        r2 = _mm_loadu_pd(&b_a[i + 4]);
        r2 = _mm_mul_pd(r2, _mm_set1_pd(scale));
        r1 = _mm_add_pd(r1, r2);
        r2 = _mm_loadu_pd(&b_a[i + 8]);
        r2 = _mm_mul_pd(r2, _mm_set1_pd(absxk));
        r1 = _mm_add_pd(r1, r2);
        r2 = _mm_loadu_pd(&b_a[i + 12]);
        r2 = _mm_mul_pd(r2, _mm_set1_pd(d));
        r1 = _mm_add_pd(r1, r2);
        _mm_storeu_pd(&q[i], r1);
      }
      if (q[0] >= 0.0) {
        d = q[0];
      } else {
        d = 0.0;
      }
      w2[0] = sqrt(d);
      if (q[1] >= 0.0) {
        d = q[1];
      } else {
        d = 0.0;
      }
      w2[1] = sqrt(d);
      if (q[2] >= 0.0) {
        d = q[2];
      } else {
        d = 0.0;
      }
      w2[2] = sqrt(d);
      if (q[3] >= 0.0) {
        d = q[3];
      } else {
        d = 0.0;
      }
      w2[3] = sqrt(d);
      rEQ0 = false;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 4)) {
        if (w2[i] > 1200.0) {
          rEQ0 = true;
          exitg1 = true;
        } else {
          i++;
        }
      }
      if (rEQ0) {
        if (w2[0] > 1200.0) {
          w2[0] = 1200.0;
        }
        if (w2[1] > 1200.0) {
          w2[1] = 1200.0;
        }
        if (w2[2] > 1200.0) {
          w2[2] = 1200.0;
        }
        if (w2[3] > 1200.0) {
          w2[3] = 1200.0;
        }
      }
    }
    /* MOTOR_STEP  一阶电机模型的精确离散更新。 */
    /*    tau*wdot + w = wcmd 的解析解，避免小时间常数下的显式欧拉不稳定。 */
    d = w2[0] + (w[0] - w2[0]) * 0.951229424500714;
    if (!(d >= 0.0)) {
      d = 0.0;
    }
    if (d <= 1200.0) {
      w[0] = d;
    } else {
      w[0] = 1200.0;
    }
    d = w2[1] + (w[1] - w2[1]) * 0.951229424500714;
    if (!(d >= 0.0)) {
      d = 0.0;
    }
    if (d <= 1200.0) {
      w[1] = d;
    } else {
      w[1] = 1200.0;
    }
    d = w2[2] + (w[2] - w2[2]) * 0.951229424500714;
    if (!(d >= 0.0)) {
      d = 0.0;
    }
    if (d <= 1200.0) {
      w[2] = d;
    } else {
      w[2] = 1200.0;
    }
    d = w2[3] + (w[3] - w2[3]) * 0.951229424500714;
    if (!(d >= 0.0)) {
      d = 0.0;
    }
    if (d <= 1200.0) {
      w[3] = d;
    } else {
      w[3] = 1200.0;
    }
    deriv(x, w, k1);
    for (i = 0; i <= 10; i += 2) {
      r1 = _mm_loadu_pd(&k1[i]);
      r2 = _mm_loadu_pd(&x[i]);
      _mm_storeu_pd(&b_x[i],
                    _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(0.0005), r1)));
    }
    b_x[12] = x[12] + 0.0005 * k1[12];
    deriv(b_x, w, k2);
    for (i = 0; i <= 10; i += 2) {
      r1 = _mm_loadu_pd(&k2[i]);
      r2 = _mm_loadu_pd(&x[i]);
      _mm_storeu_pd(&b_x[i],
                    _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(0.0005), r1)));
    }
    b_x[12] = x[12] + 0.0005 * k2[12];
    deriv(b_x, w, k3);
    if (b_i == 0) {
      j++;
      for (b_i = 0; b_i < 13; b_i++) {
        X[j + 1400 * b_i] = x[b_i];
      }
    }
    for (b_i = 0; b_i <= 10; b_i += 2) {
      r1 = _mm_loadu_pd(&k3[b_i]);
      r2 = _mm_loadu_pd(&x[b_i]);
      _mm_storeu_pd(&b_x[b_i],
                    _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(0.001), r1)));
    }
    b_x[12] = x[12] + 0.001 * k3[12];
    deriv(b_x, w, dv);
    for (b_i = 0; b_i <= 10; b_i += 2) {
      r1 = _mm_loadu_pd(&k2[b_i]);
      r2 = _mm_loadu_pd(&k1[b_i]);
      r3 = _mm_loadu_pd(&k3[b_i]);
      r4 = _mm_loadu_pd(&dv[b_i]);
      r5 = _mm_loadu_pd(&x[b_i]);
      _mm_storeu_pd(
          &x[b_i],
          _mm_add_pd(
              r5, _mm_mul_pd(
                      _mm_set1_pd(0.00016666666666666666),
                      _mm_add_pd(_mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r, r1)),
                                            _mm_mul_pd(r, r3)),
                                 r4))));
    }
    x[12] += 0.00016666666666666666 *
             (((k1[12] + 2.0 * k2[12]) + 2.0 * k3[12]) + dv[12]);
    scale = 3.3121686421112381E-170;
    absxk = fabs(x[6]);
    if (absxk > 3.3121686421112381E-170) {
      n = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      n = t * t;
    }
    absxk = fabs(x[7]);
    if (absxk > scale) {
      t = scale / absxk;
      n = n * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      n += t * t;
    }
    absxk = fabs(x[8]);
    if (absxk > scale) {
      t = scale / absxk;
      n = n * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      n += t * t;
    }
    absxk = fabs(x[9]);
    if (absxk > scale) {
      t = scale / absxk;
      n = n * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      n += t * t;
    }
    n = scale * sqrt(n);
    r1 = _mm_loadu_pd(&x[6]);
    r2 = _mm_set1_pd(n);
    _mm_storeu_pd(&x[6], _mm_div_pd(r1, r2));
    r1 = _mm_loadu_pd(&x[8]);
    _mm_storeu_pd(&x[8], _mm_div_pd(r1, r2));
  }
}

/*
 * File trailer for quad_sim.c
 *
 * [EOF]
 */
