/*
 * File: fw_sim.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-31 05:20:23
 */

/* Include Files */
#include "fw_sim.h"
#include "fw_sim_data.h"
#include "fw_sim_initialize.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Function Declarations */
static void deriv(const double x[13], const double act[4], double xd[13]);

static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : const double x[13]
 *                const double act[4]
 *                double xd[13]
 * Return Type  : void
 */
static void deriv(const double x[13], const double act[4], double xd[13])
{
  static const double a[9] = {0.45, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.85};
  __m128d r;
  double R[9];
  double q[4];
  double B[3];
  double Vb[3];
  double b_q[3];
  double b_x[3];
  double CL;
  double D;
  double L;
  double R_tmp;
  double V;
  double Ve;
  double Xa_tmp;
  double absxk;
  double alpha;
  double b_Xa_tmp;
  double beta;
  double qbar;
  double scale;
  double t;
  int i;
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
  CL = x[5] / Ve;
  if ((CL >= 0.34) || rtIsNaN(CL)) {
    CL = 0.34;
  }
  if (CL <= -0.34) {
    CL = -0.34;
  }
  alpha = asin(CL);
  CL = x[4] / Ve;
  if ((CL >= 0.34) || rtIsNaN(CL)) {
    CL = 0.34;
  }
  if (CL <= -0.34) {
    CL = -0.34;
  }
  beta = asin(CL);
  qbar = 0.6125 * (V * V);
  CL = ((5.0 * alpha + 0.25) + 3.9 * x[11] * 0.22 / (2.0 * Ve)) + 0.4 * act[1];
  D = qbar * 0.55 * (0.045 * (CL * CL) + 0.025);
  L = qbar * 0.55 * CL;
  /*  稳定性轴到机体轴 */
  Xa_tmp = sin(alpha);
  b_Xa_tmp = cos(alpha);
  /* RIGID_BODY  刚体六自由度方程（机体 FRD，世界 NED）。 */
  /*    状态 x = [pn;pe;pd; u;v;w; qw;qx;qy;qz; p;q;r]。 */
  /*    Fb/Mb 为机体系合力与合力矩（不含重力）。 */
  /* Q2DCM  四元数转旋转矩阵（机体系 -> NED）。 */
  /*    q = [w; x; y; z]，要求单位化；非单位输入按单位处理。 */
  /*    对应 ZYX 欧拉角：R = Rz(psi)*Ry(theta)*Rx(phi)。 */
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[6]);
  if (absxk > 3.3121686421112381E-170) {
    CL = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    CL = t * t;
  }
  absxk = fabs(x[7]);
  if (absxk > scale) {
    t = scale / absxk;
    CL = CL * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    CL += t * t;
  }
  absxk = fabs(x[8]);
  if (absxk > scale) {
    t = scale / absxk;
    CL = CL * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    CL += t * t;
  }
  absxk = fabs(x[9]);
  if (absxk > scale) {
    t = scale / absxk;
    CL = CL * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    CL += t * t;
  }
  CL = scale * sqrt(CL);
  if (CL < 1.0E-12) {
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
  } else {
    r = _mm_set1_pd(CL);
    _mm_storeu_pd(&q[0], _mm_div_pd(_mm_loadu_pd(&x[6]), r));
    _mm_storeu_pd(&q[2], _mm_div_pd(_mm_loadu_pd(&x[8]), r));
  }
  CL = q[3] * q[3];
  V = q[2] * q[2];
  R[0] = 1.0 - 2.0 * (V + CL);
  scale = q[1] * q[2];
  absxk = q[0] * q[3];
  R[3] = 2.0 * (scale - absxk);
  t = q[1] * q[3];
  R_tmp = q[0] * q[2];
  R[6] = 2.0 * (t + R_tmp);
  R[1] = 2.0 * (scale + absxk);
  scale = q[1] * q[1];
  R[4] = 1.0 - 2.0 * (scale + CL);
  CL = q[2] * q[3];
  absxk = q[0] * q[1];
  R[7] = 2.0 * (CL - absxk);
  R[2] = 2.0 * (t - R_tmp);
  R[5] = 2.0 * (CL + absxk);
  R[8] = 1.0 - 2.0 * (scale + V);
  /* QMUL  Hamilton 四元数乘法 a (x) b。 */
  /*    与 [w;x;y;z] 排列一致，用于 qdot = 0.5*q (x) [0;omega]。 */
  b_x[0] = x[5] * x[11] - x[4] * x[12];
  b_x[1] = x[3] * x[12] - x[5] * x[10];
  b_x[2] = x[4] * x[10] - x[3] * x[11];
  V = 0.0;
  for (i = 0; i < 3; i++) {
    CL = x[i + 10];
    q[i + 1] = CL;
    Vb[i] = (a[i] * x[10] + a[i + 3] * x[11]) + a[i + 6] * x[12];
    V += x[i + 7] * CL;
  }
  CL = qbar * 0.55 * 2.5;
  B[0] = CL * (((-0.09 * beta + -0.5 * x[10] * 2.5 / (2.0 * Ve)) +
                0.08 * x[12] * 2.5 / (2.0 * Ve)) +
               0.18 * act[2]) -
         (Vb[2] * x[11] - Vb[1] * x[12]);
  B[1] = qbar * 0.55 * 0.22 *
             (((-0.8 * alpha + 0.05) + -12.0 * x[11] * 0.22 / (2.0 * Ve)) +
              -1.2 * act[1]) -
         (Vb[0] * x[12] - Vb[2] * x[10]);
  B[2] = CL * (((0.06 * beta + -0.03 * x[10] * 2.5 / (2.0 * Ve)) +
                -0.12 * x[12] * 2.5 / (2.0 * Ve)) +
               0.09 * act[3]) -
         (Vb[1] * x[10] - Vb[0] * x[11]);
  Vb[1] = B[1] - B[0] * 0.0;
  Vb[2] = (B[2] - B[0] * 0.0) - Vb[1] * 0.0;
  Vb[2] /= 0.85;
  Vb[0] = B[0] - Vb[2] * 0.0;
  Vb[1] -= Vb[2] * 0.0;
  Vb[1] /= 0.6;
  Vb[0] -= Vb[1] * 0.0;
  Vb[0] /= 0.45;
  B[0] = ((-D * b_Xa_tmp + L * Xa_tmp) + act[0] * 40.0) / 5.0;
  B[1] = qbar * 0.55 * (-0.6 * beta + 0.19 * act[3]) / 5.0;
  B[2] = (-D * Xa_tmp - L * b_Xa_tmp) / 5.0;
  b_q[0] = q[3] * x[8] - q[2] * x[9];
  b_q[1] = q[1] * x[9] - q[3] * x[7];
  b_q[2] = q[2] * x[7] - q[1] * x[8];
  xd[6] = 0.5 * (x[6] * 0.0 - V);
  for (i = 0; i < 3; i++) {
    xd[i] = (R[i] * x[3] + R[i + 3] * x[4]) + R[i + 6] * x[5];
    xd[i + 3] = (B[i] - b_x[i]) +
                ((R[3 * i] * 0.0 + R[3 * i + 1] * 0.0) + R[3 * i + 2] * 9.81);
    xd[i + 7] = 0.5 * ((x[6] * x[i + 10] + 0.0 * x[i + 7]) + b_q[i]);
    xd[i + 10] = Vb[i];
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
 * FW_SIM  固定翼闭环仿真（C 代码生成入口）。
 *    X = uav.codegen.fw_sim()
 *    与 uav.sim.run_fw 的数值完全一致：RK4 @1 ms、控制器 100 Hz、
 *    舵机与油门一阶滞后精确离散、记录积分前的状态。
 *    任务：100 m 平飞起，t=5 s 爬升 30 m，t=20 s 右转 60°，t=35 s 加速到 22
 * m/s。 返回 6000 x 13 状态序列，采样间隔 10 ms，总时长 60 s。
 *
 *
 * Arguments    : double X[78000]
 * Return Type  : void
 */
void fw_sim(double X[78000])
{
  static const double dv[13] = {0.0,
                                0.0,
                                -100.0,
                                17.986780172076166,
                                0.0,
                                0.68973838620722472,
                                0.99981637441965876,
                                0.0,
                                0.019162918418883054,
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
  double b_x[13];
  double dv1[13];
  double k1[13];
  double k2[13];
  double k3[13];
  double x[13];
  double act[4];
  double cmd[4];
  double V;
  double absxk;
  double att_idx_0;
  double att_idx_1;
  double att_idx_2;
  double n;
  double r_psi;
  double scale;
  double st_iV;
  double t;
  double u1;
  double x_idx_0;
  double x_idx_1;
  int it;
  int j;
  int k;
  int r_V;
  int r_h;
  boolean_T rEQ0;
  if (!isInitialized_fw_sim) {
    fw_sim_initialize();
  }
  /* EULER2QUAT  由 ZYX 欧拉角构造四元数（body 到 NED）。 */
  /*    q = qz(psi) * qy(theta) * qx(phi) */
  /* QMUL  Hamilton 四元数乘法 a (x) b。 */
  /*    与 [w;x;y;z] 排列一致，用于 qdot = 0.5*q (x) [0;omega]。 */
  /* QMUL  Hamilton 四元数乘法 a (x) b。 */
  /*    与 [w;x;y;z] 排列一致，用于 qdot = 0.5*q (x) [0;omega]。 */
  memset(&X[0], 0, 78000U * sizeof(double));
  memcpy(&x[0], &dv[0], 13U * sizeof(double));
  act[0] = 0.092939621189154262;
  cmd[0] = 0.092939621189154262;
  act[1] = 0.016114544747936877;
  cmd[1] = 0.016114544747936877;
  act[2] = 0.0;
  cmd[2] = 0.0;
  act[3] = 0.0;
  cmd[3] = 0.0;
  st_iV = 0.0;
  j = -1;
  r = _mm_set1_pd(2.0);
  for (k = 0; k < 60000; k++) {
    t = (((double)k + 1.0) - 1.0) * 0.001;
    if (k == 0) {
      r_V = 0;
    } else {
      r_V = (int)fmod(((double)k + 1.0) - 1.0, 10.0);
    }
    if (r_V == 0.0) {
      /* FW_REF  固定翼任务参考（纯函数，便于 codegen 与 Simulink 复用）。 */
      /*    t=5 s 爬升 30 m，t=20 s 右转 60 度，t=35 s 加速到 22 m/s。 */
      r_h = 100;
      if (t >= 5.0) {
        r_h = 130;
      }
      r_psi = 0.0;
      if (t >= 20.0) {
        r_psi = 1.0471975511965976;
      }
      r_V = 18;
      if (t >= 35.0) {
        r_V = 22;
      }
      /* FW_CASCADE  固定翼级联控制。 */
      /*    x   : 13 维状态 [pn; pe; pd; u; v; w; qw; qx; qy; qz; p; q; r] */
      /*    ref : 参考结构体，字段
       * h（高度，m）、psi（航向，rad）、V（空速，m/s） */
      /*    st  : 控制器状态，字段 iV（油门积分） */
      /*    act : [throttle; de; da; dr] */
      /*  */
      /*    俯仰指令以配平迎角为基准：theta_cmd = alpha_trim + 高度项。 */
      /*    航迹角 gamma = theta - alpha，所以爬升率近似为 V*sin(gamma)。 */
      /* QUAT2EULER  四元数转 ZYX 欧拉角 [phi; theta; psi]，单位 rad。 */
      /*    输入四元数为机体系 -> NED，奇异点 theta = +-pi/2 处 theta 被截断。
       */
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
        att_idx_0 = 0.0;
        att_idx_1 = 0.0;
        att_idx_2 = 0.0;
      } else {
        r1 = _mm_loadu_pd(&x[6]);
        r2 = _mm_set1_pd(n);
        _mm_storeu_pd(&cmd[0], _mm_div_pd(r1, r2));
        r1 = _mm_loadu_pd(&x[8]);
        _mm_storeu_pd(&cmd[2], _mm_div_pd(r1, r2));
        u1 = 2.0 * (cmd[0] * cmd[2] - cmd[1] * cmd[3]);
        if ((u1 >= 1.0) || rtIsNaN(u1)) {
          u1 = 1.0;
        }
        n = cmd[2] * cmd[2];
        att_idx_0 = rt_atan2d_snf(2.0 * (cmd[0] * cmd[1] + cmd[2] * cmd[3]),
                                  1.0 - 2.0 * (cmd[1] * cmd[1] + n));
        if (u1 <= -1.0) {
          u1 = -1.0;
        }
        att_idx_1 = asin(u1);
        att_idx_2 = rt_atan2d_snf(2.0 * (cmd[0] * cmd[3] + cmd[1] * cmd[2]),
                                  1.0 - 2.0 * (n + cmd[3] * cmd[3]));
      }
      scale = 3.3121686421112381E-170;
      absxk = fabs(x[3]);
      if (absxk > 3.3121686421112381E-170) {
        n = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        n = t * t;
      }
      absxk = fabs(x[4]);
      if (absxk > scale) {
        t = scale / absxk;
        n = n * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        n += t * t;
      }
      absxk = fabs(x[5]);
      if (absxk > scale) {
        t = scale / absxk;
        n = n * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        n += t * t;
      }
      n = scale * sqrt(n);
      if (n >= 1.0) {
        V = n;
      } else {
        V = 1.0;
      }
      n = r_psi - att_idx_2;
      /* WRAP_PI  角度归一化到 (-pi, pi]。 */
      if (rtIsNaN(n + 3.1415926535897931) || rtIsInf(n + 3.1415926535897931)) {
        t = rtNaN;
      } else if (n + 3.1415926535897931 == 0.0) {
        t = 0.0;
      } else {
        t = fmod(n + 3.1415926535897931, 6.2831853071795862);
        rEQ0 = (t == 0.0);
        if (!rEQ0) {
          att_idx_2 = fabs((n + 3.1415926535897931) / 6.2831853071795862);
          rEQ0 = !(fabs(att_idx_2 - floor(att_idx_2 + 0.5)) >
                   2.2204460492503131E-16 * att_idx_2);
        }
        if (rEQ0) {
          t = 0.0;
        } else if (n + 3.1415926535897931 < 0.0) {
          t += 6.2831853071795862;
        }
      }
      absxk = t - 3.1415926535897931;
      if (t - 3.1415926535897931 <= -3.1415926535897931) {
        absxk = 3.1415926535897931;
      }
      /*  配平前馈 */
      /* FW_TRIM  定直平飞配平。 */
      /*    给定空速 V 与高度无关的海平面密度，解迎角 alpha、升降舵 de 与油门，
       */
      /*    使机体轴上的合力与俯仰力矩为零。 */
      /*  */
      /*    平飞时航迹角为零、俯仰角等于迎角，机体 x 轴上还有重力的前向分量， */
      /*    因此推力要抵消阻力前向分量、升力前向分量与重力分量三项。 */
      /*    升力需求与阻力耦合，用几轮不动点迭代即可收敛（阻力是小量）。 */
      /*  */
      /*    结果一次构造为结构体：代码生成要求先算完局部量再建结构体， */
      /*    分步写 t.CD 再读它算 t.D 会触发字段锁定错误。 */
      att_idx_2 = 0.6125 * (double)(r_V * r_V);
      scale = 49.050000000000004 / (att_idx_2 * 0.55);
      /*  初值：升力等于重力 */
      for (it = 0; it < 6; it++) {
        /*  解 [CLa CLde; Cma Cmde] * [alpha; de] = [CL - CL0; -Cm0] */
        x_idx_1 = (-0.05 - (scale - 0.25) * -0.16) / -1.136;
        x_idx_0 = ((scale - 0.25) - x_idx_1 * 0.4) / 5.0;
        /*  由 z 轴力平衡更新升力需求：L*cos(a) + D*sin(a) = W*cos(a) */
        n = cos(x_idx_0);
        scale = (49.050000000000004 * n -
                 att_idx_2 * 0.55 * (0.045 * (scale * scale) + 0.025) *
                     sin(x_idx_0)) /
                (att_idx_2 * 0.55 * n);
      }
      n = sin(x_idx_0);
      /*  速度回路 */
      t = (double)r_V - V;
      st_iV += 0.02 * t * 0.01;
      if (st_iV >= 0.3) {
        u1 = 0.3;
      } else {
        u1 = st_iV;
      }
      if (u1 <= -0.3) {
        st_iV = -0.3;
      } else {
        st_iV = u1;
      }
      u1 = (((att_idx_2 * 0.55 * (0.045 * (scale * scale) + 0.025) *
                  cos(x_idx_0) -
              att_idx_2 * 0.55 * scale * n) +
             49.050000000000004 * n) /
                40.0 +
            0.15 * t) +
           st_iV;
      if ((u1 >= 1.0) || rtIsNaN(u1)) {
        u1 = 1.0;
      }
      /*  高度回路 */
      t = (x_idx_0 + 0.03 * ((double)r_h - (-x[2]))) -
          0.04 * (V * sin(att_idx_1 - rt_atan2d_snf(x[5], x[3])));
      if ((t >= 0.35) || rtIsNaN(t)) {
        t = 0.35;
      }
      /*  俯仰回路：Cmde 为负，正舵（后缘下偏）产生低头力矩，故误差项取负号 */
      if (t <= -0.35) {
        t = -0.35;
      }
      n = t - att_idx_1;
      t = (x_idx_1 - 0.6 * (2.0 * n - x[11])) - 0.5 * n;
      /*  航向回路 */
      att_idx_2 = 0.8 * absxk;
      if ((att_idx_2 >= 0.52) || rtIsNaN(att_idx_2)) {
        att_idx_2 = 0.52;
      }
      /*  滚转回路 */
      if (att_idx_2 <= -0.52) {
        att_idx_2 = -0.52;
      }
      att_idx_2 = 0.4 * (3.0 * (att_idx_2 - att_idx_0) - x[10]);
      /*  偏航阻尼 + 转弯协调 */
      n = -0.5 * x[12] + 0.1 * att_idx_0;
      if ((t >= 0.43633231299858238) || rtIsNaN(t)) {
        t = 0.43633231299858238;
      }
      if ((att_idx_2 >= 0.3490658503988659) || rtIsNaN(att_idx_2)) {
        att_idx_2 = 0.3490658503988659;
      }
      if ((n >= 0.43633231299858238) || rtIsNaN(n)) {
        n = 0.43633231299858238;
      }
      if (u1 <= 0.0) {
        cmd[0] = 0.0;
      } else {
        cmd[0] = u1;
      }
      if (t <= -0.43633231299858238) {
        cmd[1] = -0.43633231299858238;
      } else {
        cmd[1] = t;
      }
      if (att_idx_2 <= -0.3490658503988659) {
        cmd[2] = -0.3490658503988659;
      } else {
        cmd[2] = att_idx_2;
      }
      if (n <= -0.43633231299858238) {
        cmd[3] = -0.43633231299858238;
      } else {
        cmd[3] = n;
      }
      j++;
      for (r_h = 0; r_h < 13; r_h++) {
        X[j + 6000 * r_h] = x[r_h];
      }
    }
    /* ACTUATOR_STEP  舵机与油门的一阶滞后（精确离散）。 */
    /*    cmd : [throttle; de; da; dr] 指令 */
    /*    a   : 上一拍的执行机构位置 */
    /*    时间常数：油门 tauT，三个舵面 tauA。 */
    act[0] = cmd[0] + (act[0] - cmd[0]) * 0.99335550625503444;
    act[1] = cmd[1] + (act[1] - cmd[1]) * 0.98019867330675525;
    act[2] = cmd[2] + (act[2] - cmd[2]) * 0.98019867330675525;
    act[3] = cmd[3] + (act[3] - cmd[3]) * 0.98019867330675525;
    deriv(x, act, k1);
    for (r_h = 0; r_h <= 10; r_h += 2) {
      r1 = _mm_loadu_pd(&k1[r_h]);
      r2 = _mm_loadu_pd(&x[r_h]);
      _mm_storeu_pd(&b_x[r_h],
                    _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(0.0005), r1)));
    }
    b_x[12] = x[12] + 0.0005 * k1[12];
    deriv(b_x, act, k2);
    for (r_h = 0; r_h <= 10; r_h += 2) {
      r1 = _mm_loadu_pd(&k2[r_h]);
      r2 = _mm_loadu_pd(&x[r_h]);
      _mm_storeu_pd(&b_x[r_h],
                    _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(0.0005), r1)));
    }
    b_x[12] = x[12] + 0.0005 * k2[12];
    deriv(b_x, act, k3);
    for (r_h = 0; r_h <= 10; r_h += 2) {
      r1 = _mm_loadu_pd(&k3[r_h]);
      r2 = _mm_loadu_pd(&x[r_h]);
      _mm_storeu_pd(&b_x[r_h],
                    _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(0.001), r1)));
    }
    b_x[12] = x[12] + 0.001 * k3[12];
    deriv(b_x, act, dv1);
    for (r_h = 0; r_h <= 10; r_h += 2) {
      r1 = _mm_loadu_pd(&k2[r_h]);
      r2 = _mm_loadu_pd(&k1[r_h]);
      r3 = _mm_loadu_pd(&k3[r_h]);
      r4 = _mm_loadu_pd(&dv1[r_h]);
      r5 = _mm_loadu_pd(&x[r_h]);
      _mm_storeu_pd(
          &x[r_h],
          _mm_add_pd(
              r5, _mm_mul_pd(
                      _mm_set1_pd(0.00016666666666666666),
                      _mm_add_pd(_mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r, r1)),
                                            _mm_mul_pd(r, r3)),
                                 r4))));
    }
    x[12] += 0.00016666666666666666 *
             (((k1[12] + 2.0 * k2[12]) + 2.0 * k3[12]) + dv1[12]);
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
 * File trailer for fw_sim.c
 *
 * [EOF]
 */
