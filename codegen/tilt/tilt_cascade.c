/*
 * File: tilt_cascade.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-31 05:20:27
 */

/* Include Files */
#include "tilt_cascade.h"
#include "mod.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "tilt_sim_internal_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
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
 * TILT_CASCADE  倾转四旋翼统一级联控制（空速调度 + 权限分配）。
 *    x   : 21 维状态，前 13 维与四旋翼/固定翼一致，14 短舱角，15:18
 * 旋翼转速，19:21 舵面 ref : 结构体，字段 p（NED 位置）、v、a、psi、h、V cmd :
 * [om(4); de; da; dr; betaCmd]，8 维
 *
 *    位置环先算期望加速度 a，再由推力需求 Tn=m*(a-g) 得到旋翼要补的力
 *    Frot = R'*Tn - Fa。姿态指令按空速权重 k 在两支之间线性混合：
 *      k=0 四旋翼支：机体 -z 轴对准 Tn；
 *      k=1 固定翼支：俯仰维持配平迎角 + 高度/速度反馈，升力由机翼出。
 *    硬性按约束 R*d(beta)=n 反解姿态在 cb≈0.3 处会跳变，实测把俯仰打到 -17°，
 *    机翼迎角反号，越低头越掉高，所以改成混合而不是解约束。
 *
 *    U1 用投影而不是模长：U1=max(d'*Frot,0)，d=[sin b;0;-cos b]。模长会把
 *    横向分量也算成推力，β=90° 时推力全给前向，转速顶到限幅收不回来。
 *
 * Arguments    : const double x[21]
 *                const double ref_p[3]
 *                const double ref_v[3]
 *                const double ref_a[3]
 *                double ref_V
 *                struct_T *st
 *                double cmd[8]
 * Return Type  : void
 */
void tilt_cascade(const double x[21], const double ref_p[3],
                  const double ref_v[3], const double ref_a[3], double ref_V,
                  struct_T *st, double cmd[8])
{
  static const double b_varargin_1[3] = {-0.0, -0.0, -1.2};
  static const double dv[3] = {1.0, 1.0, 1.6};
  static const double dv1[3] = {1.8, 1.8, 2.2};
  static const double dv2[3] = {0.0, 0.0, 0.8};
  static const double dv3[3] = {0.0, 0.0, 9.81};
  static const double varargin_1[3] = {0.0, 0.0, 1.2};
  __m128d r;
  __m128d r1;
  double R[9];
  double T[4];
  double Frot[3];
  double Vn[3];
  double ep[3];
  double minval[3];
  double yb[3];
  double L;
  double R_tmp;
  double V;
  double Ve;
  double alpha;
  double att_idx_0;
  double att_idx_1;
  double att_idx_2;
  double cb;
  double k;
  double n;
  double qbar;
  double u1;
  int b_k;
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
    r = _mm_set1_pd(n);
    _mm_storeu_pd(&T[0], _mm_div_pd(_mm_loadu_pd(&x[6]), r));
    _mm_storeu_pd(&T[2], _mm_div_pd(_mm_loadu_pd(&x[8]), r));
  }
  n = T[3] * T[3];
  Ve = T[2] * T[2];
  R[0] = 1.0 - 2.0 * (Ve + n);
  alpha = T[1] * T[2];
  qbar = T[0] * T[3];
  R[3] = 2.0 * (alpha - qbar);
  L = T[1] * T[3];
  R_tmp = T[0] * T[2];
  R[6] = 2.0 * (L + R_tmp);
  R[1] = 2.0 * (alpha + qbar);
  alpha = T[1] * T[1];
  R[4] = 1.0 - 2.0 * (alpha + n);
  n = T[2] * T[3];
  qbar = T[0] * T[1];
  R[7] = 2.0 * (n - qbar);
  R[2] = 2.0 * (L - R_tmp);
  R[5] = 2.0 * (n + qbar);
  R[8] = 1.0 - 2.0 * (alpha + Ve);
  /* QUAT2EULER  四元数转 ZYX 欧拉角 [phi; theta; psi]，单位 rad。 */
  /*    输入四元数为机体系 -> NED，奇异点 theta = +-pi/2 处 theta 被截断。 */
  n = b_norm(&x[6]);
  if (n < 1.0E-12) {
    att_idx_0 = 0.0;
    att_idx_1 = 0.0;
    att_idx_2 = 0.0;
  } else {
    r = _mm_set1_pd(n);
    _mm_storeu_pd(&T[0], _mm_div_pd(_mm_loadu_pd(&x[6]), r));
    _mm_storeu_pd(&T[2], _mm_div_pd(_mm_loadu_pd(&x[8]), r));
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
  V = c_norm(&x[3]);
  cb = 0.6125 * (V * V);
  /*  转捩调度用参考空速而不是实测空速：实测空速低于计划时，若按它算 k， */
  /*  k 会回落，短舱跟着往回收，机体在过渡段来回摆。用 ref.V 做开环调度， */
  /*  短舱按任务计划单调前倾，实测速度只作为内环的被控量。 */
  /*  减速段曾改成按实测空速调度（仿 PX4 的 back-transition 判据），实测更差：
   */
  /*  短舱收得晚、机翼迎角一直偏大，机体变成稳定爬升，末端高度偏差 22 m。 */
  /*  细节见 research/REPORT_r2_backtransition.md。 */
  /*  */
  /*  现在的折中：调度量取 min(实测, 参考+2)。参考空速领先实测 6–8 m/s 时， */
  /*  短舱会提前收回、迎角突然变大，这是 r5 里峰值 23.8 m 的来源；纯实测又会 */
  /*  收得太晚。加 2 m/s 的领先帽，等价于「转捩走廊是真实速度的函数， */
  /*  但允许最多 2 m/s 的超前」，细节见 research/REPORT_r3_decel.md。 */
  if ((V <= ref_V + 2.0) || rtIsNaN(ref_V + 2.0)) {
    Ve = V;
  } else {
    Ve = ref_V + 2.0;
  }
  u1 = (Ve - 3.0) / 11.0;
  if ((u1 >= 1.0) || rtIsNaN(u1)) {
    u1 = 1.0;
  }
  if (u1 <= 0.0) {
    k = 0.0;
  } else {
    k = u1;
  }
  /*  ---- 位置环 ---- */
  for (b_k = 0; b_k < 3; b_k++) {
    alpha = (R[b_k] * x[3] + R[b_k + 3] * x[4]) + R[b_k + 6] * x[5];
    Vn[b_k] = alpha;
    n = ref_p[b_k] - x[b_k];
    Ve = st->ip[b_k] + n * 0.01;
    if ((varargin_1[b_k] <= Ve) || rtIsNaN(Ve)) {
      Ve = varargin_1[b_k];
    }
    if (b_varargin_1[b_k] >= Ve) {
      Ve = b_varargin_1[b_k];
    }
    st->ip[b_k] = Ve;
    n = ((dv[b_k] * n + dv1[b_k] * (ref_v[b_k] - alpha)) + ref_a[b_k]) +
        dv2[b_k] * Ve;
    ep[b_k] = n;
  }
  /*  后向加速度受气动阻力限制：转捩段机翼已经在扛重力，任何大幅抬头都会让 */
  /*  升力翻倍（13.7 m/s、24° 迎角时升力约 810 N），多出来的只能变成爬升。 */
  /*  平飞可用的减速力就是阻力，约 1.5 m/s^2；超过它的需求一律截到 c.aDecMax。
   */
  if ((k > 0.1) && (ep[0] < 0.0) && (!(ep[0] >= -1.5))) {
    ep[0] = -1.5;
  }
  n = d_norm(&ep[0]);
  if (n > 2.5) {
    r = _mm_loadu_pd(&ep[0]);
    _mm_storeu_pd(&ep[0], _mm_mul_pd(r, _mm_set1_pd(2.5 / n)));
  }
  if ((ep[2] >= 4.0) || rtIsNaN(ep[2])) {
    u1 = 4.0;
  } else {
    u1 = ep[2];
  }
  if (u1 <= -4.0) {
    ep[2] = -4.0;
  } else {
    ep[2] = u1;
  }
  /*  限速：位置误差大时位置环会一直给满加速度，速度一路超。超过参考两米每秒 */
  /*  就把沿速度方向的加速度压成"减速"，让速度回到参考附近。 */
  n = c_norm(ref_v);
  if (V > n + 2.0) {
    alpha = Vn[0] / V;
    Vn[0] = alpha;
    Ve = ep[0] * alpha;
    alpha = Vn[1] / V;
    Vn[1] = alpha;
    Ve += ep[1] * alpha;
    alpha = Vn[2] / V;
    Vn[2] = alpha;
    Ve += ep[2] * alpha;
    n = -((V - n) - 2.0);
    if (Ve > n) {
      n -= Ve;
      r = _mm_loadu_pd(&Vn[0]);
      r1 = _mm_loadu_pd(&ep[0]);
      _mm_storeu_pd(&ep[0], _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(n))));
      ep[2] += alpha * n;
    }
  }
  /*  ---- 需要的合力与旋翼份额 ---- */
  /*  NED，非重力合力（悬停时 [0;0;-mg]） */
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
  r = _mm_loadu_pd(&ep[0]);
  _mm_storeu_pd(&ep[0], _mm_mul_pd(_mm_set1_pd(8.0),
                                   _mm_sub_pd(r, _mm_loadu_pd(&dv3[0]))));
  _mm_storeu_pd(&T[1], _mm_loadu_pd(&x[18]));
  _mm_storeu_pd(&Vn[0], _mm_loadu_pd(&x[3]));
  ep[2] = 8.0 * (ep[2] - 9.81);
  Vn[2] = x[5];
  n = c_norm(Vn);
  if (n >= 1.0) {
    Ve = n;
  } else {
    Ve = 1.0;
  }
  /*  低速时避免除零，只在悬停/起飞段生效 */
  /*  线性气动只在失速前有效：悬停遇侧风时相对气流可以接近垂直于机身， */
  /*  atan2(0,-2) 这种退化情形会给出 180°，线性 CL/CY 会算出几十牛的气动力 */
  /*  把整机掀翻（r17 实测 6 m/s 侧风滚转 141°）。这里用 asin 速度分量比， */
  /*  并把两个角都截到 ±19.5°——相当于"失速后气动力不再增长"的最粗模型。 */
  u1 = x[5] / Ve;
  if ((u1 >= 0.34) || rtIsNaN(u1)) {
    u1 = 0.34;
  }
  if (u1 <= -0.34) {
    u1 = -0.34;
  }
  alpha = asin(u1);
  u1 = Vn[1] / Ve;
  if ((u1 >= 0.34) || rtIsNaN(u1)) {
    u1 = 0.34;
  }
  qbar = 0.6125 * (n * n);
  n = ((5.0 * alpha + 0.45) + 3.9 * x[11] * 0.34 / (2.0 * Ve)) + 0.4 * T[1];
  Ve = qbar * 1.6 * (0.045 * (n * n) + 0.025);
  L = qbar * 1.6 * n;
  /*  稳定性轴到机体轴 */
  R_tmp = sin(alpha);
  n = cos(alpha);
  minval[0] = -Ve * n + L * R_tmp;
  if (u1 <= -0.34) {
    u1 = -0.34;
  }
  minval[1] = qbar * 1.6 * (-0.6 * asin(u1) + 0.19 * x[20]);
  minval[2] = -Ve * R_tmp - L * n;
  alpha = ep[0];
  n = ep[1];
  Ve = ep[2];
  for (b_k = 0; b_k < 3; b_k++) {
    Frot[b_k] =
        ((R[3 * b_k] * alpha + R[3 * b_k + 1] * n) + R[3 * b_k + 2] * Ve) -
        minval[b_k];
  }
  if (c_norm(Frot) < 1.0E-6) {
    Frot[0] = 0.0;
    Frot[1] = 0.0;
    Frot[2] = -78.48;
  }
  /*  ---- 姿态指令：四旋翼支与固定翼支按 k 混合 ---- */
  if (cb >= 1.0) {
    L = cb;
  } else {
    L = 1.0;
  }
  u1 = (78.48 / (L * 1.6) - 0.45) / 5.0;
  /*  悬停时不能用机翼配平迎角 */
  /*  四旋翼支：机体 -z 轴对准 Tn */
  cb = c_norm(ep);
  if (cb >= 1.0E-9) {
    n = cb;
  } else {
    n = 1.0E-9;
  }
  r = _mm_loadu_pd(&ep[0]);
  _mm_storeu_pd(&ep[0],
                _mm_div_pd(_mm_mul_pd(r, _mm_set1_pd(-1.0)), _mm_set1_pd(n)));
  ep[2] = -ep[2] / n;
  Vn[0] = cos(att_idx_2);
  Vn[1] = sin(att_idx_2);
  yb[0] = ep[1] * 0.0 - Vn[1] * ep[2];
  yb[1] = Vn[0] * ep[2] - ep[0] * 0.0;
  yb[2] = ep[0] * Vn[1] - Vn[0] * ep[1];
  qbar = c_norm(yb);
  if (qbar < 1.0E-6) {
    yb[0] = ep[1] * 0.0 - 0.0 * ep[2];
    yb[1] = ep[2] - ep[0] * 0.0;
    yb[2] = ep[0] * 0.0 - ep[1];
    qbar = c_norm(yb);
  }
  r = _mm_loadu_pd(&yb[0]);
  r = _mm_div_pd(r, _mm_set1_pd(qbar));
  _mm_storeu_pd(&yb[0], r);
  _mm_storeu_pd(&R[3], r);
  r = _mm_loadu_pd(&ep[0]);
  _mm_storeu_pd(&R[6], r);
  R[2] = yb[0] * ep[1] - ep[0] * yb[1];
  if ((R[2] >= 1.0) || rtIsNaN(R[2])) {
    R_tmp = 1.0;
  } else {
    R_tmp = R[2];
  }
  /*  固定翼支：配平迎角 + 高度与空速反馈 */
  st->iH += (20.0 - (-x[2])) * 0.01;
  if ((st->iH >= 40.0) || rtIsNaN(st->iH)) {
    n = 40.0;
  } else {
    n = st->iH;
  }
  if (n <= -40.0) {
    st->iH = -40.0;
  } else {
    st->iH = n;
  }
  if (u1 >= 0.3) {
    u1 = 0.3;
  }
  u1 = (((k * u1 + 0.03 * (20.0 - (-x[2]))) + 0.006 * st->iH) -
        0.04 * (V * sin(att_idx_1 - rt_atan2d_snf(x[5], x[3])))) +
       0.03 * (V - ref_V);
  if ((u1 >= 0.3) || rtIsNaN(u1)) {
    u1 = 0.3;
  }
  /* WRAP_PI  角度归一化到 (-pi, pi]。 */
  Ve = b_mod((0.0 - att_idx_2) + 3.1415926535897931);
  n = Ve - 3.1415926535897931;
  if (Ve - 3.1415926535897931 <= -3.1415926535897931) {
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
  Vn[0] =
      (((1.0 - k) * rt_atan2d_snf(yb[2] / qbar, ep[2]) + k * n) - att_idx_0) +
      3.1415926535897931;
  yb[0] = b_mod(Vn[0]) - 3.1415926535897931;
  if (R_tmp <= -1.0) {
    R_tmp = -1.0;
  }
  if (u1 <= -0.3) {
    u1 = -0.3;
  }
  yb[1] = b_mod((((1.0 - k) * -asin(R_tmp) + k * u1) - att_idx_1) +
                3.1415926535897931) -
          3.1415926535897931;
  yb[2] = Ve - 3.1415926535897931;
  if (yb[0] <= -3.1415926535897931) {
    yb[0] = 3.1415926535897931;
  }
  if (yb[1] <= -3.1415926535897931) {
    yb[1] = 3.1415926535897931;
  }
  if (Ve - 3.1415926535897931 <= -3.1415926535897931) {
    yb[2] = 3.1415926535897931;
  }
  /*  舵面支的力矩增益要随动压放大：力矩到舵偏是 de = M/(qbar*S*c*Cmde)， */
  /*  动压从 20 Pa 涨到 120 Pa 时同一个 M 只能给出 1/6 的舵偏。不补这一项， */
  /*  巡航段 30° 的俯仰误差只换来 2° 舵偏，机体一路抬头爬高。 */
  if (1.0 - k <= 0.0) {
    att_idx_0 = 0.0;
  } else {
    att_idx_0 = 1.0 - k;
  }
  u1 = L / 20.0;
  if (u1 <= 1.0) {
    u1 = 1.0;
  }
  n = (1.0 - att_idx_0) * u1;
  /*  力矩预算随权限走：悬停时惯量小，几 N·m 就够；巡航动压上来后舵面能给出 */
  /*  几十 N·m，若仍按悬停的 maxM 截断，俯仰指令同样会被压住。 */
  att_idx_1 = L * 1.6 * 3.6;
  V = att_idx_1 * 0.18;
  Vn[0] = att_idx_0 * 2.5 + (1.0 - att_idx_0) * (V * 0.3490658503988659);
  att_idx_2 = L * 1.6 * 0.34;
  Vn[1] = att_idx_0 * 2.5 +
          (1.0 - att_idx_0) * (att_idx_2 * 1.2 * 0.43633231299858238);
  att_idx_1 *= 0.09;
  Vn[2] = att_idx_0 + (1.0 - att_idx_0) * (att_idx_1 * 0.43633231299858238);
  /*  ---- 分配：旋翼份额与舵面份额 ---- */
  L = sin(x[13]);
  cb = cos(x[13]);
  alpha = st->ia[0] + yb[0] * 0.01;
  if ((alpha >= 0.0) || rtIsNaN(alpha)) {
    alpha = 0.0;
  }
  if (alpha <= -0.0) {
    alpha = -0.0;
  }
  st->ia[0] = alpha;
  alpha = (6.0 * yb[0] + 0.0 * alpha) - 4.0 * x[10];
  alpha = att_idx_0 * alpha + n * alpha;
  if ((Vn[0] <= alpha) || rtIsNaN(alpha)) {
    alpha = Vn[0];
  }
  if ((-Vn[0] >= alpha) || rtIsNaN(alpha)) {
    yb[0] = -Vn[0];
  } else {
    yb[0] = alpha;
  }
  alpha = st->ia[1] + yb[1] * 0.01;
  if ((alpha >= 0.0) || rtIsNaN(alpha)) {
    alpha = 0.0;
  }
  if (alpha <= -0.0) {
    alpha = -0.0;
  }
  st->ia[1] = alpha;
  alpha = (6.0 * yb[1] + 0.0 * alpha) - 4.0 * x[11];
  alpha = att_idx_0 * alpha + n * alpha;
  if ((Vn[1] <= alpha) || rtIsNaN(alpha)) {
    alpha = Vn[1];
  }
  if ((-Vn[1] >= alpha) || rtIsNaN(alpha)) {
    yb[1] = -Vn[1];
  } else {
    yb[1] = alpha;
  }
  alpha = st->ia[2] + yb[2] * 0.01;
  if ((alpha >= 0.35) || rtIsNaN(alpha)) {
    alpha = 0.35;
  }
  if (alpha <= -0.35) {
    alpha = -0.35;
  }
  st->ia[2] = alpha;
  alpha = (3.0 * yb[2] + 0.6 * alpha) - 2.0 * x[12];
  alpha = att_idx_0 * alpha + n * alpha;
  if ((Vn[2] <= alpha) || rtIsNaN(alpha)) {
    alpha = Vn[2];
  }
  if ((-Vn[2] >= alpha) || rtIsNaN(alpha)) {
    yb[2] = -Vn[2];
  } else {
    yb[2] = alpha;
  }
  n = (L * Frot[0] + 0.0 * Frot[1]) + -cb * Frot[2];
  if ((n <= 0.0) || rtIsNaN(n)) {
    n = 0.0;
  }
  u1 = (1.0 - k) * c_norm(Frot) + k * n;
  if ((u1 >= 125.83999999999999) || rtIsNaN(u1)) {
    u1 = 125.83999999999999;
  }
  if (u1 <= 0.0) {
    R_tmp = 0.0;
  } else {
    R_tmp = u1;
  }
  n = att_idx_0 * yb[0];
  alpha = att_idx_0 * yb[2];
  Ve = (-L * n + cb * alpha) / 0.019230769230769232;
  /* TILT_MIX_FAULT  考虑单电机降效的虚拟控制量分配。 */
  /*    [om, sat, Tmix] = uav.plant.tilt_mix_fault(U1, U2, U3, U4, p, act,
   * method) */
  /*  */
  /*    正常分配解 B*T = [U1;U2;U3;U4]（B 见 uav.plant.tilt_mix）。电机 i 降效
   */
  /*    因子 act(i)∈[0,1] 时，实际产生的推力是 act(i)*T(i)，于是约束变成 */
  /*  */
  /*        B * diag(act) * T = U */
  /*  */
  /*    4 个未知、4 个方程，但秩只有 3（一台失效时）——偏航方向的分量不再可控。
   */
  /*  */
  /*    method: */
  /*      'pinv'（默认）——最小范数解，滚转/俯仰/总推力尽量满足，偏航残差留给 */
  /*         机体自转。悬停时剩下的三台可能被推到饱和，姿态随后发散（r10
   * 实测）。 */
  /*      'rpi'——带饱和的处理：先解最小范数，把超限的那一台钉在边界，再对剩下的
   */
  /*         三台重解，重复到没有新的越界。代价是最小范数性，收益是总推力与 */
  /*         滚转/俯仰优先满足，悬停断桨不再必然发散。 */
  /*  */
  /*    输入 act 为 4x1，全 1 时退化成普通逆混控。 */
  /* TILT_MIX  虚拟控制量 -> 四个旋翼转速。 */
  /*    解 [U1;U2;U3;U4] = M * [T1;T2;T3;T4] */
  /*        M = [ 1    1    1    1 ; */
  /*             -aY  -aY   aY   aY ; */
  /*              aX  -aX  -aX   aX ; */
  /*              1   -1    1   -1 ] */
  /*    逆矩阵按显式算式展开，便于在 Simulink 图表里内联。 */
  /*    sat 为 1 表示有旋翼推力被截断到 [0, wMax]（或出现过负推力需求）。 */
  qbar = 0.25 * (cb * n + L * alpha) / 0.9;
  L = 0.25 * R_tmp - qbar;
  if (!(cb >= 0.2)) {
    cb = 0.2;
  }
  n = 0.25 * (att_idx_0 * yb[1] / cb) / 0.4;
  T[0] = (L + n) + 0.25 * Ve;
  T[1] = (L - n) - 0.25 * Ve;
  L = 0.25 * R_tmp + qbar;
  T[2] = (L - n) + 0.25 * Ve;
  T[3] = (L + n) - 0.25 * Ve;
  u1 = (1.0 - att_idx_0) * yb[0] / V;
  R_tmp = (1.0 - att_idx_0) * yb[1] / (att_idx_2 * -1.2);
  n = (1.0 - att_idx_0) * yb[2] / att_idx_1;
  if ((R_tmp >= 0.43633231299858238) || rtIsNaN(R_tmp)) {
    R_tmp = 0.43633231299858238;
  }
  if ((u1 >= 0.3490658503988659) || rtIsNaN(u1)) {
    u1 = 0.3490658503988659;
  }
  if ((n >= 0.43633231299858238) || rtIsNaN(n)) {
    n = 0.43633231299858238;
  }
  /*  早期这里写 st.dbg = [a(:); phiD; thetaD; U1; k] 用于调参， */
  /*  代码生成不允许在结构体被读取后再加字段，已删除；需要诊断时临时加回来。 */
  if (T[0] >= 0.0) {
    Ve = T[0];
  } else {
    Ve = 0.0;
  }
  alpha = sqrt(Ve / 2.6E-5);
  if (alpha <= 1100.0) {
    cmd[0] = alpha;
  } else {
    cmd[0] = 1100.0;
  }
  if (T[1] >= 0.0) {
    Ve = T[1];
  } else {
    Ve = 0.0;
  }
  alpha = sqrt(Ve / 2.6E-5);
  if (alpha <= 1100.0) {
    cmd[1] = alpha;
  } else {
    cmd[1] = 1100.0;
  }
  if (T[2] >= 0.0) {
    Ve = T[2];
  } else {
    Ve = 0.0;
  }
  alpha = sqrt(Ve / 2.6E-5);
  if (alpha <= 1100.0) {
    cmd[2] = alpha;
  } else {
    cmd[2] = 1100.0;
  }
  if (T[3] >= 0.0) {
    Ve = T[3];
  } else {
    Ve = 0.0;
  }
  alpha = sqrt(Ve / 2.6E-5);
  if (alpha <= 1100.0) {
    cmd[3] = alpha;
  } else {
    cmd[3] = 1100.0;
  }
  if (R_tmp <= -0.43633231299858238) {
    cmd[4] = -0.43633231299858238;
  } else {
    cmd[4] = R_tmp;
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
  cmd[7] = 1.5707963267948966 * k;
}

/*
 * File trailer for tilt_cascade.c
 *
 * [EOF]
 */
