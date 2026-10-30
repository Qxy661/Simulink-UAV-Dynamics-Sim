/*
 * File: tilt_cascade.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-30 21:47:54
 */

#ifndef TILT_CASCADE_H
#define TILT_CASCADE_H

/* Include Files */
#include "rtwtypes.h"
#include "tilt_sim_internal_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void tilt_cascade(const double x[21], const double ref_p[3],
                  const double ref_v[3], const double ref_a[3], double ref_V,
                  struct_T *st, double cmd[8]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tilt_cascade.h
 *
 * [EOF]
 */
