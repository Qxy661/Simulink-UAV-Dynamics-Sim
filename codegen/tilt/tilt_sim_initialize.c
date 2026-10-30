/*
 * File: tilt_sim_initialize.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-30 21:47:54
 */

/* Include Files */
#include "tilt_sim_initialize.h"
#include "rt_nonfinite.h"
#include "tilt_sim_data.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void tilt_sim_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_tilt_sim = true;
}

/*
 * File trailer for tilt_sim_initialize.c
 *
 * [EOF]
 */
