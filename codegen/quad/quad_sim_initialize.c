/*
 * File: quad_sim_initialize.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-30 21:47:26
 */

/* Include Files */
#include "quad_sim_initialize.h"
#include "quad_sim_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void quad_sim_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_quad_sim = true;
}

/*
 * File trailer for quad_sim_initialize.c
 *
 * [EOF]
 */
