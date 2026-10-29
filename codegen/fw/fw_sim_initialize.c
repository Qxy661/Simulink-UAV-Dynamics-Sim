/*
 * File: fw_sim_initialize.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-29 21:15:09
 */

/* Include Files */
#include "fw_sim_initialize.h"
#include "fw_sim_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void fw_sim_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_fw_sim = true;
}

/*
 * File trailer for fw_sim_initialize.c
 *
 * [EOF]
 */
