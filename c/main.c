/*
 * main.c —— 三个构型的独立运行器。
 *
 * 同一个源文件配不同的 -DMISSION_* 宏，分别链接 codegen/quad、codegen/fw、
 * codegen/tilt 下由 MATLAB Coder 生成的代码，不依赖 MATLAB。
 *
 * 用法：uav_<构型>_runner [输出文件]
 *   默认写 out/<构型>.csv
 */

#include <stdio.h>
#include <stdlib.h>

#if defined(MISSION_quad)
#include "quad_sim.h"
#include "quad_sim_initialize.h"
#include "quad_sim_terminate.h"
#define N_SAMP 1400
#define N_STATE 13
#define HEADER "t,pn,pe,pd,u,v,w,qw,qx,qy,qz,p,q,r"
#define CALL(X) do { quad_sim_initialize(); quad_sim(refp, psi, (X)); } while (0)
#define FINISH() quad_sim_terminate()
#elif defined(MISSION_fw)
#include "fw_sim.h"
#include "fw_sim_initialize.h"
#include "fw_sim_terminate.h"
#define N_SAMP 6000
#define N_STATE 13
#define HEADER "t,pn,pe,pd,u,v,w,qw,qx,qy,qz,p,q,r"
#define CALL(X) do { fw_sim_initialize(); fw_sim((X)); } while (0)
#define FINISH() fw_sim_terminate()
#elif defined(MISSION_tilt)
#include "tilt_sim.h"
#include "tilt_sim_initialize.h"
#include "tilt_sim_terminate.h"
#define N_SAMP 4500
#define N_STATE 21
#define HEADER "t,pn,pe,pd,u,v,w,qw,qx,qy,qz,p,q,r,beta,om1,om2,om3,om4,de,da,dr"
#define CALL(X) do { tilt_sim_initialize(); tilt_sim((X)); } while (0)
#define FINISH() tilt_sim_terminate()
#else
#error "define MISSION_quad, MISSION_fw or MISSION_tilt"
#endif

#define DT_LOG 0.01

/* MATLAB 列主序：元素下标 = 列 * N_SAMP + 行 */
#define XGET(X, k, i) ((X)[(i) * N_SAMP + (k)])

static double X[N_SAMP * N_STATE];

int main(int argc, char *argv[])
{
    const char *path = "out/trace.csv";
    int i, k;
    FILE *f;
#if defined(MISSION_quad)
    double refp[3] = {3.0, 2.0, -5.0};
    double psi = 0.4;
#endif

    if (argc >= 2) {
        path = argv[1];
    }

    CALL(X);

    f = fopen(path, "w");
    if (f == NULL) {
        fprintf(stderr, "cannot open %s\n", path);
        FINISH();
        return 1;
    }

    fprintf(f, "%s\n", HEADER);
    for (k = 0; k < N_SAMP; k++) {
        fprintf(f, "%.6f", k * DT_LOG);
        for (i = 0; i < N_STATE; i++) {
            fprintf(f, ",%.17g", XGET(X, k, i));
        }
        fprintf(f, "\n");
    }
    fclose(f);

    printf("wrote %s (%d samples x %d states)\n", path, N_SAMP, N_STATE);
    printf("final p = [%.6f %.6f %.6f]\n",
           XGET(X, N_SAMP - 1, 0), XGET(X, N_SAMP - 1, 1), XGET(X, N_SAMP - 1, 2));

    FINISH();
    return 0;
}
