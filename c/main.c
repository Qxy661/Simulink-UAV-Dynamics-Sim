/*
 * main.c —— 四旋翼闭环仿真的独立运行器。
 *
 * 不依赖 MATLAB：链接 codegen/ 下由 uav.codegen.quad_sim 生成的 C 代码，
 * 跑同一段 14 s 阶跃任务，把 1400 个样本写成 CSV。
 *
 * 用法：uav_quad_runner [pn pe pd psi] [输出文件]
 *   默认 3 2 -5 0.4 与 out/quad_step.csv
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "quad_sim.h"
#include "quad_sim_initialize.h"
#include "quad_sim_terminate.h"

#define N_SAMP 1400
#define N_STATE 13
#define DT_LOG 0.01

/* MATLAB 的列主序：数组下标 = 列 * N_SAMP + 行 */
#define XGET(k, i) (X[(i) * N_SAMP + (k)])

static double X[N_SAMP * N_STATE];

int main(int argc, char *argv[])
{
    double ref[3] = {3.0, 2.0, -5.0};
    double psi = 0.4;
    const char *path = "out/quad_step.csv";
    int i, k;
    FILE *f;

    if (argc >= 5) {
        ref[0] = atof(argv[1]);
        ref[1] = atof(argv[2]);
        ref[2] = atof(argv[3]);
        psi    = atof(argv[4]);
    }
    if (argc >= 6) {
        path = argv[5];
    }

    quad_sim_initialize();
    quad_sim(ref, psi, X);

    f = fopen(path, "w");
    if (f == NULL) {
        fprintf(stderr, "cannot open %s\n", path);
        quad_sim_terminate();
        return 1;
    }

    fprintf(f, "t,pn,pe,pd,u,v,w,qw,qx,qy,qz,p,q,r\n");
    for (k = 0; k < N_SAMP; k++) {
        fprintf(f, "%.6f", k * DT_LOG);
        for (i = 0; i < N_STATE; i++) {
            fprintf(f, ",%.17g", XGET(k, i));
        }
        fprintf(f, "\n");
    }
    fclose(f);

    printf("wrote %s (%d samples)\n", path, N_SAMP);
    printf("final p = [%.6f %.6f %.6f]\n",
           XGET(N_SAMP - 1, 0), XGET(N_SAMP - 1, 1), XGET(N_SAMP - 1, 2));

    quad_sim_terminate();
    return 0;
}
