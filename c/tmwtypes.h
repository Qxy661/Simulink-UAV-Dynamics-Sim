/*
 * tmwtypes.h —— 最小类型垫片。
 *
 * MATLAB Coder 生成的 rtwtypes.h 会 #include "tmwtypes.h"，而这个文件属于
 * MATLAB 安装目录（extern/include），不在仓库里。为了让本仓库 clone 下来
 * 只用 gcc 就能编译生成的 C，这里给出生成的代码实际用到的那几个类型定义。
 *
 * 若将来引入定点或复数信号，需要把对应类型补进来，或者改用 MATLAB 安装目录
 * 下的官方头文件（-I"$MATLABROOT/extern/include"）。
 */

#ifndef TMWTYPES_H
#define TMWTYPES_H

#include <stddef.h>

typedef double   real_T;
typedef float    real32_T;
typedef char     char_T;
typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short    int16_T;
typedef unsigned short uint16_T;
typedef int      int32_T;
typedef unsigned int uint32_T;
typedef long long int64_T;
typedef unsigned long long uint64_T;

typedef int      boolean_T;
typedef double   time_T;

#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif

#endif
