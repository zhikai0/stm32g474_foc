#pragma once
/* float32_t 兼容定义：C++ 不 include arm_math.h 时自行定义 */
#ifndef __CMSIS_GENERIC
#ifdef __cplusplus
  #ifndef ARM_MATH_CM4
    typedef float float32_t;
  #endif
#else
  #include "arm_math.h"
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 查表法 sin/cos（精度 ~1e-4，~60 cycles @ 170MHz）*/
float32_t our_arm_sin_f32(float32_t x);
float32_t our_arm_cos_f32(float32_t x);

/* CORDIC 硬件轮询 sin（精度 ~1e-6，~48 cycles @ 170MHz）*/
float32_t cordic_sin_f32(float32_t x);

/* 快速 atan2（多项式近似，精度 ~0.005 rad，~15 cycles）*/
float32_t fast_atan2(float32_t y, float32_t x);

#ifdef __cplusplus
}
#endif
