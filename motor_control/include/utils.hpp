#pragma once
#include <tuple>
#include "fast_math.h"  /* our_arm_sin_f32, our_arm_cos_f32, cordic_sin_f32, fast_atan2 */

#ifdef M_PI
#undef M_PI
#endif

/* ============================================================
 * 数学常量
 * ============================================================ */
constexpr float M_PI         = 3.14159265358979323846f;
constexpr float one_by_sqrt3 = 0.57735026919f;  /* 1/sqrt(3)，Clarke 变换系数 */
constexpr float two_by_sqrt3 = 1.15470053838f;  /* 2/sqrt(3)，Clarke 变换系数 */
constexpr float sqrt3_by_2   = 0.86602540378f;  /* sqrt(3)/2，SVM 过调制边界 */

/* ============================================================
 * 内联工具函数（头文件内定义，编译器自动内联展开）
 * static inline：每个翻译单元独立副本，避免链接符号冲突
 * 外部可正常调用，static 不代表私有
 * ============================================================ */

/* x 的平方 */
template<typename T>
constexpr T SQ(const T& x) { return x * x; }

/* 四舍五入到最近整数（ARM 用 vcvtr 指令，约 1 cycle）*/
static inline int round_int(float x) {
#ifdef __arm__
    int res;
    asm("vcvtr.s32.f32 %[res], %[x]"
        : [res] "=X" (res)
        : [x]   "w"  (x));
    return res;
#else
    return (int)__builtin_roundf(x);
#endif
}

/* 限幅：将 val 钳位到 [lo, hi] */
static inline float clamp(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* 符号函数：0 视为正数（返回 +1.0），用于梯形轨迹方向判断 */
static inline float sign_hard(float val) {
    return (val < 0.0f) ? -1.0f : 1.0f;
}

/* NaN 检测（绕过 -ffast-math 的 nan 忽略优化）
 * 用于 FOC 输出合法性检查 */
__attribute__((optimize("-fno-finite-math-only")))
static inline bool is_nan(float x) {
    return __builtin_isnan(x);
}

/* 将 x 归一化到以 y 为周期的对称区间 [-y/2, y/2)
 * 例：wrap_pm(370deg, 360deg) = 10deg */
static inline float wrap_pm(float x, float y) {
    float intval = __builtin_roundf(x / y);
    return x - intval * y;
}

/* 将角度归一化到 (-pi, pi]，FOC 角度运算必用 */
static inline float wrap_pm_pi(float x) {
    return wrap_pm(x, 2.0f * M_PI);
}

/* 正数取模：结果始终 >= 0，y 必须为正数
 * 用于编码器位置计算（防止负数索引） */
static inline float fmodf_pos(float x, float y) {
    float res = wrap_pm(x, y);
    if (res < 0.0f) res += y;
    return res;
}
