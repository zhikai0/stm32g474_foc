#include "arm_common_tables.h"  /* 提供 sinTable_f32[]：512+1 个 float 构成的正弦查找表 */
#include "stm32g4xx_ll_cordic.h" /* STM32G4 CORDIC 协处理器 LL 驱动 */
#include "fast_math.h"          /* 本模块对外接口声明*/
/* ==========================================================================
 * our_arm_sin_f32 : 查表法 + 线性插值计算 sin(x)
 * 精度：误差约 1e-4；速度：约 60 cycles @ 170MHz
 * ========================================================================== */
float32_t our_arm_sin_f32(float32_t x){
    uint16_t N = FAST_MATH_TABLE_SIZE;      // 表内存储的采样点数量
    float32_t norm_x = x * 0.159154943092f; // 0.159154943092 =1/2pi,如果外部x在0-2pi区间则norm_x在0-1区间
    int32_t floor_x = (int32_t)norm_x;      // 强制类型转换的规则是向0取整
    if (norm_x<0.0f){floor_x--;}                 // 需要减1,例如 int(0.1) = 0,int(-0.1) = int(-1.1)= -1
    // 把外部x数据映射到0-1区间,对应norm(x)到[0,2pi]
    // norm_x负数表示反向,例如 0.2 = -0.8-(-1),1圈等效于norm_正+norm_x,norm_正即我们需要的[0,1]的归一化数值
    norm_x = norm_x - (float32_t)floor_x;   // norm_x是通用的归一化后的x
    float32_t fidx = norm_x * N;    // 用于后续插值
    uint16_t idx = (uint16_t)fidx;      
    if(idx>=N){   // idx在[0,N-1],所以idx=N时需要回绕到0
        idx = 0;
        fidx = fidx - (float32_t)N;  // 回绕到0后,fidx需要减去N
    }
    float32_t fract = fidx - (float32_t)idx;  // 转换成相同类型计算小数部分
    // 查表计算正弦值
    float32_t a = sinTable_f32[idx];
    float32_t b = sinTable_f32[idx+1];
    float32_t sinVal = (1.0f - fract) * a + fract * b; // 线性插值,x =a+(b-a)/(idx+1-idx)*fract,所以sinVal = a+(b-a)*fract
    return sinVal;
}

float32_t our_arm_cos_f32(float32_t x){
    uint16_t N = FAST_MATH_TABLE_SIZE;      
    float32_t norm_x = x * 0.159154943092f + 0.25f; // 偏移0.25，对应cos(x) = sin(x + pi/2)
    int32_t floor_x = (int32_t)norm_x;      
    if (norm_x<0.0f){floor_x--;}                 
    norm_x = norm_x - (float32_t)floor_x;   
    float32_t fidx = norm_x * N;  
    uint16_t idx = (uint16_t)fidx;      
    if(idx>=N){   
        idx = 0;
        fidx = fidx - (float32_t)N; 
    }
    float32_t fract = fidx - (float32_t)idx; 
    // 查表计算正弦值
    float32_t a = sinTable_f32[idx];
    float32_t b = sinTable_f32[idx+1];
    float32_t cosVal = (1.0f - fract) * a + fract * b; 
    return cosVal;
}

// based on https://math.stackexchange.com/a/1105038/81278
float32_t fast_atan2(float32_t y, float32_t x) {
    float32_t abs_y = (y < 0.0f) ? -y : y;
    float32_t abs_x = (x < 0.0f) ? -x : x;
    /* inject FLT_MIN to avoid division by zero */
    float32_t a = (abs_x < abs_y ? abs_x : abs_y) /
              ((abs_x > abs_y ? abs_x : abs_y) + 1.175494351e-38f);
    float32_t s = a * a;
    float32_t r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (abs_y > abs_x)  r = 1.57079637f - r;
    if (x < 0.0f)       r = 3.14159274f - r;
    if (y < 0.0f)       r = -r;
    return r;
}
/* ==========================================================================
 * cordic_sin_f32 : 硬件 CORDIC 轮询模式，单次调用
 *
 * STM32G4 内置 CORDIC 协处理器，专门加速三角函数计算
 * 适合 FOC 实时控制环（每控制周期算 1~2 个 sin/cos）
 * 速度：~48 cycles @ 170MHz（0.28 us），比查表快约 20%
 * 精度：PRECISION=4cycles 对应约 20bit（误差 ~1e-6），优于查表法
 * 需先调用 MX_CORDIC_DMA_Init() 完成 CORDIC CSR 寄存器配置
 * ========================================================================== */
float32_t cordic_sin_f32(float32_t x) {
    /* CORDIC 输入为 q1.31 格式，[-pi, pi] 映射到 [-1, 1]，即除以 pi */
    float32_t angle_norm = x * (float32_t)(1.0 / M_PI);

    /* 限幅到 [-1, 1]，防止 q1.31 转换溢出 */
    if (angle_norm >  1.0f) angle_norm =  1.0f;
    if (angle_norm < -1.0f) angle_norm = -1.0f;

    /* float -> q1.31：乘以 2^31（0x7FFFFFFF 对应 +1.0） */
    int32_t arg_q31 = (int32_t)(angle_norm * 2147483648.0f);

    /* 写 WDATA，触发 CORDIC 硬件开始计算（后台异步执行） */
    LL_CORDIC_WriteData(CORDIC, (uint32_t)arg_q31);

    /* 轮询 RRDY 标志，等待计算完成（PRECISION=4cycles，约 4 AHB 周期） */
    while (!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)) {}

    /* 读取 RDATA，q1.31 格式的 sin 结果 */
    int32_t res_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);

    /* q1.31 -> float：除以 2^31 还原到 [-1, 1] */
    return (float32_t)res_q31 / 2147483648.0f;
}

