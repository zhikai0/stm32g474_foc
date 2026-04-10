#pragma once

#include <optional>
#include <stdint.h>  // uint32_t, int32_t

#ifdef __cplusplus
// ============================================================
// 硬件时序常量（与 TIM1 PWM 配置保持一致）
// STM32G431 主频 170 MHz，TIM1 用于 FOC PWM 生成
// ============================================================
constexpr uint32_t TIM_1_8_CLOCK_HZ     = 170000000U; // TIM1/TIM8 时钟频率 [Hz]，与 SystemCoreClock 一致
constexpr uint32_t CURRENT_MEAS_HZ      = 20000U;     // 电流采样频率 [Hz]（= PWM 频率，中心对齐双更新）
constexpr float    current_meas_period  = 1.0f / (float)CURRENT_MEAS_HZ; // 电流采样周期 [s] = 50us

// update() 与 on_measurement() 时间戳允许的最大偏差（HCLK ticks）
// 超过此值说明主循环和中断数据不同步，FOC 返回 BAD_TIMING
constexpr int32_t MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA =
    (int32_t)(TIM_1_8_CLOCK_HZ / CURRENT_MEAS_HZ) * 2; // 允许 2 个采样周期的偏差
#endif // __cplusplus

// 三相电流快照（单位：A）
struct PhaseCurrentsABC {
    float ia;
    float ib;
    float ic;
};

// MotorControl 本地 low-level 共享状态接口。
// 职责：提供硬件采样后的“统一缓存”，供控制层/保护层读取。
// 不放具体外设驱动细节（ADC/TIM寄存器配置应放在 Core/Board 层）。
class LowLevel {
public:
    // 单例入口：保证全局只有一份 low-level 共享状态。
    static LowLevel& instance();

    // 母线电压（V）
    void set_vbus_voltage(float v);
    float vbus_voltage() const;

    // 母线电流（A）
    void set_ibus(float i);
    float ibus() const;

    // 三相电流（A）
    void set_phase_currents(const PhaseCurrentsABC& currents);
    std::optional<PhaseCurrentsABC> phase_currents() const;
    void invalidate_phase_currents();

private:
    LowLevel() = default;

    float vbus_voltage_ = 0.0f; // 母线电压
    float ibus_ = 0.0f;         // 母线电流
    std::optional<PhaseCurrentsABC> phase_currents_ = std::nullopt;
};
