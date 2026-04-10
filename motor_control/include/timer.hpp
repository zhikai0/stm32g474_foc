#pragma once
/* timer.hpp : 统一时间服务模块
 *
 * 提供三类功能：
 *   1. DWT 初始化 + 底层 cycle 读取
 *   2. TaskTimer  : 代码段耗时测量，用 MEASURE_TIME(timer){ } 包住被测代码
 *   3. LogicTimer : 逻辑计数器（状态机超时、debounce 等，无硬件依赖）
 *
 * 平台：STM32G431，Cortex-M4，170 MHz
 * 依赖：main.h（CoreDebug / DWT / SystemCoreClock 定义）
 *
 * 使用前置条件：
 *   在 main() 或系统初始化阶段调用一次 timing_init()。
 *   TaskTimer::enabled 需在某个 .cpp 中定义：
 *     bool TaskTimer::enabled = false;
 */

#include <stdint.h>
#include <algorithm>  /* std::max, std::min */
#include "main.h"     /* CoreDebug, DWT, SystemCoreClock */

/* ============================================================
 * 1. DWT 初始化 + 基础读取工具
 * ============================================================ */

/* 启动 DWT CYCCNT 自由计数（系统启动时调用一次） */
static inline void timing_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 使能 DWT 跟踪 */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;           /* 启动 CYCCNT */
}

/* 读取当前 CPU cycle（32-bit，@ 170MHz 约 25s 溢出） */
static inline uint32_t get_cycles(void) {
    return DWT->CYCCNT;
}

/* cycles 差值转微秒（仅用于调试输出，勿在热路径调用） */
static inline float cycles_to_us(uint32_t cycles) {
    return (float)cycles / ((float)SystemCoreClock * 1e-6f);
}

/* ============================================================
 * 2. TaskTimer : 代码段耗时测量
 *
 * 用法一（手动）：
 *   uint32_t t = TaskTimer::start();
 *   ... 被测代码 ...
 *   my_timer.stop(t);
 *
 * 用法二（RAII，推荐）：
 *   MEASURE_TIME(my_timer) {
 *       ... 被测代码 ...
 *   }
 *
 * 结果读取：
 *   my_timer.last_cycles  — 上次耗时（cycles）
 *   my_timer.max_cycles   — 历史最大耗时（cycles）
 *   my_timer.last_us()    — 上次耗时（µs）
 *   my_timer.max_us()     — 历史最大耗时（µs）
 *
 * enabled 开关：
 *   enabled=false 时 stop() 不更新 last_cycles，但始终更新 max_cycles。
 *   可在 ISR 里按需置位，单轮测量后清零，对实时性零影响。
 * ============================================================ */
struct TaskTimer {
    uint32_t last_cycles = 0;
    uint32_t max_cycles  = 0;

    static bool enabled; /* 定义：bool TaskTimer::enabled = false; 放在某个 .cpp */

    static inline uint32_t start(void) {
        return DWT->CYCCNT;
    }

    inline void stop(uint32_t start_cycle) {
        uint32_t len = DWT->CYCCNT - start_cycle; /* 无符号减，自动处理溢出回绕 */
        if (enabled) {
            last_cycles = len;
        }
        max_cycles = std::max(max_cycles, len);   /* max 无论 enabled 与否都更新 */
    }

    inline void reset_max(void) { max_cycles = 0; }

    float last_us(void) const { return cycles_to_us(last_cycles); }
    float max_us(void)  const { return cycles_to_us(max_cycles);  }
};

/* RAII 作用域计时上下文（禁止拷贝/移动） */
struct TaskTimerContext {
    TaskTimerContext(const TaskTimerContext&)  = delete;
    TaskTimerContext(TaskTimerContext&&)       = delete;
    void operator=(const TaskTimerContext&)   = delete;
    void operator=(TaskTimerContext&&)        = delete;

    explicit TaskTimerContext(TaskTimer& t)
        : timer_(t), start_cycle_(TaskTimer::start()) {}
    ~TaskTimerContext() { timer_.stop(start_cycle_); }

    TaskTimer& timer_;
    uint32_t   start_cycle_;
    bool       exit_ = false;
};

/* MEASURE_TIME(timer) { ... }
 * for 体只执行一次，离开作用域时析构函数自动调用 stop() */
#define MEASURE_TIME(timer) \
    for (TaskTimerContext __tc{(timer)}; !__tc.exit_; __tc.exit_ = true)

/* ============================================================
 * 3. LogicTimer<T> : 逻辑计数器（无硬件依赖）
 *
 * 单位由调用方定义（控制周期数、ms 等）。
 * 典型用法：
 *   LogicTimer<uint32_t> wdog;
 *   wdog.setTimeout(1000);   // 1000 个控制周期后超时
 *   wdog.setIncrement(1);
 *   wdog.start();
 *   // 每个控制周期调用：
 *   wdog.update();
 *   if (wdog.expired()) { ... }
 * ============================================================ */
template<typename T>
class LogicTimer {
public:
    void setTimeout(const T v)   { timeout_   = v; }
    void setIncrement(const T v) { increment_ = v; }
    void start()  { running_ = true; }
    void stop()   { running_ = false; }
    void reset()  { timer_ = static_cast<T>(0); }

    void update() {
        if (running_)
            timer_ = std::min<T>(timer_ + increment_, timeout_);
    }

    bool expired() const { return timer_ >= timeout_; }
    T    current() const { return timer_; }

private:
    T    timer_     = static_cast<T>(0);
    T    timeout_   = static_cast<T>(0);
    T    increment_ = static_cast<T>(0);
    bool running_   = false;
};
