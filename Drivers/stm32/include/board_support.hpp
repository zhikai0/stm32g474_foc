#pragma once

#include <stdio.h>

// sleep(s) - 以秒为单位延时，用法类似 Python 的 time.sleep
// 例：sleep(0.5) => HAL_Delay(500)
#define sleep(s) HAL_Delay((uint32_t)((s) * 1000.0))

// 软件定时器宏，需要放到主循环内（非阻塞运行,绝对时间戳,只要执行间隔 < 计数器半周期）
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static uint32_t last_##__LINE__ = 0; \
    uint32_t now_##__LINE__ = HAL_GetTick(); \
    if (now_##__LINE__ - last_##__LINE__ >= (MS)) { \
        X; \
        last_##__LINE__ = now_##__LINE__; \
    } \
} while(0)

#define EXECUTE_EVERY_N_US(US, X) do { \
    static uint32_t last_##__LINE__ = 0; \
    uint32_t now_##__LINE__ = micros(); \
    if (now_##__LINE__ - last_##__LINE__ >= (US)) { \
        X; \
        last_##__LINE__ = now_##__LINE__; \
    } \
} while(0)

#ifdef __cplusplus
extern "C" {
#endif
// HAL 外设头文件（板级依赖，统一在此 include）
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

void SystemClock_Config(void); // 时钟初始化
uint32_t micros(void);        // 读取自启动以来的微秒时间戳
void delay_us(uint32_t us);   // us 级忙等延时
void print_memory_usage(void); // 打印 FLASH/RAM 使用情况
void board_init(void);         // 板级初始化：HAL + 时钟 + 所有外设
bool can_start(void);         //
void can_test(void);         //

#ifdef __cplusplus
}
#endif

/* ==========================================================================
 * print模板
 *
 * 目前支持以下几种写法：
 * 1) print();
 *    - 只打印换行
 *
 * 2) print("hello");
 *    - 打印普通字符串并换行
 *
 * 3) print("x=%d y=%.3f", x, y);
 *    - 如果首字符串里包含 printf 风格的 % 占位符，则按 printf 方式打印
 *
 * 4) print("adc", raw, voltage, ntc_r, temp_c);
 *    - 如果首字符串里不包含 % / {}，则按“普通值列表”打印
 *    - 默认使用 PRINT_VALUE_SEPARATOR 作为分隔符
 *
 * 5) print("adc: raw={} voltage={} ntc_r={} temp_c={}", raw, voltage, ntc_r, temp_c);
 *    - 如果首字符串里包含 {}，则按轻量花括号占位替换打印
 *
 * 当前自动推导 print_value() 支持的值类型：
 * - bool                  -> true / false
 * - 整型(有符号/无符号)   -> %ld / %lu
 * - 浮点(float/double)    -> %.3f
 * - const char* / 字符串字面量 -> %s
 * - 其他类型              -> 按地址 %p 打印
 *
 * 注意：
 * - C++ 模板版主要面向 STM32 上常用的 bool / 整型 / 浮点 / C字符串。
 * - 如果后续要直接打印 std::string、enum class、自定义类、容器等，建议继续补充
 *   print_value() 的分支，或者显式转成基础类型/const char* 再传入。
 * ========================================================================== */
#ifdef __cplusplus
#include <array>
#include <cstdio>
#include <tuple>
#include <type_traits>

// print的分割符的宏定义默认是和py一样" "
#ifndef PRINT_VALUE_SEPARATOR   
#define PRINT_VALUE_SEPARATOR ","
#endif


inline void print() {
    std::printf("\r\n");
}

template<typename T>
inline void print_value(const T& value) {
    using D = std::decay_t<T>;

    if constexpr (std::is_same_v<D, bool>) {
        std::printf("%s", value ? "true" : "false");
    } else if constexpr (std::is_integral_v<D>) {
        if constexpr (std::is_unsigned_v<D>) {
            std::printf("%lu", static_cast<unsigned long>(value));
        } else {
            std::printf("%ld", static_cast<long>(value));
        }
    } else if constexpr (std::is_floating_point_v<D>) {
        std::printf("%.3f", static_cast<double>(value));
    } else if constexpr (std::is_convertible_v<T, const char*>) {
        std::printf("%s", static_cast<const char*>(value));
    } else {
        std::printf("%p", static_cast<const void*>(&value));
    }
}

template<typename T, size_t N>
inline void print_value(const T (&arr)[N]) {
    std::printf("[");
    for (size_t i = 0; i < N; ++i) {
        if (i > 0) {
            std::printf(", ");
        }
        print_value(arr[i]);
    }
    std::printf("]");
}

template<typename T, size_t N>
inline void print_value(const std::array<T, N>& arr) {
    std::printf("[");
    for (size_t i = 0; i < N; ++i) {
        if (i > 0) {
            std::printf(", ");
        }
        print_value(arr[i]);
    }
    std::printf("]");
}

template<size_t I = 0, typename... Ts>
inline void print_tuple_elements(const std::tuple<Ts...>& tup) {
    if constexpr (I < sizeof...(Ts)) {
        if constexpr (I > 0) {
            std::printf(", ");
        }
        print_value(std::get<I>(tup));
        print_tuple_elements<I + 1>(tup);
    }
}

template<typename... Ts>
inline void print_value(const std::tuple<Ts...>& tup) {
    std::printf("(");
    print_tuple_elements(tup);
    std::printf(")");
}

inline bool print_has_printf_tokens(const char* s) {
    for (; *s != '\0'; ++s) {
        if (*s == '%') {
            if (*(s + 1) == '%') {
                ++s;
                continue;
            }
            return true;
        }
    }
    return false;
}

inline bool print_has_brace_tokens(const char* s) {
    for (; *s != '\0'; ++s) {
        if (s[0] == '{' && s[1] == '}') {
            return true;
        }
    }
    return false;
}

inline void print_space_impl() {}

template<typename T>
inline void print_space_impl(const T& value) {
    print_value(value);
}

template<typename T, typename... Rest>
inline void print_space_impl(const T& value, const Rest&... rest) {
    print_value(value);
    std::printf("%s", PRINT_VALUE_SEPARATOR);
    print_space_impl(rest...);
}

inline void print_brace_impl(const char* fmt) {
    std::printf("%s", fmt);
}

template<typename T, typename... Rest>
inline void print_brace_impl(const char* fmt, const T& value, const Rest&... rest) {
    while (*fmt != '\0') {
        if (fmt[0] == '{' && fmt[1] == '}') {
            print_value(value);
            print_brace_impl(fmt + 2, rest...);
            return;
        }
        std::printf("%c", *fmt++);
    }
}

template<typename... Args>
inline void print(const char* first, const Args&... args) {
    if constexpr (sizeof...(args) == 0) {
        std::printf("%s\r\n", first);
    } else if (print_has_brace_tokens(first)) {
        print_brace_impl(first, args...);
        std::printf("\r\n");
    } else if (print_has_printf_tokens(first)) {
        std::printf(first, args...);
        std::printf("\r\n");
    } else {
        print_space_impl(first, args...);
        std::printf("\r\n");
    }
}

template<typename T, typename... Rest,
         typename = std::enable_if_t<!std::is_convertible_v<T, const char*>>>
inline void print(const T& value, const Rest&... rest) {
    print_space_impl(value, rest...);
    std::printf("\r\n");
}
#else
// print(fmt, ...) - 自动追加 \r\n，用法同 printf
// 例：print("val=%d", x)  =>  printf("val=%d\r\n", x)
#define print(fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
#endif


