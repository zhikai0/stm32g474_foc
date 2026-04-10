#pragma once

#include "stm32g4xx.h"  // __get_PRIMASK(), __disable_irq(), __set_PRIMASK() 等 Cortex-M 内核寄存器操作
#include <stdint.h>       // uint32_t

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t cpu_enter_critical() {
    uint32_t primask = __get_PRIMASK(); // 1. 读取当前状态（可能是0或1），表示中断当前是否开启，1表示中断关闭
    __disable_irq();                    // 2. 强制关中断请求，IRQ = Interrupt ReQuest
    return primask;                     // 3. 返回旧值
}

static inline void cpu_exit_critical(uint32_t priority_mask) {
    __set_PRIMASK(priority_mask);        // 4. 恢复之前的状态,如果本来中断就是开启的，有了保存的状态值，这里执行完会再次开启，而不是固定关闭
}

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
/*
cpp的"Rule of Five",cpp11加入move移动语义,之前只有三规则,move效率快
1.析构函数
2.拷贝构造 ->复制对象，深拷贝
3.拷贝赋值
4.移动构造 ->提升性能,只转移指针
5.移动赋值
*/
// #####IRQ 并发保护#####
struct CriticalSectionContext {
    CriticalSectionContext(const CriticalSectionContext&) = delete; // 禁止通过拷贝构造创建新对象
    CriticalSectionContext(const CriticalSectionContext&&) = delete;// 禁止通过移动构造创建新对象
    void operator=(const CriticalSectionContext&) = delete;         // 禁止对已有对象进行拷贝赋值
    void operator=(const CriticalSectionContext&&) = delete;        // 禁止对已有对象进行移动赋值
    operator bool() { return true; };                          //  隐式 bool 转换
    CriticalSectionContext() : mask_(cpu_enter_critical()) {}   // 构造器先调用cpu_enter_critical(),返回值给到mask_保存
    ~CriticalSectionContext() { cpu_exit_critical(mask_); }     // 析构时调用cpu_exit_critical，传入之前保存的mask
    uint32_t mask_;
    bool exit_ = false; // clang编译器会用
};

#ifdef __clang__
#define CRITICAL_SECTION() for (CriticalSectionContext __critical_section_context; !__critical_section_context.exit_; __critical_section_context.exit_ = true)
#else
#define CRITICAL_SECTION() if (CriticalSectionContext __critical_section_context{})//CriticalSectionContext构造时关中断
#endif

#endif
