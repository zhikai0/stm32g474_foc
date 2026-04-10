#pragma once
#include <stdint.h>
#include <optional>
#include <array>
#include "utils.hpp"
#include "interfaces.hpp"

/* ==========================================================================
 * PhaseControlLaw<N> : N 相控制律抽象接口
 *
 * 所有控制律（FOC、开环、电阻标定）都实现此接口。
 * Motor 层只持有 PhaseControlLaw<3>* 指针，不关心具体实现。
 *
 * 调用时序（由 TIM1 中断驱动，每个 PWM 周期）：
 *   1. ADC 采样完成 → on_measurement()  ← 存储测量值
 *   2. PWM 更新时刻 → get_output()      ← 计算并输出 PWM
 why?
 因为foc控制需要AlphaBeta的控制。所以直接继承这个，
 可能还有其他非三相星型连接的控制方式，这样就没法用AlphaBetaFrameController和svm
 上面再加一个顶层的抽象父类，PhaseControlLaw相位控制法则,motor就可以不需要关心具体怎么实现,
 不同控制方式只需增加新controller，更新对应实现就可
 * ========================================================================== */
template<size_t N_PHASES>
class PhaseControlLaw {
public:
    /*
        #####三种析构函数声明方式的对比#####
        1.virtual ~Base()=default；基类的虚析构函数,删除基类指针的时候会先执行子类析构，再执行父类析构
        2.~Base()=default；基类的析构函数,删除基类指针的时候不会执行子类析构,只会执行父类析构,会泄露资源
        3.virtual ~Base()=0；基类的纯析构函数,和虚析构类似,不过强制子类要实现,
    */
    virtual ~PhaseControlLaw() = default; // 基类的虚析构函数,删除基类指针的时候会先执行子类析构，再执行父类析构
    virtual void reset() = 0;   // 实例化的子类(即new的对象)必须实现,AlphaBetaFrameController有纯虚函数属于抽象类,不需要实现

    /* ADC 采样完成时调用（高优先级中断上下文，必须快速返回）
     * @param vbus_voltage  母线电压 [V]，测量无效时为 nullopt
     * @param currents      各相电流 [A]，测量无效时为 nullopt
     * @param timestamp     采样时刻（HCLK ticks）
     * @return              ERROR_NONE 或错误码（触发电机下使能）*/
    virtual Motor::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<std::array<float, N_PHASES>> currents,
            uint32_t timestamp) = 0;

    /* PWM 更新时刻调用（高优先级中断上下文，必须快速返回）
     * @param output_timestamp  本次 PWM 作用的中心时刻（HCLK ticks）
     * @param pwm_timings       输出：各相 PWM 占空比 [0.0, 1.0]
     * @param ibus              输出：估算的母线电流 [A]
     * @return                  ERROR_NONE 或错误码 */
    virtual Motor::Error get_output(
            uint32_t output_timestamp,
            float (&pwm_timings)[N_PHASES],
            std::optional<float>* ibus) = 0; // virtual:虚函数关键词;lvirtual fn=0;c纯虚函数标志,纯虚函数必须重写,纯虚函数对应的类就是抽象类
};

/* ==========================================================================
 * AlphaBetaFrameController : 在 PhaseControlLaw<3> 基础上
 * 封装 Clarke 变换（三相→αβ）和 SVM（αβ→三相PWM）
 *
 * 子类只需实现 αβ 坐标系下的控制逻辑，无需关心三相和PWM细节：
 *   - on_measurement(vbus, Ialpha_beta, timestamp)    ← 存储 αβ 电流
 *   - get_alpha_beta_output(timestamp, mod_ab, ibus)  ← 输出 αβ 调制度
 * ========================================================================== */
class AlphaBetaFrameController : public PhaseControlLaw<3> {
public:
    /* 由父接口调用，内部做 Clarke 变换后转发给子类 */
    Motor::Error on_measurement(
        std::optional<float> vbus_voltage,
        std::optional<std::array<float, 3>> currents,
        uint32_t timestamp) final;

    /* 由父接口调用，内部调用子类 get_alpha_beta_output 后做 SVM */
    Motor::Error get_output(
        uint32_t output_timestamp,
        float (&pwm_timings)[3],
        std::optional<float>* ibus) final; //final（禁止再被重写）

protected:
    // 子类需要实现,例如foc的FieldOrientedController(磁场定向控制类),这样的好处是统一接口
    virtual Motor::Error on_measurement(
        std::optional<float> vbus_voltage,
        std::optional<std::pair<float,float>> Ialpha_beta,
        uint32_t timestamp) = 0;

    virtual Motor::Error get_alpha_beta_output(
        uint32_t output_timestamp,
        std::optional<std::pair<float,float>>* mod_alpha_beta,
        std::optional<float>* ibus) = 0;
};
