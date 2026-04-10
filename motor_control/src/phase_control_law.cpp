#include "phase_control_law.hpp"
#include "interfaces.hpp"

/* ==========================================================================
 * SVM : 空间矢量调制（7 段式 SVPWM）
 *
 * 输入：alpha/beta 静止坐标系调制度矢量（幅值须 <= sqrt(3)/2 ≈ 0.866）
 * 输出：tA/tB/tC PWM 占空比 [0,1]，valid=true 表示未过调制
 *
 * 算法：将 αβ 平面分为 6 个扇区，每个扇区用相邻两基本矢量合成，
 *       零矢量时间均分到两端，形成 7 段对称波形。
 * 参考：基于幅值不变 Clarke 变换（magnitude-invariant）
 * ========================================================================== */
std::tuple<float, float, float, bool> SVM(float alpha, float beta)
{
    float tA, tB, tC;
    int sextant;            // 六分仪,扇区判断
    if (beta >= 0.0f) {
        if (alpha >= 0.0f)  // 第一象限
            sextant = (one_by_sqrt3 * beta > alpha) ? 2 : 1; // tan60°进行扇区判断
        else                // 第二象限
            sextant = (-one_by_sqrt3 * beta > alpha) ? 3 : 2;// 同上
    } else {               
        if (alpha >= 0.0f)  // 第四象限
            sextant = (-one_by_sqrt3 * beta > alpha) ? 5 : 6;
        else                // 第三象限
            sextant = (one_by_sqrt3 * beta > alpha) ? 4 : 5;
    }

    switch (sextant) {
        case 1: {
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;
            tA = (1.0f - t1 - t2) * 0.5f;
            tB = tA + t1;
            tC = tB + t2;
        } break;
        case 2: {
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;
            tB = (1.0f - t2 - t3) * 0.5f;
            tA = tB + t3;
            tC = tA + t2;
        } break;
        case 3: {
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;
            tB = (1.0f - t3 - t4) * 0.5f;
            tC = tB + t3;
            tA = tC + t4;
        } break;
        case 4: {
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;
            tC = (1.0f - t4 - t5) * 0.5f;
            tB = tC + t5;
            tA = tB + t4;
        } break;
        case 5: {
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;
            tC = (1.0f - t5 - t6) * 0.5f;
            tA = tC + t5;
            tB = tA + t6;
        } break;
        case 6: {
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;
            tA = (1.0f - t6 - t1) * 0.5f;
            tC = tA + t1;
            tB = tC + t6;
        } break;
        default:
            return {0.0f, 0.0f, 0.0f, false};
    }

    bool valid = (tA >= 0.0f && tA <= 1.0f
               && tB >= 0.0f && tB <= 1.0f
               && tC >= 0.0f && tC <= 1.0f);
    return {tA, tB, tC, valid};
}

/* ==========================================================================
 * AlphaBetaFrameController::on_measurement
 *
 * 由 Motor 层在 ADC 采样完成后调用（三相电流原始值）。
 * 内部完成 Clarke 变换（三相 → αβ），然后转发给子类。
 *
 *   Iα = Ia + 1/2*Ib + 1/2*Ic = 3/2*Ia
 *   Iβ = √3/2*(Ib - Ic)
 * Clarke 变换（幅值不变形式）上下都乘以2/3：
 *   Iα = Ia
 *   Iβ = (Ib - Ic) / √3
 *
 * 注：Ia + Ib + Ic = 0，实际只需采样两相，第三相软件推算。
 * ========================================================================== */
Motor::Error AlphaBetaFrameController::on_measurement(
        std::optional<float> vbus_voltage,// cpp17三剑客：optional有无(同类型判断有无值),variant哪个(不同类型判断哪个类型),any任意,
        std::optional<std::array<float, 3>> currents,// 同类型用array,不同类型用tuple
        uint32_t timestamp)
{
    std::optional<std::pair<float,float>> Ialpha_beta; // 同类型且2个用pair
    if (currents.has_value()) { //has_value():optional的内置成员函数,返回bool
        float Ialpha = (*currents)[0];                                    /* Iα = Ia */
        float Ibeta  = one_by_sqrt3 * ((*currents)[1] - (*currents)[2]); /* Iβ = (Ib-Ic)/√3 */
        Ialpha_beta = {Ialpha, Ibeta}; // optional类型表示可能有pair<float,float>，或者没有，pair类型用来把两个数据绑定成一个整体
    }

    return on_measurement(vbus_voltage, Ialpha_beta, timestamp);
}

/* ==========================================================================
 * AlphaBetaFrameController::get_output
 *
 * 由 Motor 层在 PWM 更新时刻调用。
 * 内部调用子类 get_alpha_beta_output 获取 αβ 调制度，
 * 然后通过 SVM 转换为三相 PWM 占空比。
 * ========================================================================== */
Motor::Error AlphaBetaFrameController::get_output(
        uint32_t output_timestamp,
        float (&pwm_timings)[3],
        std::optional<float>* ibus)
{
    std::optional<std::pair<float,float>> mod_alpha_beta; // mod:Modulation,即待调制的αβ,强调是控制量
    Motor::Error err = get_alpha_beta_output(output_timestamp, &mod_alpha_beta, ibus); // 这里会调用foc实现的逆park,即dq->αβ,
    if (err != Motor::ERROR_NONE) return err;

    if (!mod_alpha_beta.has_value()) return Motor::CONTROLLER_FAILED; // 逆park失败
    auto [mod_alpha, mod_beta] = *mod_alpha_beta; // 解引用,重载了operator*()操作符,返回<>内部的pair对象

    if (is_nan(mod_alpha) || is_nan(mod_beta)) return Motor::MODULATION_IS_NAN; // 除0错误或者上层计算错误

    auto [tA, tB, tC, success] = SVM(mod_alpha, mod_beta);
    if (!success) return Motor::MODULATION_MAGNITUDE;    // 过调制错误

    pwm_timings[0] = tA;
    pwm_timings[1] = tB;
    pwm_timings[2] = tC;

    return Motor::ERROR_NONE;
}


// // 设置三相电压(spwm)
// void setPhaseVoltage(float Ud,float Uq,float Erad){
//     float voltage_power_supply=12;

//     // 帕克逆变换
//     float Ualpha =  -Uq*sin(Erad); 
//     float Ubeta =   Uq*cos(Erad); 

//     // 克拉克逆变换
//     float Ua = Ualpha + voltage_power_supply/2;
//     float Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
//     float Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
// }

