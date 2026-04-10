#include "foc.hpp"
#include "utils.hpp"    // our_arm_sin_f32, our_arm_cos_f32, is_nan, sqrt3_by_2
#include "low_level.hpp" // TIM_1_8_CLOCK_HZ, current_meas_period, MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA
#include "system.h"     // CRITICAL_SECTION()
#include <cstdlib>     // abs
#include <cmath>       // std::sqrt


/* --------------------------------------------------------------------------
 * 状态重置,Motor::arm(PhaseControlLaw<3>* control_law)函数内通过control_law_->reset()调用
 * -------------------------------------------------------------------------- */
void FieldOrientedController::reset() {
    v_current_control_integral_d_ = 0.0f;
    v_current_control_integral_q_ = 0.0f;
    vbus_voltage_measured_ = std::nullopt;  // std::optional<T>类型；无值状态
    Ialpha_beta_measured_  = std::nullopt;
    power_ = 0.0f;
}

/* --------------------------------------------------------------------------
 * maincpp controllerloop周期性调用,调用on_measurement()和get_alpha_beta_output()
 * CRITICAL_SECTION 确保中断不会读到半更新的数据。
 * -------------------------------------------------------------------------- */
void FieldOrientedController::update(uint32_t timestamp) {
    CRITICAL_SECTION() {    // 临时关闭中断,宏作用域结束后恢复原中断状态,目的是防止中断读到半更新的数据,
        ctrl_timestamp_        = timestamp; //例如浮点数,4字节,写到2字节触发中断并且需要这个数来做计算,就会导致错误
        enable_current_control_= enable_current_control_src_;
        Idq_setpoint_          = Idq_setpoint_src_.present();  // 电流设定值 [A]，来自速度/位置环
        Vdq_setpoint_          = Vdq_setpoint_src_.present();  // 电压前馈 [V]
        phase_                 = phase_src_.present();         // 电角度 [rad]，来自编码器
        phase_vel_             = phase_vel_src_.present();     // 电角速度 [rad/s]
    }
}

/* --------------------------------------------------------------------------
 * on_measurement()：ADC 采样完成中断调用（高优先级）
 *
 * 父类 AlphaBetaFrameController 已完成 Clarke 变换（三相→αβ），
 * 这里只需存储 αβ 电流和母线电压，供 get_alpha_beta_output() 使用。
 * -------------------------------------------------------------------------- */
Motor::Error FieldOrientedController::on_measurement(
        std::optional<float> vbus_voltage,
        std::optional<float2D> Ialpha_beta,
        uint32_t input_timestamp) {
    i_timestamp_           = input_timestamp;
    vbus_voltage_measured_ = vbus_voltage;
    Ialpha_beta_measured_  = Ialpha_beta;
    return Motor::ERROR_NONE;
}

/* --------------------------------------------------------------------------
 * get_alpha_beta_output()：PWM 更新中断调用（高优先级）
 *
 * 执行完整的 FOC 计算链：
 *   1. 检查数据有效性（时序、设定值、电压）
 *   2. Park 变换（αβ → dq）
 *   3. PI 电流控制器（或直接电压控制）
 *   4. 调制度饱和限幅 + 积分器抗饱和
 *   5. 反 Park 变换（dq → αβ），输出调制度
 * -------------------------------------------------------------------------- */
Motor::Error FieldOrientedController::get_alpha_beta_output(
        uint32_t output_timestamp,
        std::optional<float2D>* mod_alpha_beta,
        std::optional<float>* ibus) {

    // 1. 检查测量值是否就绪
    if (!vbus_voltage_measured_.has_value() || !Ialpha_beta_measured_.has_value()) {
        return Motor::CONTROLLER_INITIALIZING; // FOC 尚未收到电流测量值
    }

    // 检查主循环时间戳与电流采样时间戳的偏差
    if (abs((int32_t)(i_timestamp_ - ctrl_timestamp_)) > MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA) {
        return Motor::BAD_TIMING; // 主循环和中断数据不同步
    }

    // 检查设定值
    if (!Vdq_setpoint_.has_value()) {
        return Motor::UNKNOWN_VOLTAGE_COMMAND;
    }
    if (!phase_.has_value() || !phase_vel_.has_value()) {
        return Motor::UNKNOWN_PHASE_ESTIMATE;
    }

    auto [Vd, Vq]           = *Vdq_setpoint_;
    float phase             = *phase_;
    float phase_vel         = *phase_vel_;
    float vbus_voltage      = *vbus_voltage_measured_;

    // 2. Park 变换：αβ → dq（需要补偿电流采样与控制时间戳的相位差）
    std::optional<float2D> Idq;
    if (Ialpha_beta_measured_.has_value()) {
        auto [Ialpha, Ibeta] = *Ialpha_beta_measured_;
        // 补偿电流采样时刻与控制时刻的相位差（时间差 × 电角速度）
        float I_phase = phase + phase_vel * ((float)(int32_t)(i_timestamp_ - ctrl_timestamp_)
                                              / (float)TIM_1_8_CLOCK_HZ);
        float c_I = our_arm_cos_f32(I_phase);
        float s_I = our_arm_sin_f32(I_phase);
        // Park 变换：Id = cos*Iα + sin*Iβ，Iq = cos*Iβ - sin*Iα
        Idq = { c_I * Ialpha + s_I * Ibeta,
                c_I * Ibeta  - s_I * Ialpha };
        // 一阶低通滤波更新上报电流（I_measured_report_filter_k_=1.0 时无滤波）
        Id_measured_ += I_measured_report_filter_k_ * (Idq->first  - Id_measured_);
        Iq_measured_ += I_measured_report_filter_k_ * (Idq->second - Iq_measured_);
    } else {
        Id_measured_ = 0.0f;
        Iq_measured_ = 0.0f;
    }

    // 调制度到电压的换算系数（幅值不变 Clarke 变换下：mod=1 对应 2/3 * Vbus）
    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d, mod_q;

    if (enable_current_control_) {
        // 3a. 电流控制模式：PI 控制器
        if (!pi_gains_.has_value()) {
            return Motor::UNKNOWN_GAINS;
        }
        if (!Idq.has_value()) {
            return Motor::UNKNOWN_CURRENT_MEASUREMENT;
        }
        if (!Idq_setpoint_.has_value()) {
            return Motor::UNKNOWN_CURRENT_COMMAND;
        }

        auto [p_gain, i_gain]       = *pi_gains_;
        auto [Id, Iq]               = *Idq;
        auto [Id_setpoint, Iq_setpoint] = *Idq_setpoint_;

        float Ierr_d = Id_setpoint - Id; // d 轴电流误差
        float Ierr_q = Iq_setpoint - Iq; // q 轴电流误差

        // PI 输出（Vdq_setpoint_ 作为前馈项）
        mod_d = V_to_mod * (Vd + v_current_control_integral_d_ + Ierr_d * p_gain);
        mod_q = V_to_mod * (Vq + v_current_control_integral_q_ + Ierr_q * p_gain);

        // 4. 调制度饱和限幅 + 积分器抗饱和（anti-windup）
        // 最大调制度设为 0.8 * sqrt(3)/2（留余量防过调制）
        float mod_scalefactor = 0.80f * sqrt3_by_2 / std::sqrt(mod_d * mod_d + mod_q * mod_q);
        if (mod_scalefactor < 1.0f) {
            // 过调制：等比缩放，积分器缓慢衰减（抗积分饱和）
            mod_d *= mod_scalefactor;
            mod_q *= mod_scalefactor;
            v_current_control_integral_d_ *= 0.99f; // 缓慢衰减积分项
            v_current_control_integral_q_ *= 0.99f;
        } else {
            // 正常区：积分器正常累积
            v_current_control_integral_d_ += Ierr_d * (i_gain * current_meas_period);
            v_current_control_integral_q_ += Ierr_q * (i_gain * current_meas_period);
        }
    } else {
        // 3b. 电压控制模式（云台电机/开环测试）：直接换算，无 PI
        mod_d = V_to_mod * Vd;
        mod_q = V_to_mod * Vq;
    }

    // 5. 反 Park 变换：dq → αβ（补偿输出时刻与控制时刻的相位差）
    float pwm_phase = phase + phase_vel * ((float)(int32_t)(output_timestamp - ctrl_timestamp_)
                                            / (float)TIM_1_8_CLOCK_HZ);
    float c_p = our_arm_cos_f32(pwm_phase);
    float s_p = our_arm_sin_f32(pwm_phase);
    // 反 Park：mod_α = cos*mod_d - sin*mod_q，mod_β = sin*mod_d + cos*mod_q
    float mod_alpha = c_p * mod_d - s_p * mod_q;
    float mod_beta  = s_p * mod_d + c_p * mod_q;

    // 记录最终施加的静止坐标系电压（供无传感器估算器使用）
    final_v_alpha_ = mod_to_V * mod_alpha;
    final_v_beta_  = mod_to_V * mod_beta;

    *mod_alpha_beta = {mod_alpha, mod_beta};

    // 估算母线电流：ibus = mod_d*Id + mod_q*Iq
    if (Idq.has_value()) {
        auto [Id, Iq] = *Idq;
        *ibus  = mod_d * Id + mod_q * Iq;
        power_ = vbus_voltage * (*ibus).value();
    }

    return Motor::ERROR_NONE;
}
