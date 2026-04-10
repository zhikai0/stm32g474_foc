#pragma once

#include "phase_control_law.hpp"  // AlphaBetaFrameController
#include "component.hpp"           // ComponentBase, InputPort
#include "utils.hpp"               // float2D = std::pair<float,float>
#include "interfaces.hpp"          // Motor::Error, float2D

/*
 * FieldOrientedController：磁场定向控制（FOC）
 *
 * 继承 AlphaBetaFrameController（负责 Clarke 变换和 SVM）
 * 继承 ComponentBase（由主循环每周期调用 update()）
 *
 * 数据流：
 *   主循环（低优先级）：update() 从 InputPort 读取设定值，原子写入内部变量
 *   ADC中断（高优先级）：on_measurement() 存储电流/电压测量值
 *   PWM中断（高优先级）：get_alpha_beta_output() 执行 Park/反Park + PI 控制，输出调制度
 *
 * 工作模式：
 *   enable_current_control_=true  → 电流控制模式（需要 pi_gains_ 和 Idq_setpoint_）
 *   enable_current_control_=false → 电压控制模式（直接用 Vdq_setpoint_）
 */
class FieldOrientedController : public AlphaBetaFrameController, public ComponentBase {
public:
    // ComponentBase 接口：每个控制周期由主循环调用，原子地更新控制输入
    void update(uint32_t timestamp) final;

    // PhaseControlLaw 接口：重置积分器和测量缓存
    void reset() final; // final:禁止重写;状态重置,积分器清零,测量缓存清零,由于继承AlphaBetaFrameController，所以必须实现这个纯虚函数

    // AlphaBetaFrameController 接口：存储 αβ 电流和母线电压测量值
    Motor::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<float2D> Ialpha_beta,
            uint32_t input_timestamp) final;

    // AlphaBetaFrameController 接口：执行 FOC 计算，输出 αβ 调制度
    Motor::Error get_alpha_beta_output(
            uint32_t output_timestamp,
            std::optional<float2D>* mod_alpha_beta,
            std::optional<float>* ibus) final;

    // ---- 配置（控制器非活动时设置）----
    std::optional<float2D> pi_gains_; // [V/A, V/As] PI 增益，标定后由 Motor 层写入
    float I_measured_report_filter_k_ = 1.0f; // 电流滤波系数（1.0=不滤波）

    // ---- 输入端口（由主循环/上层控制器连接）----
    bool enable_current_control_src_ = false;  // 工作模式源（主循环写，中断读）
    InputPort<float2D> Idq_setpoint_src_;       // dq 电流设定值 [A]（电流模式）
    InputPort<float2D> Vdq_setpoint_src_;       // dq 电压设定值 [V]（前馈或电压模式）
    InputPort<float> phase_src_;               // 电角度 [rad]（来自编码器或估算器）
    InputPort<float> phase_vel_src_;           // 电角速度 [rad/s]

    // ---- 内部状态（update() 原子写，中断读）----
    uint32_t ctrl_timestamp_ = 0;         // 主循环时间戳 [HCLK ticks]
    bool enable_current_control_ = false; // 工作模式快照
    std::optional<float2D> Idq_setpoint_;  // dq 电流设定值快照 [A]
    std::optional<float2D> Vdq_setpoint_;  // dq 电压设定值快照 [V]
    std::optional<float> phase_;           // 电角度快照 [rad]
    std::optional<float> phase_vel_;       // 电角速度快照 [rad/s]

    // ---- 测量值（on_measurement() 写，get_alpha_beta_output() 读）----
    uint32_t i_timestamp_ = 0;                    // 电流采样时间戳 [HCLK ticks]
    std::optional<float> vbus_voltage_measured_;  // 母线电压 [V]
    std::optional<float2D> Ialpha_beta_measured_; // αβ 电流 [A]
    float Id_measured_ = 0.0f;  // d 轴电流滤波值 [A]（用于上报）
    float Iq_measured_ = 0.0f;  // q 轴电流滤波值 [A]（用于上报）

    // ---- PI 积分器（get_alpha_beta_output() 读写）----
    float v_current_control_integral_d_ = 0.0f; // d 轴积分项 [V]
    float v_current_control_integral_q_ = 0.0f; // q 轴积分项 [V]

    // ---- 输出（供无传感器估算器使用）----
    float final_v_alpha_ = 0.0f; // 最终施加的 α 轴电压 [V]
    float final_v_beta_  = 0.0f; // 最终施加的 β 轴电压 [V]
    float power_         = 0.0f; // 估算功率 [W] = Vd*Id + Vq*Iq
};
