#pragma once
#include <stdint.h>
#include <utility>  // std::pair

// ============================================================
// 通用类型别名
// ============================================================
using float2D = std::pair<float, float>;  // αβ/dq 坐标系二维矢量，first=α/d, second=β/q

// ============================================================
// 各模块枚举定义
// 用法：Motor::Error, Encoder::Mode, Axis::State 等
// 错误码为位掩码，可按位或叠加：motor_error |= Motor::CONTROLLER_FAILED;
// ============================================================
namespace Motor {

enum Error : uint64_t {
    ERROR_NONE                            = 0x000000000, // 无错误，正常运行
    PHASE_RESISTANCE_OUT_OF_RANGE         = 0x000000001, // 相电阻标定值超范围（标定失败）
    PHASE_INDUCTANCE_OUT_OF_RANGE         = 0x000000002, // 相电感标定值超范围（标定失败）
    DRV_FAULT                             = 0x000000008, // 栅极驱动器硬件故障
    CONTROL_DEADLINE_MISSED               = 0x000000010, // 控制周期超时，PWM更新时刻计算未完成
    MODULATION_MAGNITUDE                  = 0x000000080, // SVM调制度幅值超出线性区（过调制）
    CURRENT_SENSE_SATURATION              = 0x000000400, // 电流采样饱和，ADC读数超量程
    CURRENT_LIMIT_VIOLATION               = 0x000001000, // 相电流超过电流限制（current_lim）
    MODULATION_IS_NAN                     = 0x000010000, // SVM输出为NaN，通常是除零或上游计算异常
    MOTOR_THERMISTOR_OVER_TEMP            = 0x000020000, // 电机本体温度过高（外置热敏电阻）
    FET_THERMISTOR_OVER_TEMP              = 0x000040000, // MOS管（逆变器）温度过高（板载热敏电阻）
    TIMER_UPDATE_MISSED                   = 0x000080000, // TIM更新事件丢失，PWM节拍不连续
    CURRENT_MEASUREMENT_UNAVAILABLE       = 0x000100000, // 电流测量数据不可用（ADC未就绪或DMA错误）
    CONTROLLER_FAILED                     = 0x000200000, // 上层控制器（FOC/速度环）返回错误
    I_BUS_OUT_OF_RANGE                    = 0x000400000, // 母线电流超出安全范围
    BRAKE_RESISTOR_DISARMED               = 0x000800000, // 制动电阻被意外关闭（再生制动失效）
    SYSTEM_LEVEL                          = 0x001000000, // 系统级错误（由 Axis 层向下传递）
    BAD_TIMING                            = 0x002000000, // 时序错误，on_measurement/get_output调用顺序异常
    UNKNOWN_PHASE_ESTIMATE                = 0x004000000, // 电角度估算不可用（编码器未就绪）
    UNKNOWN_PHASE_VEL                     = 0x008000000, // 电角速度不可用（编码器速度无效）
    UNKNOWN_TORQUE                        = 0x010000000, // 力矩设定值不可用（上层未给定）
    UNKNOWN_CURRENT_COMMAND               = 0x020000000, // 电流指令不可用（FOC内部Idq未计算）
    UNKNOWN_CURRENT_MEASUREMENT           = 0x040000000, // 电流测量值不可用（采样结果无效）
    UNKNOWN_VBUS_VOLTAGE                  = 0x080000000, // 母线电压未知（ADC未就绪）
    UNKNOWN_VOLTAGE_COMMAND               = 0x100000000, // 电压指令未知（云台电机 GIMBAL 模式下）
    UNKNOWN_GAINS                         = 0x200000000, // 控制增益未设置（PI参数为0，标定未完成）
    CONTROLLER_INITIALIZING               = 0x400000000, // 控制器正在初始化（允许暂时不输出有效PWM）
    UNBALANCED_PHASES                     = 0x800000000, // 三相不平衡（相电流之和不为零）
};

enum Type {
    HIGH_CURRENT = 0, // 普通大电流永磁同步电机（BLDC/PMSM），力矩单位 Nm
    GIMBAL       = 2, // 云台电机（低速高精度），控制量直接为电压 V
    ACIM         = 3, // 异步感应电机，需要磁通估算
};

} // namespace Motor


namespace Encoder {

enum Error : uint32_t {
    ERROR_NONE                 = 0x00000000, // 无错误
    UNSTABLE_GAIN              = 0x00000001, // 增益不稳定（校准失败）
    CPR_POLEPAIRS_MISMATCH     = 0x00000002, // CPR 与极对数不匹配
    NO_RESPONSE                = 0x00000004, // 编码器无响应
    UNSUPPORTED_ENCODER_MODE   = 0x00000008, // 不支持的编码器模式
    ILLEGAL_HALL_STATE         = 0x00000010, // 非法霍尔状态（三相霍尔信号组合无效）
    INDEX_NOT_FOUND_YET        = 0x00000020, // 尚未找到 Z 相索引脉冲
    ABS_SPI_TIMEOUT            = 0x00000040, // 绝对值编码器 SPI 通信超时
    ABS_SPI_COM_FAIL           = 0x00000080, // 绝对值编码器 SPI 通信失败
    ABS_SPI_NOT_READY          = 0x00000100, // 绝对值编码器 SPI 未就绪
    HALL_NOT_CALIBRATED_YET    = 0x00000200, // 霍尔编码器尚未完成校准
};

enum Mode : uint32_t {
    INCREMENTAL  = 0,   // 增量式编码器（ABZ 相）
    HALL         = 1,   // 霍尔传感器（三相，6步换相）
    SINCOS       = 2,   // 正余弦模拟编码器
    SPI_ABS_CUI  = 256, // SPI 绝对值编码器：CUI AMT
    SPI_ABS_AMS  = 257, // SPI 绝对值编码器：AMS AS5047
    SPI_ABS_AEAT = 258, // SPI 绝对值编码器：Broadcom AEAT
    SPI_ABS_RLS  = 259, // SPI 绝对值编码器：RLS
    SPI_ABS_MA732= 260, // SPI 绝对值编码器：MagAlpha MA732
};

} // namespace Encoder


namespace Axis {

enum Error : uint32_t {
    ERROR_NONE                    = 0x00000000, // 无错误
    INVALID_STATE                 = 0x00000001, // 请求了无效的状态转换
    MOTOR_FAILED                  = 0x00000040, // 电机子模块报错
    SENSORLESS_ESTIMATOR_FAILED   = 0x00000080, // 无传感器估算器报错
    ENCODER_FAILED                = 0x00000100, // 编码器子模块报错
    CONTROLLER_FAILED             = 0x00000200, // 控制器子模块报错
    WATCHDOG_TIMER_EXPIRED        = 0x00000800, // 看门狗超时（通信中断）
    MIN_ENDSTOP_PRESSED           = 0x00001000, // 最小限位开关触发
    MAX_ENDSTOP_PRESSED           = 0x00002000, // 最大限位开关触发
    ESTOP_REQUESTED               = 0x00004000, // 急停信号触发
    HOMING_WITHOUT_ENDSTOP        = 0x00020000, // 未配置限位开关就执行回零
    OVER_TEMP                     = 0x00040000, // 温度超限
    UNKNOWN_POSITION              = 0x00080000, // 位置未知（未完成校准或回零）
};

enum State : uint32_t {
    UNDEFINED                        = 0,  // 未定义（上电初始状态）
    IDLE                             = 1,  // 空闲，PWM关闭
    STARTUP_SEQUENCE                 = 2,  // 启动序列（自动执行一系列校准）
    FULL_CALIBRATION_SEQUENCE        = 3,  // 完整校准序列（电机+编码器）
    MOTOR_CALIBRATION                = 4,  // 电机电阻/电感标定
    ENCODER_INDEX_SEARCH             = 6,  // 搜索编码器 Z 相索引脉冲
    ENCODER_OFFSET_CALIBRATION       = 7,  // 编码器偏置角校准
    CLOSED_LOOP_CONTROL              = 8,  // 闭环控制（正常运行模式）
    LOCKIN_SPIN                      = 9,  // 开环锁定旋转（用于编码器搜索）
    ENCODER_DIR_FIND                 = 10, // 自动检测编码器方向
    HOMING                           = 11, // 回零（寻找限位开关）
    ENCODER_HALL_POLARITY_CALIBRATION= 12, // 霍尔极性校准
    ENCODER_HALL_PHASE_CALIBRATION   = 13, // 霍尔相位校准
};

} // namespace Axis


namespace Controller {

enum Error : uint32_t {
    ERROR_NONE             = 0x00000000, // 无错误
    OVERSPEED              = 0x00000001, // 超速（速度超过限制）
    INVALID_INPUT_MODE     = 0x00000002, // 无效的输入模式
    UNSTABLE_GAIN          = 0x00000004, // 增益不稳定（可能导致振荡）
    INVALID_MIRROR_AXIS    = 0x00000008, // 无效的镜像轴配置
    INVALID_LOAD_ENCODER   = 0x00000010, // 无效的负载编码器
    INVALID_ESTIMATE       = 0x00000020, // 状态估算无效
    INVALID_CIRCULAR_RANGE = 0x00000040, // 无效的循环范围配置
    SPINOUT_DETECTED       = 0x00000080, // 检测到甩轴（失步）
};

enum ControlMode : uint32_t {
    VOLTAGE_CONTROL  = 0, // 电压控制（云台电机直接给定电压）
    TORQUE_CONTROL   = 1, // 力矩控制（电流环，输出 Nm）
    VELOCITY_CONTROL = 2, // 速度控制（速度环，输出 turn/s）
    POSITION_CONTROL = 3, // 位置控制（位置环，输出 turn）
};

enum InputMode : uint32_t {
    INACTIVE    = 0, // 输入无效（控制器不响应输入）
    PASSTHROUGH = 1, // 直通（输入直接作为设定值）
    VEL_RAMP    = 2, // 速度斜坡（以固定加速度渐变到目标速度）
    POS_FILTER  = 3, // 位置滤波（二阶低通滤波平滑位置指令）
    MIX_CHANNELS= 4, // 混合通道（多轴混控，遥控器用）
    TRAP_TRAJ   = 5, // 梯形轨迹（自动规划加减速曲线）
    TORQUE_RAMP = 6, // 力矩斜坡（以固定速率渐变到目标力矩）
    MIRROR      = 7, // 镜像模式（跟随另一个轴）
    TUNING      = 8, // 调参模式（注入正弦激励信号）
};

} // namespace Controller
