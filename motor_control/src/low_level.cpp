#include "low_level.hpp"

LowLevel& LowLevel::instance() { // LowLevel对象通过instance函数实例化后只有程序退出后LowLevel对象才会被销毁
    static LowLevel self;
    return self;
}

void LowLevel::set_vbus_voltage(float v) {
    vbus_voltage_ = v;
}

float LowLevel::vbus_voltage() const {
    return vbus_voltage_;
}

void LowLevel::set_ibus(float i) {
    ibus_ = i;
}

float LowLevel::ibus() const {
    return ibus_;
}

void LowLevel::set_phase_currents(const PhaseCurrentsABC& currents) {
    phase_currents_ = currents;
}

std::optional<PhaseCurrentsABC> LowLevel::phase_currents() const {
    return phase_currents_;
}

void LowLevel::invalidate_phase_currents() {
    phase_currents_ = std::nullopt;
}
