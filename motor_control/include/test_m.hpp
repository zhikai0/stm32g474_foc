#pragma once

#include <optional>
#include <utility>
#include "phase_control_law.hpp"

class DemoController : public AlphaBetaFrameController {
public:
    void reset() override;

    Motor::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<std::pair<float, float>> Ialpha_beta,
            uint32_t timestamp) override;

    Motor::Error get_alpha_beta_output(
            uint32_t output_timestamp,
            std::optional<std::pair<float, float>>* mod_alpha_beta,
            std::optional<float>* ibus) override;

    void set_modulation(float alpha, float beta);

    float Ialpha_ = 0.0f;
    float Ibeta_ = 0.0f;
    float vbus_ = 0.0f;

private:
    float mod_alpha_ = 0.3f;
    float mod_beta_ = 0.1f;
};
