#include "test_m.hpp"

void DemoController::reset() {
  mod_alpha_ = 0.0f;
  mod_beta_ = 0.0f;
  vbus_ = 0.0f;
}

Motor::Error DemoController::on_measurement(
    std::optional<float> vbus_voltage,
    std::optional<std::pair<float, float>> Ialpha_beta,
    uint32_t timestamp) {
  (void)timestamp;

  if (vbus_voltage.has_value()) {
    vbus_ = *vbus_voltage;
  }
  if (Ialpha_beta.has_value()) {
    Ialpha_ = Ialpha_beta->first;
    Ibeta_ = Ialpha_beta->second;
  }
  return Motor::ERROR_NONE;
}

Motor::Error DemoController::get_alpha_beta_output(
    uint32_t output_timestamp,
    std::optional<std::pair<float, float>>* mod_alpha_beta,
    std::optional<float>* ibus) {
  (void)output_timestamp;

  *mod_alpha_beta = {mod_alpha_, mod_beta_};
  *ibus = std::nullopt;
  return Motor::ERROR_NONE;
}

void DemoController::set_modulation(float alpha, float beta) {
  mod_alpha_ = alpha;
  mod_beta_ = beta;
}
