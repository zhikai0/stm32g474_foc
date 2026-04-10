#include <array>
#include <math.h>
#include <tuple>

#include "board_support.hpp"  // HAL 外设头文件 + print/sleep/print_memory_usage/sys_init
#include "test_m.hpp"

static uint16_t adc2_scan_raw[2] = {0, 0}; // [0]=PA6/ADC2_IN3, [1]=PA7/ADC2_IN4

auto test_pair(auto x) {
  return x;
}

void spi_test_once() {
  uint16_t tx[3] = {0xA003, 0x0000, 0x0000}; // 连续读命令：0xA003，然后两个 dummy，返回 3 个 16bit word
  uint16_t rx[3] = {0, 0, 0};

  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx, (uint8_t*)rx, 3, 20);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  if (st != HAL_OK) {
    print("SPI error: %d", (int)st);
    return;
  }

  uint32_t angle21 = ((uint32_t)rx[1] << 5) | (rx[2] >> 11);
  float angle_deg = (float)angle21 * (360.0f / 2097152.0f); // 2^21 注意：rx[0] 常为状态/空帧，角度主要在 rx[1], rx[2]
  auto p = std::pair{42, 3.14f};
  auto [x, y] = p;
  auto [data, data2] = test_pair(p);
  auto rx_arr = std::array{rx[0], rx[1], rx[2]};
  auto rx_tuple = std::tuple{rx[0], rx[1], rx[2]};
  // print("[MT6835]rx_arr{} rx_tuple={} angle21={} angle={} deg x={} y={} data={}",
  //       rx_arr, rx_tuple, angle21, angle_deg, x, y, data);
    print("[MT6835]angle21={} angle={}°",
          angle21, angle_deg);
}

static bool adc2_scan_read_once(uint16_t* vbus_raw, uint16_t* temp_raw) {
  // 保险起见，先停掉上一次可能残留的规则组转换。
  (void)HAL_ADC_Stop(&hadc2);

  if (HAL_ADC_Start(&hadc2) != HAL_OK) {
    print("[adc] start failed");
    return false;
  }

  if (HAL_ADC_PollForConversion(&hadc2, 20) != HAL_OK) {
    print("[adc] poll rank1 failed");
    HAL_ADC_Stop(&hadc2);
    return false;
  }
  adc2_scan_raw[0] = (uint16_t)HAL_ADC_GetValue(&hadc2); // PA6 / ADC2_IN3

  if (HAL_ADC_PollForConversion(&hadc2, 20) != HAL_OK) {
    print("[adc] poll rank2 failed");
    HAL_ADC_Stop(&hadc2);
    return false;
  }
  adc2_scan_raw[1] = (uint16_t)HAL_ADC_GetValue(&hadc2); // PA7 / ADC2_IN4

  HAL_ADC_Stop(&hadc2);

  if (vbus_raw) {
    *vbus_raw = adc2_scan_raw[0];
  }
  if (temp_raw) {
    *temp_raw = adc2_scan_raw[1];
  }
  return true;
}

void adc2_pa7_thermistor_test_once() {
  uint16_t vbus_raw = 0;
  uint16_t pa7_raw = 0;
  if (!adc2_scan_read_once(&vbus_raw, &pa7_raw)) {
    return;
  }

  constexpr float vref = 3.3f;
  constexpr float pullup_r = 10000.0f; // 3.3V -> 10k -> PA7 -> NTC -> GND
  constexpr float ntc_r25 = 10000.0f;  // NTC 10k @ 25C
  constexpr float ntc_beta = 3380.0f;  // NCP18XH103F03RB
  constexpr float t25_k = 25.0f + 273.15f;

  const float pa6_voltage = (float)vbus_raw * (vref / 4095.0f);
  const float pa7_voltage = (float)pa7_raw * (vref / 4095.0f);

  if (pa7_voltage <= 0.001f || pa7_voltage >= (vref - 0.001f)) {
    print("[adc] pa6_raw=%u pa6_voltage=%.3fV pa7_raw=%u pa7_voltage=%.3fV out of range",
          (unsigned)vbus_raw,
          pa6_voltage,
          (unsigned)pa7_raw,
          pa7_voltage);
    return;
  }

  const float ntc_r = pullup_r * pa7_voltage / (vref - pa7_voltage);
  const float temp_k = 1.0f / ((1.0f / ntc_beta) * logf(ntc_r / ntc_r25) + (1.0f / t25_k));
  const float temp_c = temp_k - 273.15f;
  print("[adc] pa6_raw=%u pa6_voltage=%.3fV pa7_raw=%u pa7_voltage=%.3fV ntc_r=%.1fohm temp_c=%.2f",
        (unsigned)vbus_raw,
        pa6_voltage,
        (unsigned)pa7_raw,
        pa7_voltage,
        ntc_r,
        temp_c);
}

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
