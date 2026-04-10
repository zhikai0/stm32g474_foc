#include <array>
#include <math.h>
#include <tuple>

#include "board_support.hpp"  // HAL 外设头文件 + print/sleep/print_memory_usage/sys_init
#include "test_m.hpp"

static uint16_t adc2_scan_raw[2] = {0, 0}; // [0]=unused, [1]=PA7/ADC2_IN4

static bool adc_poll_single_rank(ADC_HandleTypeDef* hadc, uint16_t* raw) {
  if (HAL_ADC_Start(hadc) != HAL_OK) {
    return false;
  }
  if (HAL_ADC_PollForConversion(hadc, 20) != HAL_OK) {
    HAL_ADC_Stop(hadc);
    return false;
  }
  *raw = (uint16_t)HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);
  return true;
}

static bool adc2_read_channel_stable(uint32_t channel, uint16_t* raw_first, uint16_t* raw_second) {
  ADC_ChannelConfTypeDef sConfig = {};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  (void)HAL_ADC_Stop(&hadc2);
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    print("[adc] cfg ch=%lu failed", (unsigned long)channel);
    return false;
  }

  uint16_t first = 0;
  uint16_t second = 0;
  if (!adc_poll_single_rank(&hadc2, &first)) {
    print("[adc] read1 ch=%lu failed", (unsigned long)channel);
    return false;
  }
  if (!adc_poll_single_rank(&hadc2, &second)) {
    print("[adc] read2 ch=%lu failed", (unsigned long)channel);
    return false;
  }

  if (raw_first) {
    *raw_first = first;
  }
  if (raw_second) {
    *raw_second = second;
  }
  return true;
}

static void adc2_dump_regs_once(void) {
  static bool dumped = false;
  if (dumped) {
    return;
  }
  dumped = true;

  print("[adc] regs syscfg_cfgr1=0x%08lx gpioa_moder=0x%08lx adc2_cfgr=0x%08lx",
        (unsigned long)SYSCFG->CFGR1,
        (unsigned long)GPIOA->MODER,
        (unsigned long)ADC2->CFGR);
  print("[adc] regs adc2_cfgr2=0x%08lx adc2_sqr1=0x%08lx adc2_smpr1=0x%08lx",
        (unsigned long)ADC2->CFGR2,
        (unsigned long)ADC2->SQR1,
        (unsigned long)ADC2->SMPR1);
  print("[adc] regs difsel=0x%08lx calfact=0x%08lx ccr=0x%08lx",
        (unsigned long)ADC2->DIFSEL,
        (unsigned long)ADC2->CALFACT,
        (unsigned long)ADC12_COMMON->CCR);
  print("[adc] regs ofr1=0x%08lx ofr2=0x%08lx ofr3=0x%08lx ofr4=0x%08lx",
        (unsigned long)ADC2->OFR1,
        (unsigned long)ADC2->OFR2,
        (unsigned long)ADC2->OFR3,
        (unsigned long)ADC2->OFR4);
}

static bool adc1_read_vrefint(uint16_t* vref_raw) {
  ADC_ChannelConfTypeDef sConfig = {};
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  (void)HAL_ADC_Stop(&hadc1);
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    print("[adc] cfg vrefint failed");
    return false;
  }
  uint16_t throwaway = 0;
  if (!adc_poll_single_rank(&hadc1, &throwaway)) {
    print("[adc] prime vrefint failed");
    return false;
  }
  if (!adc_poll_single_rank(&hadc1, vref_raw)) {
    print("[adc] read vrefint failed");
    return false;
  }
  return true;
}

static bool adc2_read_pa7_once(uint16_t* pa7_raw) {
  uint16_t pa7_first = 0;
  uint16_t pa7_second = 0;

  if (!adc2_read_channel_stable(ADC_CHANNEL_4, &pa7_first, &pa7_second)) {
    return false;
  }

  adc2_scan_raw[1] = pa7_second;

  if (pa7_raw) {
    *pa7_raw = pa7_second;
  }

  print("[adc] settle pa7=%u->%u",
        (unsigned)pa7_first,
        (unsigned)pa7_second);
  return true;
}

template <typename T>
T test_pair(T x) {
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

void adc2_pa7_thermistor_test_once() {
  adc2_dump_regs_once();

  uint16_t vref_raw = 0;
  uint16_t pa7_raw = 0;
  if (!adc1_read_vrefint(&vref_raw)) {
    return;
  }
  if (!adc2_read_pa7_once(&pa7_raw)) {
    return;
  }

  constexpr float pullup_r = 10000.0f; // 3.3V -> 10k -> PA7 -> NTC -> GND
  constexpr float ntc_r25 = 10000.0f;  // NTC 10k @ 25C
  constexpr float ntc_beta = 3380.0f;  // NCP18XH103F03RB
  constexpr float t25_k = 25.0f + 273.15f;
  const uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_12B);
  const float vdda = (float)vdda_mv / 1000.0f;

  const float pa7_voltage = (float)pa7_raw * (vdda / 4095.0f);

  if (pa7_voltage <= 0.001f || pa7_voltage >= (vdda - 0.001f)) {
    print("[adc] vref=%u vdda=%lumV pa7=%u/%.3fV out",
          (unsigned)vref_raw,
          (unsigned long)vdda_mv,
          (unsigned)pa7_raw,
          pa7_voltage);
    return;
  }

  const float ntc_r = pullup_r * pa7_voltage / (vdda - pa7_voltage);
  const float temp_k = 1.0f / ((1.0f / ntc_beta) * logf(ntc_r / ntc_r25) + (1.0f / t25_k));
  const float temp_c = temp_k - 273.15f;
  print("[adc] vref=%u vdda=%lumV pa7=%u/%.3fV ntc=%.1fohm t=%.2fC",
        (unsigned)vref_raw,
        (unsigned long)vdda_mv,
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
