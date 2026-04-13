#include "board_support.hpp"
#include "interface_test.hpp"

namespace {
bool adc_read_single_once(ADC_HandleTypeDef* hadc, uint16_t* raw) {
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
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

bool adc1_init_for_vrefint_once() {
    static bool initialized = false;
    if (initialized) {
        return true;
    }

    sleep(0.001f);
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        print("[adc] adc1 calibration failed");
        return false;
    }

    initialized = true;
    return true;
}

bool adc1_config_vrefint() {
    ADC_ChannelConfTypeDef sConfig = {};
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    (void)HAL_ADC_Stop(&hadc1);
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return false;
    }

    delay_us(20);
    return true;
}

bool adc1_read_vdda(uint16_t* vref_raw, uint32_t* vdda_mv) {
    if (!adc1_init_for_vrefint_once()) {
        return false;
    }
    if (!adc1_config_vrefint()) {
        print("[adc] adc1 cfg vrefint failed");
        return false;
    }

    constexpr int kSamples = 8;
    uint32_t sum = 0;
    for (int i = 0; i < kSamples; ++i) {
        uint16_t raw = 0;
        if (!adc_read_single_once(&hadc1, &raw)) {
            print("[adc] adc1 read vrefint failed");
            return false;
        }
        sum += raw;
    }

    const uint16_t avg_raw = (uint16_t)((sum + (kSamples / 2)) / kSamples);
    if (vref_raw) {
        *vref_raw = avg_raw;
    }
    if (vdda_mv) {
        *vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(avg_raw, hadc1.Init.Resolution);
    }
    return true;
}

bool adc2_read_next_conversion(uint16_t* raw) {
    if (HAL_ADC_PollForConversion(&hadc2, 20) != HAL_OK) {
        return false;
    }
    *raw = (uint16_t)HAL_ADC_GetValue(&hadc2);
    return true;
}

bool adc2_config_rank_pair(uint32_t rank1_channel, uint32_t rank2_channel) {
    ADC_ChannelConfTypeDef sConfig = {};
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    (void)HAL_ADC_Stop(&hadc2);

    sConfig.Channel = rank1_channel;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        return false;
    }

    sConfig.Channel = rank2_channel;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        return false;
    }

    return true;
}

bool adc2_read_rank_pair(uint16_t* rank1_raw, uint16_t* rank2_raw) {
    if (!rank1_raw || !rank2_raw) {
        return false;
    }

    (void)HAL_ADC_Stop(&hadc2);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
    if (HAL_ADC_Start(&hadc2) != HAL_OK) {
        return false;
    }

    constexpr int kDiscardPairs = 1;
    constexpr int kAveragePairs = 8;
    uint32_t rank1_sum = 0;
    uint32_t rank2_sum = 0;

    for (int i = 0; i < (kDiscardPairs + kAveragePairs); ++i) {
        uint16_t rank1 = 0;
        uint16_t rank2 = 0;
        if (!adc2_read_next_conversion(&rank1) || !adc2_read_next_conversion(&rank2)) {
            HAL_ADC_Stop(&hadc2);
            return false;
        }

        if (i >= kDiscardPairs) {
            rank1_sum += rank1;
            rank2_sum += rank2;
        }
    }

    HAL_ADC_Stop(&hadc2);

    *rank1_raw = (uint16_t)((rank1_sum + (kAveragePairs / 2)) / kAveragePairs);
    *rank2_raw = (uint16_t)((rank2_sum + (kAveragePairs / 2)) / kAveragePairs);
    return true;
}

bool adc2_read_stable_channel_with_vdda(uint32_t channel,
                                        uint16_t* raw,
                                        uint16_t* vref_raw,
                                        uint32_t* vdda_mv) {
    if (!raw) {
        return false;
    }
    if (!adc1_read_vdda(vref_raw, vdda_mv)) {
        return false;
    }

    uint16_t rank1_raw = 0;
    uint16_t rank2_raw = 0;
    if (!adc2_config_rank_pair(channel, channel)) {
        print("[adc] adc2 cfg stable channel failed");
        return false;
    }
    if (!adc2_read_rank_pair(&rank1_raw, &rank2_raw)) {
        print("[adc] adc2 stable read failed");
        return false;
    }

    *raw = (uint16_t)((rank1_raw + rank2_raw + 1U) / 2U);
    return true;
}

float adc_raw_to_volts(uint16_t raw, uint32_t vdda_mv) {
    return (static_cast<float>(raw) * static_cast<float>(vdda_mv)) / (4095.0f * 1000.0f);
}

}  // namespace

void adc2_pa7_thermistor_test_once(void) {
    static bool printed_channel_note = false;
    uint16_t pa7_raw = 0;
    uint16_t pa6_raw = 0;
    uint16_t vref_raw = 0;
    uint32_t vdda_mv = 0;

    if (!printed_channel_note) {
        print("[adc] 当前测试读取 ADC2_IN4(PA7) 和 ADC2_IN3(PA6)，未读取 ADC_CHANNEL_6");
        print("[adc] 若 PA6 悬空，ADC 读数只代表浮空节点与采样残留，不代表真实电压");
        printed_channel_note = true;
    }

    if (!adc2_read_stable_channel_with_vdda(ADC_CHANNEL_4, &pa7_raw, &vref_raw, &vdda_mv)) {
        return;
    }
    if (!adc2_read_stable_channel_with_vdda(ADC_CHANNEL_3, &pa6_raw, nullptr, nullptr)) {
        return;
    }
    (void)adc2_config_rank_pair(ADC_CHANNEL_4, ADC_CHANNEL_4);

    print("[adc] vrefint={},vdda={}V,ch3(PA6)={}/{},ch4(PA7)={}/{}",
        vref_raw,vdda_mv/1000.0f,
        pa6_raw,adc_raw_to_volts(pa6_raw, vdda_mv),
        pa7_raw,adc_raw_to_volts(pa7_raw, vdda_mv));
}
