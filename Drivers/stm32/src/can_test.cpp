#include "board_support.hpp"
#include "interface_test.hpp"

namespace {
constexpr bool kCanUseInternalLoopback = false;
constexpr uint32_t kCanTxStdId = 0x123;
constexpr uint32_t kCanNominalPrescaler = 17;
constexpr uint32_t kCanNominalSyncJumpWidth = 1;
constexpr uint32_t kCanNominalTimeSeg1 = 16;
constexpr uint32_t kCanNominalTimeSeg2 = 3;

uint32_t fdcan_dlc_to_len(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 0;
    }
}

bool can_reconfigure_core() {
    if (hfdcan2.State == HAL_FDCAN_STATE_BUSY) {
        if (HAL_FDCAN_Stop(&hfdcan2) != HAL_OK) {
            print("[can] hfdcan2 stop failed err=0x%08lx", (unsigned long)hfdcan2.ErrorCode);
            return false;
        }
    }

    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan2.Init.Mode = kCanUseInternalLoopback ? FDCAN_MODE_INTERNAL_LOOPBACK : FDCAN_MODE_NORMAL;
    hfdcan2.Init.AutoRetransmission = DISABLE;
    hfdcan2.Init.NominalPrescaler = kCanNominalPrescaler;
    hfdcan2.Init.NominalSyncJumpWidth = kCanNominalSyncJumpWidth;
    hfdcan2.Init.NominalTimeSeg1 = kCanNominalTimeSeg1;
    hfdcan2.Init.NominalTimeSeg2 = kCanNominalTimeSeg2;
    hfdcan2.Init.StdFiltersNbr = 1;
    hfdcan2.Init.ExtFiltersNbr = 0;

    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
        print("[can] hfdcan2 re-init failed err=0x%08lx", (unsigned long)hfdcan2.ErrorCode);
        return false;
    }
    return true;
}

bool can_configure_filters() {
    FDCAN_FilterTypeDef rx_filter = {};
    rx_filter.IdType = FDCAN_STANDARD_ID;
    rx_filter.FilterIndex = 0;
    rx_filter.FilterType = FDCAN_FILTER_MASK;
    rx_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    rx_filter.FilterID1 = 0x000;
    rx_filter.FilterID2 = 0x000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &rx_filter) != HAL_OK) {
        print("[can] hfdcan2 filter config failed err=0x%08lx", (unsigned long)hfdcan2.ErrorCode);
        return false;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan2,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK) {
        print("[can] hfdcan2 global filter config failed err=0x%08lx", (unsigned long)hfdcan2.ErrorCode);
        return false;
    }

    return true;
}

void can_dump_rx_fifo0(void) {
    uint32_t rx_fifo_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
    while (rx_fifo_fill > 0U) {
        FDCAN_RxHeaderTypeDef rx_hdr = {};
        uint8_t rx_data[64] = {0};
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx_hdr, rx_data) != HAL_OK) {
            print("[can] rx read failed err=0x%08lx fill=%lu",
                  (unsigned long)hfdcan2.ErrorCode,
                  (unsigned long)rx_fifo_fill);
            return;
        }

        const uint32_t rx_len = fdcan_dlc_to_len(rx_hdr.DataLength);
        print("[can] rx id=0x%03lx len=%lu data=%02X %02X %02X %02X %02X %02X %02X %02X",
              (unsigned long)rx_hdr.Identifier,
              (unsigned long)rx_len,
              rx_data[0],
              rx_data[1],
              rx_data[2],
              rx_data[3],
              rx_data[4],
              rx_data[5],
              rx_data[6],
              rx_data[7]);

        rx_fifo_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
    }
}
}  // namespace

bool can_start(void) {
    if (!can_reconfigure_core()) {
        return false;
    }
    if (!can_configure_filters()) {
        return false;
    }
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        print("[can] hfdcan2 start failed err=0x%08lx", (unsigned long)hfdcan2.ErrorCode);
        return false;
    }

    print("[can] hfdcan2 started mode=%s nominal=%lu/%lu/%lu std-filter=mask-all->fifo0 tx-id=0x%03lx",
          kCanUseInternalLoopback ? "loopback" : "normal",
          (unsigned long)kCanNominalPrescaler,
          (unsigned long)kCanNominalTimeSeg1,
          (unsigned long)kCanNominalTimeSeg2,
          (unsigned long)kCanTxStdId);
    return true;
}

void can_test(void) {
    static uint32_t can_tx_count = 0;

    FDCAN_TxHeaderTypeDef tx_hdr = {};
    tx_hdr.Identifier = kCanTxStdId;
    tx_hdr.IdType = FDCAN_STANDARD_ID;
    tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
    tx_hdr.DataLength = FDCAN_DLC_BYTES_8;
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
    tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
    tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_hdr.MessageMarker = 0;

    can_tx_count++;
    uint8_t tx_data[8];
    tx_data[0] = (uint8_t)(can_tx_count >> 24);
    tx_data[1] = (uint8_t)(can_tx_count >> 16);
    tx_data[2] = (uint8_t)(can_tx_count >> 8);
    tx_data[3] = (uint8_t)(can_tx_count >> 0);
    tx_data[4] = (uint8_t)(HAL_GetTick() >> 24);
    tx_data[5] = (uint8_t)(HAL_GetTick() >> 16);
    tx_data[6] = (uint8_t)(HAL_GetTick() >> 8);
    tx_data[7] = (uint8_t)(HAL_GetTick() >> 0);

    const uint32_t tx_fifo_free_before = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
    const HAL_StatusTypeDef tx_ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_hdr, tx_data);
    const uint32_t tx_fifo_free_after = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);

    FDCAN_ProtocolStatusTypeDef protocol_status = {};
    FDCAN_ErrorCountersTypeDef error_counters = {};
    HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol_status);
    HAL_FDCAN_GetErrorCounters(&hfdcan2, &error_counters);

    if (tx_ret == HAL_OK) {
        print("[can] tx queued id=0x%03lx cnt=%lu fifo=%lu->%lu lec=%lu act=%lu ep=%lu ew=%lu bo=%lu tec=%lu rec=%lu",
              (unsigned long)tx_hdr.Identifier,
              (unsigned long)can_tx_count,
              (unsigned long)tx_fifo_free_before,
              (unsigned long)tx_fifo_free_after,
              (unsigned long)protocol_status.LastErrorCode,
              (unsigned long)protocol_status.Activity,
              (unsigned long)protocol_status.ErrorPassive,
              (unsigned long)protocol_status.Warning,
              (unsigned long)protocol_status.BusOff,
              (unsigned long)error_counters.TxErrorCnt,
              (unsigned long)error_counters.RxErrorCnt);
    } else {
        print("[can] tx queue failed ret=%lu err=0x%08lx fifo=%lu->%lu lec=%lu act=%lu ep=%lu ew=%lu bo=%lu tec=%lu rec=%lu",
              (unsigned long)tx_ret,
              (unsigned long)hfdcan2.ErrorCode,
              (unsigned long)tx_fifo_free_before,
              (unsigned long)tx_fifo_free_after,
              (unsigned long)protocol_status.LastErrorCode,
              (unsigned long)protocol_status.Activity,
              (unsigned long)protocol_status.ErrorPassive,
              (unsigned long)protocol_status.Warning,
              (unsigned long)protocol_status.BusOff,
              (unsigned long)error_counters.TxErrorCnt,
              (unsigned long)error_counters.RxErrorCnt);
    }

    if (kCanUseInternalLoopback) {
        const uint32_t wait_start_us = micros();
        while ((uint32_t)(micros() - wait_start_us) < 2000U) {
            if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0U) {
                break;
            }
        }
    }

    can_dump_rx_fifo0();
}
