#include "board_support.hpp"
#include "interface_test.hpp"

void mt6835_test(void) {
    uint16_t tx[3] = {0xA003, 0x0000, 0x0000};
    uint16_t rx[3] = {0, 0, 0};

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx, (uint8_t*)rx, 3, 20);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    if (st != HAL_OK) {
        print("SPI error: %d", (int)st);
        return;
    }

    uint32_t angle21 = ((uint32_t)rx[1] << 5) | (rx[2] >> 11);
    float angle_deg = (float)angle21 * (360.0f / 2097152.0f);
    print("[MT6835]angle21={} angle={}°", angle21, angle_deg);
}
