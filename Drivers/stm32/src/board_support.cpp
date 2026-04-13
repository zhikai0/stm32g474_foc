#include "board_support.hpp"  // HAL 外设头文件（main.h/usart.h/gpio.h 等已在此 include）
#include "uart_dma_tx.hpp"
#include "usbd_cdc_if.h"          // CDC_Transmit_FS(), USBD_OK

#include <stdint.h>
#include <stdio.h>

namespace {
inline uintptr_t get_sp(void) {
    register uintptr_t sp __asm("sp");
    return sp;
}

inline uintptr_t flash_size_bytes() {
#if defined(FLASHSIZE_BASE)
    return (uintptr_t)(*((uint16_t*)FLASHSIZE_BASE)) * 1024u;
#else
    return 0u;
#endif
}

inline uintptr_t flash_base_addr() {
#if defined(FLASH_BASE)
    return (uintptr_t)FLASH_BASE;
#else
    return 0x08000000u;
#endif
}

inline uintptr_t ram_base_addr() {
#if defined(SRAM_BASE)
    return (uintptr_t)SRAM_BASE;
#else
    return (uintptr_t)&_sdata;
#endif
}
}  // namespace

extern "C" {
extern uint32_t _sidata;  // LMA(.data) in FLASH
extern uint32_t _sdata;   // VMA(.data) start in RAM
extern uint32_t _edata;   // VMA(.data) end in RAM
extern uint32_t _ebss;    // VMA(.bss) end in RAM
extern uint8_t _estack;   // top of RAM from linker
}

// 返回自系统启动以来经过的微秒数。
// TIM2 已在 CubeMX 中配置为 1MHz 基准计数器，因此 CNT 每加 1 就代表 1us。
extern "C" uint32_t micros(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}

// us 级忙等延时：适合极短延时，不适合大段阻塞等待。
extern "C" void delay_us(uint32_t us) {
    const uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) {
        __NOP();
    }
}

// printf 重定向：将 newlib 的 _write 系统调用转发到 USART3 DMA ring buffer
extern "C" int _write(int file, char* ptr, int len) {
    (void)file;

    if (!ptr || len <= 0) {
        return 0;
    }
    return uart_dma_tx_write(reinterpret_cast<const uint8_t*>(ptr), (uint16_t)len) ? len : 0;
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart3) {
        uart_dma_tx_on_complete_isr();
    }
}
// ============================================================
// 板级初始化：时钟配置、外设初始化、错误处理
// 从 main.cpp 移入，保持 main.cpp 只含业务逻辑
// ============================================================
extern "C" void board_init(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_FDCAN2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USB_Device_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_SPI1_Init();
    MX_USART3_UART_Init();
    sleep(0.1); // 等待 ADC 稳定
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    uart_dma_tx_init();
    HAL_TIM_Base_Start(&htim2); // micros() 依赖 TIM2 作为时间基准，us级别时间戳
    // MX_CORDIC_Init(); // 硬件三角函数（待实现）


}

extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}
extern "C" void Error_Handler(void)
{
    __disable_irq(); // 关闭所有中断
    while (1) {}
}

#ifdef USE_FULL_ASSERT
extern "C" void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert failed: %s line %lu\r\n", (char*)file, (unsigned long)line);
}
#endif

// 打印 FLASH/RAM 使用情况
extern "C" void print_memory_usage(void) {
    const uintptr_t kFlashBase = flash_base_addr();
    const uintptr_t kFlashSize = flash_size_bytes();

    const uintptr_t flash_used_end = (uintptr_t)&_sidata + ((uintptr_t)&_edata - (uintptr_t)&_sdata);
    const uintptr_t flash_used = (flash_used_end > kFlashBase) ? (flash_used_end - kFlashBase) : 0u;
    const uintptr_t flash_free = (kFlashSize > flash_used) ? (kFlashSize - flash_used) : 0u;

    const uintptr_t kRamBase = ram_base_addr();
    const uintptr_t kRamTop = (uintptr_t)&_estack;
    const uintptr_t kRamSize = (kRamTop > kRamBase) ? (kRamTop - kRamBase) : 0u;

    const uintptr_t ram_static_used = ((uintptr_t)&_ebss > kRamBase) ? ((uintptr_t)&_ebss - kRamBase) : 0u;
    const uintptr_t sp = get_sp();
    const uintptr_t ram_stack_used = (kRamTop > sp) ? (kRamTop - sp) : 0u;
    const uintptr_t ram_gap = (sp > (uintptr_t)&_ebss) ? (sp - (uintptr_t)&_ebss) : 0u;

    printf("[MEM] FLASH used=%luKB free=%luKB (%lu/%lu B)\r\n",
           (unsigned long)(flash_used / 1024u),
           (unsigned long)(flash_free / 1024u),
           (unsigned long)flash_used,
           (unsigned long)kFlashSize);
    printf("[MEM] RAM static=%luKB stack=%luKB gap=%luKB (%lu B total)\r\n",
           (unsigned long)(ram_static_used / 1024u),
           (unsigned long)(ram_stack_used / 1024u),
           (unsigned long)(ram_gap / 1024u),
           (unsigned long)kRamSize);
}
