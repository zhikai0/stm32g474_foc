#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * uart_dma_tx : USART3 非阻塞发送模块
 *
 * 目标：
 *   1. 让 printf/_write 不再阻塞等待 UART 逐字节发送完成
 *   2. 使用“环形缓冲区 + DMA”把发送搬到后台执行
 *   3. 接口尽量保持简单，方便 board_support.cpp 直接接入
 *
 * 设计说明：
 *   - 写入路径：用户/主线程调用 uart_dma_tx_write()
 *   - 发送路径：若 DMA 空闲，则立即启动一次 DMA 发送
 *   - 完成路径：DMA 发送完成后，在 HAL_UART_TxCpltCallback() 中调用
 *               uart_dma_tx_on_complete_isr()，继续发送下一段
 *
 * 行为约定：
 *   - 本模块为“非阻塞日志输出”设计
 *   - 若环形缓冲区空间不足，写入失败并返回 false，不等待、不阻塞
 *   - 适合调试日志；若后续做高速数据流，建议进一步改为二进制协议
 * ========================================================================== */

void uart_dma_tx_init(void);
bool uart_dma_tx_write(const uint8_t* data, uint16_t len);
void uart_dma_tx_on_complete_isr(void);

#ifdef __cplusplus
}
#endif
