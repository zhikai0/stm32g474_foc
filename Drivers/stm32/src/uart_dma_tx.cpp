#include "uart_dma_tx.hpp"

#include "system.h"
#include "usart.h"

#include <string.h>

namespace {

/* 环形缓冲区容量：用于暂存待发送日志。
 * 这里选择 2KB，足够覆盖短时间的日志突发。 */
constexpr uint16_t kUartTxRingBufferSize = 2048;

/* ring buffer 本体：
 *   head 指向“下一个要写入的位置”
 *   tail 指向“下一段待发送数据的起点” */
uint8_t uart_tx_ring_buffer[kUartTxRingBufferSize];
volatile uint16_t uart_tx_head = 0;
volatile uint16_t uart_tx_tail = 0;

/* 当前 DMA 发送状态：
 *   uart_tx_dma_busy 表示 DMA 正在发送
 *   uart_tx_dma_len  记录这一次 DMA 实际发送了多少字节 */
volatile uint16_t uart_tx_dma_len = 0;
volatile bool uart_tx_dma_busy = false;

/* DMA staging buffer：
 * 为了把 ring buffer 中“当前连续的一段”整理成一块连续内存，
 * 这里使用单独 staging buffer 交给 DMA 发送。 */
uint8_t uart_tx_dma_staging[kUartTxRingBufferSize];

/* 计算 ring buffer 当前已占用字节数 */
inline uint16_t uart_tx_count() {
    return (uart_tx_head >= uart_tx_tail)
            ? (uint16_t)(uart_tx_head - uart_tx_tail)
            : (uint16_t)(kUartTxRingBufferSize - uart_tx_tail + uart_tx_head);
}

/* 计算剩余可写空间。
 * 这里保留 1 字节空位，用于区分“空”和“满”。 */
inline uint16_t uart_tx_free_space() {
    return (uint16_t)(kUartTxRingBufferSize - 1u - uart_tx_count());
}

/* 在“已进入临界区”的前提下，尝试启动一次 DMA 发送。
 * 规则：
 *   - DMA 正忙：直接返回
 *   - ring buffer 为空：直接返回
 *   - 否则取出 tail 开始的一段连续数据，复制到 staging，再启动 DMA */
void uart_dma_tx_kick_locked() {
    if (uart_tx_dma_busy || uart_tx_head == uart_tx_tail) {
        return;
    }

    uint16_t len = (uart_tx_head > uart_tx_tail)
            ? (uint16_t)(uart_tx_head - uart_tx_tail)
            : (uint16_t)(kUartTxRingBufferSize - uart_tx_tail);

    memcpy(uart_tx_dma_staging, &uart_tx_ring_buffer[uart_tx_tail], len);
    uart_tx_dma_len = len;
    uart_tx_dma_busy = true;

    if (HAL_UART_Transmit_DMA(&huart3, uart_tx_dma_staging, len) != HAL_OK) {
        uart_tx_dma_busy = false;
        uart_tx_dma_len = 0;
    }
}

}  // namespace

extern "C" void uart_dma_tx_init(void) {
    CRITICAL_SECTION() {
        uart_tx_head = 0;
        uart_tx_tail = 0;
        uart_tx_dma_len = 0;
        uart_tx_dma_busy = false;
    }
}

extern "C" bool uart_dma_tx_write(const uint8_t* data, uint16_t len) {
    if (!data || len == 0) {
        return true;
    }

    bool wrote = false;
    CRITICAL_SECTION() {
        if (uart_tx_free_space() < len) {
            wrote = false;
        } else {
            for (uint16_t i = 0; i < len; ++i) {
                uart_tx_ring_buffer[uart_tx_head] = data[i];
                uart_tx_head = (uint16_t)((uart_tx_head + 1) % kUartTxRingBufferSize);
            }

            uart_dma_tx_kick_locked();
            wrote = true;
        }
    }
    return wrote;
}

extern "C" void uart_dma_tx_on_complete_isr(void) {
    CRITICAL_SECTION() {
        if (!uart_tx_dma_busy) {
            return;
        }

        uart_tx_tail = (uint16_t)((uart_tx_tail + uart_tx_dma_len) % kUartTxRingBufferSize);
        uart_tx_dma_len = 0;
        uart_tx_dma_busy = false;

        /* 若 ring buffer 中还有积压数据，继续无缝发下一段 */
        uart_dma_tx_kick_locked();
    }
}
