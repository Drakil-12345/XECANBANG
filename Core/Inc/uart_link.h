#ifndef UART_LINK_H
#define UART_LINK_H

/*
 * USART2 link to the ESP32 (PA2 = TX, PA3 = RX, AF7), 8N1.
 * Register-level driver: it does NOT need HAL_UART_MODULE_ENABLED, CubeMX
 * changes or the HAL UART source files. RX and TX are interrupt driven with
 * ring buffers, so nothing here ever blocks the 250 Hz balance loop.
 */

#include <stdbool.h>
#include <stdint.h>

void UartLink_Init(uint32_t baud);

/* Returns next received byte, or -1 if the RX buffer is empty. */
int UartLink_ReadByte(void);

/* Queues len bytes for sending. Returns false (queues nothing) if no room. */
bool UartLink_Write(const uint8_t *data, uint16_t len);

/* Debug counters (Live Expressions). */
extern volatile uint32_t g_uart_rx_bytes;
extern volatile uint32_t g_uart_rx_dropped;

#endif /* UART_LINK_H */
