/**
 * hal_uart.h — UART 通信硬件抽象层
 *
 * UART2 (TX_P07_1, RX_P07_0) 用于与上位机(Schucker-Pilot)通信
 * 回调机制: ISR 逐字节分发到已注册的 parser
 */
#ifndef HAL_UART_H
#define HAL_UART_H
#include "common.h"

#define UART_CAR_CH  0

typedef void (*uart_rx_cb_t)(u8 byte);

void hal_uart_init(void);
void hal_uart_send(const u8 *data, u16 len);
void hal_uart_recv_cb_register(uart_rx_cb_t cb);
void hal_uart_rx_dispatch(void);

#endif
