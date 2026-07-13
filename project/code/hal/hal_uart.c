/**
 * hal_uart.c — UART2 驱动实现 (TX_P07_1, RX_P07_0)
 *
 * 基于逐飞 zf_driver_uart, 使用中断接收 + 回调分发
 */
#include "hal_uart.h"
#include "zf_driver_uart.h"

#define UART_RX_CB_MAX  2

static uart_rx_cb_t s_rx_cb[UART_RX_CB_MAX];
static u8           s_rx_cb_cnt;

void hal_uart_init(void) {
    s_rx_cb_cnt = 0;
    uart_init(UART_2, 115200, UART2_TX_P07_1, UART2_RX_P07_0);
    uart_rx_interrupt(UART_2, 1);
}

void hal_uart_send(const u8 *data, u16 len) {
    uart_write_buffer(UART_2, data, len);
}

void hal_uart_recv_cb_register(uart_rx_cb_t cb) {
    if (s_rx_cb_cnt < UART_RX_CB_MAX)
        s_rx_cb[s_rx_cb_cnt++] = cb;
}

/* ── UART2 接收分发: 从 zf_driver 缓冲区逐字节读出, 分发给所有注册 parser ── */
void hal_uart_rx_dispatch(void) {
    u8 byte;
    while (uart_query_byte(UART_2, &byte)) {
        for (u8 i = 0; i < s_rx_cb_cnt; i++)
            s_rx_cb[i](byte);
    }
}
