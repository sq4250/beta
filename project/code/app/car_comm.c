/**
 * car_comm.c — 帧解析 + 帧打包
 *
 * 接收 (Schucker-Pilot → 车):
 *   AA 55 | 0x20 | len(8) | ax(f32) | ay(f32) | XOR   共 13 字节
 *   ax/ay 为机体系加速度 [cm/s²]
 *
 * 发送 (车 → Schucker-Pilot):
 *   AA 55 | 0x21 | len(24) | x | y | theta | v | delta | ey | XOR
 *   6×f32 = 24 字节 payload, 帧长 29 字节
 */
#include "car_comm.h"
#include "hal_uart.h"
#include <string.h>

/* ── 接收帧常量 ── */
#define RX_HDR0  0xAA
#define RX_HDR1  0x55
#define RX_CMD   0x20
#define RX_LEN   8
#define RX_SIZE  13

/* ── 发送帧常量 ── */
#define TX_CMD   0x21
#define TX_LEN   24   /* 6×f32 */
#define TX_SIZE  29   /* 2(hdr) + 1(cmd) + 1(len) + 24(payload) + 1(xor) */

static volatile car_comm_rx_t s_rx;
static u8  s_rx_buf[RX_SIZE];
static u8  s_rx_len;

/* ── ISR 回调: 滑动窗口逐字节解析 ── */
static void car_comm_feed(u8 byte) {
    if (s_rx_len < RX_SIZE) {
        s_rx_buf[s_rx_len++] = byte;
    } else {
        memmove(s_rx_buf, s_rx_buf + 1, RX_SIZE - 1);
        s_rx_buf[RX_SIZE - 1] = byte;
    }
    if (s_rx_len < RX_SIZE) return;

    if (s_rx_buf[0] != RX_HDR0 || s_rx_buf[1] != RX_HDR1) return;
    if (s_rx_buf[2] != RX_CMD  || s_rx_buf[3] != RX_LEN)  return;

    /* XOR 校验: bytes [2..11] */
    u8 x = 0;
    for (u8 j = 0; j < 2 + RX_LEN; j++) x ^= s_rx_buf[2 + j];
    if (x != s_rx_buf[RX_SIZE - 1]) return;

    memcpy((void *)&s_rx.ax, &s_rx_buf[4], 4);
    memcpy((void *)&s_rx.ay, &s_rx_buf[8], 4);
    s_rx.seq++;
}

void car_comm_init(void) {
    memset((void *)&s_rx, 0, sizeof(s_rx));
    s_rx_len = 0;
    hal_uart_init();
    hal_uart_recv_cb_register(car_comm_feed);
}

/* ── 无锁一致性读 ── */
car_comm_rx_t car_comm_get(void) {
    car_comm_rx_t out;
    u32 s1, s2;
    do {
        s1  = s_rx.seq;
        out = s_rx;
        s2  = s_rx.seq;
    } while (s1 != s2);
    return out;
}

/* ── 打包发送: car state → 29 字节帧 → UART2 ── */
void car_comm_send(const CarState *car, f32 ey) {
    u8  frame[TX_SIZE];
    u8  xor_val;
    u8  i;

    frame[0] = RX_HDR0;
    frame[1] = RX_HDR1;
    frame[2] = TX_CMD;
    frame[3] = TX_LEN;

    f32 payload[6];
    payload[0] = car->x;
    payload[1] = car->y;
    payload[2] = car->theta;
    payload[3] = car->v;
    payload[4] = car->delta;
    payload[5] = ey;
    memcpy(&frame[4], payload, TX_LEN);

    /* XOR over cmd + len + payload */
    xor_val = 0;
    for (i = 0; i < 2 + TX_LEN; i++) xor_val ^= frame[2 + i];
    frame[TX_SIZE - 1] = xor_val;

    hal_uart_send(frame, TX_SIZE);
}
