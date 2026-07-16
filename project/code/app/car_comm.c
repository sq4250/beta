/**
 * car_comm.c — 帧解析 + 帧打包
 *
 * 接收 (飞机→车, UART RX ISR):
 *   CMD 0x10 ACT → 解析动作指令 a, omega
 *
 * 发送 (车→飞机, task 层调用):
 *   CMD 0x20 POS → 世界位置 world_x, world_y (cm)
 *
 * 帧格式: AA 55 | CMD | LEN | PAYLOAD | XOR
 *   ACT: LEN=8   13 字节
 *   POS: LEN=8   13 字节
 */
#include "car_comm.h"
#include "hal_uart.h"
#include <string.h>

/* ── 帧常量 ── */
#define FRM_HDR0      0xAA
#define FRM_HDR1      0x55
#define FRM_CMD_ACT   0x10
#define FRM_CMD_POS   0x20
#define FRM_LEN       8
#define FRM_SIZE      13   /* HDR0+HDR1+CMD+LEN+8B_PAYLOAD+XOR */
#define ACT_LEN       8    /* a+omega = 2×f32 */
#define ACT_SIZE      13   /* 2+1+1+8+1 */
#define BUF_MAX       13

static volatile car_comm_rx_t s_rx;
static u8  s_rx_buf[BUF_MAX];
static u8  s_rx_len;

/* ── ISR 回调: 逐字节解析 ── */
static void car_comm_feed(u8 byte) {
    if (s_rx_len < FRM_SIZE) {
        s_rx_buf[s_rx_len++] = byte;
    } else {
        memmove(s_rx_buf, s_rx_buf + 1, s_rx_len - 1);
        s_rx_buf[s_rx_len - 1] = byte;
    }
    if (s_rx_len < FRM_SIZE) return;

    if (s_rx_buf[0] != FRM_HDR0 || s_rx_buf[1] != FRM_HDR1) return;

    /* ── CMD 0x10: ACT (飞机→车) ── */
    if (s_rx_buf[2] != FRM_CMD_ACT || s_rx_buf[3] != ACT_LEN) return;

    u8 x = 0;
    for (u8 j = 0; j < 2 + ACT_LEN; j++) x ^= s_rx_buf[2 + j];
    if (x != s_rx_buf[ACT_SIZE - 1]) return;

    memcpy((void *)&s_rx.a,     &s_rx_buf[4], 4);
    memcpy((void *)&s_rx.omega, &s_rx_buf[8], 4);
    s_rx.seq++;
    s_rx_len = 0;
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

/* ── 打包发送: 世界位置 cm → CMD 0x20 帧 → UART ── */
void car_comm_send(f32 world_x, f32 world_y) {
    u8 frame[FRM_SIZE];
    u8 i, xor_val;

    frame[0] = FRM_HDR0;
    frame[1] = FRM_HDR1;
    frame[2] = FRM_CMD_POS;
    frame[3] = FRM_LEN;

    memcpy(&frame[4], &world_x, sizeof(f32));
    memcpy(&frame[8], &world_y, sizeof(f32));

    xor_val = 0;
    for (i = 0; i < 2 + FRM_LEN; i++) xor_val ^= frame[2 + i];
    frame[FRM_SIZE - 1] = xor_val;

    hal_uart_send(frame, FRM_SIZE);
}
