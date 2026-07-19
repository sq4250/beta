/**
 * car_comm.c — 变长帧解析 + 帧打包
 *
 * 接收 (飞机→车, UART RX ISR):
 *   CMD 0x10 ACT    LEN=8   a_n, a_w [m/s²]
 *   CMD 0x30 WP     LEN=24  3×(x,y) [cm]
 *   CMD 0x31 START  LEN=0   启动信号
 *
 * 发送 (车→飞机, task 层):
 *   CMD 0x20 STATUS LEN=16  a_n, a_w [m/s²], world_x, world_y [cm] (NWU)
 */
#include "car_comm.h"
#include "hal_uart.h"
#include <string.h>

#define FRM_HDR0  0xAA
#define FRM_HDR1  0x55
#define BUF_MAX   32   /* 最大帧 4+24+1=29 */

static volatile car_comm_rx_t s_rx;
static u8  s_buf[BUF_MAX];
static u8  s_len;

/* ── 从 buffer 偏移处读 n 个 f32 ── */
static void rd_f32(volatile f32 *dst, const u8 *src, u8 n) {
    for (u8 i = 0; i < n; i++)
        dst[i] = ((const f32 *)(const void *)src)[i];
}

/* ── XOR [CMD, LEN, payload...] ── */
static u8 xsum(const u8 *buf, u8 dlen) {
    u8 x = 0;
    for (u8 j = 0; j < 2 + dlen; j++) x ^= buf[2 + j];
    return x;
}

/* ── ISR 回调: 逐字节送入, 循环解析直到数据不足 ── */
static void car_comm_feed(u8 byte) {
    /* 写入缓冲区 */
    if (s_len < BUF_MAX) {
        s_buf[s_len++] = byte;
    } else {
        memmove(s_buf, s_buf + 1, BUF_MAX - 1);
        s_buf[BUF_MAX - 1] = byte;
    }

    /* 循环解析: 处理缓冲区内所有完整帧 */
    for (;;) {
        if (s_len < 4) break;

        /* 滑动直到找到帧头 */
        if (s_buf[0] != FRM_HDR0 || s_buf[1] != FRM_HDR1) {
            memmove(s_buf, s_buf + 1, --s_len);
            continue;
        }

        u8 cmd  = s_buf[2];
        u8 dlen = s_buf[3];
        u8 total = 4 + dlen + 1;
        if (total > BUF_MAX) { memmove(s_buf, s_buf + 1, --s_len); continue; }
        if (s_len < total) break;  /* 数据不足, 等下一字节 */

        if (xsum(s_buf, dlen) != s_buf[total - 1]) {
            memmove(s_buf, s_buf + 1, --s_len);  /* XOR 错, 滑 1 字节重试 */
            continue;
        }

        /* ── 分发 ── */
        switch (cmd) {
        case 0x10: if (dlen == 8)  { rd_f32(&s_rx.a_n, &s_buf[4], 2); s_rx.a_seq++; }     break;
        case 0x30: if (dlen == 24) { rd_f32(&s_rx.wp[0].x, &s_buf[4], 6); s_rx.wp_seq++; } break;
        case 0x31: if (dlen == 0)  { s_rx.start = true;                      s_rx.start_seq++; } break;
        case 0x32: if (dlen == 0)  { s_rx.stop  = true;                      s_rx.stop_seq++;  } break;
        }

        /* 消费已解析帧, 剩余字节前移, 继续循环 */
        if (total >= s_len) { s_len = 0; break; }
        u8 rem = s_len - total;
        memmove(s_buf, s_buf + total, rem);
        s_len = rem;
    }
}

void car_comm_init(void) {
    memset((void *)&s_rx, 0, sizeof(s_rx));
    s_len = 0;
    hal_uart_init();
    hal_uart_recv_cb_register(car_comm_feed);
}

/* ── 原子快照 (关中断杜绝 ISR 半写) ── */
car_comm_rx_t car_comm_get(void) {
    car_comm_rx_t out;
    u32 st;
    __asm volatile ("mrs %0, primask" : "=r"(st));
    __asm volatile ("cpsid i");
    out = s_rx;
    __asm volatile ("msr primask, %0" :: "r"(st));
    return out;
}

/* ── CMD 0x20 STATUS: a_n + a_w + world_x + world_y (NWU) ── */
void car_comm_send(f32 a_n, f32 a_w, f32 world_x, f32 world_y) {
    u8 f[21];
    f[0] = 0xAA; f[1] = 0x55; f[2] = 0x20; f[3] = 16;
    memcpy(&f[4],  &a_n,     4);
    memcpy(&f[8],  &a_w,     4);
    memcpy(&f[12], &world_x,  4);
    memcpy(&f[16], &world_y,  4);
    u8 x = 0;
    for (u8 i = 0; i < 18; i++) x ^= f[2 + i];
    f[20] = x;
    hal_uart_send(f, 21);
}
