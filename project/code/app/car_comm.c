/**
 * car_comm.c — 状态帧打包发送 (补录分支: 仅车→上位机, 飞机接收已剥离)
 *
 * 发送 (车→上位机, task 层 20Hz):
 *   CMD 0x20 STATUS LEN=20   a_fwd, a_lat [m/s²], world_x, world_y [cm], theta [rad]
 */
#include "car_comm.h"
#include "hal_uart.h"
#include <string.h>

void car_comm_init(void) {
    hal_uart_init();
}

/* ── CMD 0x20 STATUS: a_fwd + a_lat + world_x + world_y + theta ── */
void car_comm_send(f32 a_fwd, f32 a_lat, f32 world_x, f32 world_y, f32 theta) {
    u8 f[25];  /* 4 + 20 + 1 */
    f[0] = 0xAA; f[1] = 0x55; f[2] = 0x20; f[3] = 20;
    memcpy(&f[4],  &a_fwd,    4);
    memcpy(&f[8],  &a_lat,    4);
    memcpy(&f[12], &world_x,  4);
    memcpy(&f[16], &world_y,  4);
    memcpy(&f[20], &theta,    4);
    u8 x = 0;
    for (u8 i = 0; i < 22; i++) x ^= f[2 + i];
    f[24] = x;
    hal_uart_send(f, 25);
}
