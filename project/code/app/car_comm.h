#pragma once
#include "common.h"

/* 飞机→车:
 *   CMD 0x10 ACT      a_n, a_w (LEN=8,  f32×2, m/s²)
 *   CMD 0x30 WP       n 路点   (LEN=n*8, f32×2n, cm)  n≤6
 *   CMD 0x31 START    启动
 *   CMD 0x32 STOP     停止+复位
 * 车→飞机:
 *   CMD 0x20 STATUS   a_fwd, a_lat, x, y, theta (LEN=20) */

#define REMOTE_WP_COUNT 6

typedef struct { f32 x, y; } remote_waypoint_t;

typedef struct {
    f32  a_n, a_w;  u32 a_seq;
    remote_waypoint_t wp[REMOTE_WP_COUNT]; u32 wp_seq; u8 n_wp;
    bool start;     u32 start_seq;
    bool stop;      u32 stop_seq;
} car_comm_rx_t;

void          car_comm_init(void);
void          car_comm_send(f32 a_fwd, f32 a_lat, f32 x, f32 y, f32 theta);
car_comm_rx_t car_comm_get(void);
