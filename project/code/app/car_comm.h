/**
 * car_comm.h — 车-飞机通信协议 (变长帧)
 *
 * 接收帧 (飞机→车):
 *   CMD 0x10 ACT      a_n, a_w (LEN=8,  f32×2, m/s²)
 *   CMD 0x30 WP       3 目标点    (LEN=24, f32×6, cm)
 *   CMD 0x31 START    启动信号    (LEN=0)
 *   CMD 0x32 STOP     停止+复位   (LEN=0)
 *
 * 发送帧 (车→飞机):
 *   CMD 0x20 STATUS   a_n, a_w, world_x, world_y (LEN=16, NWU)
 */
#ifndef CAR_COMM_H
#define CAR_COMM_H
#include "common.h"
#include "car_state.h"

#define REMOTE_WP_COUNT 3

/* 单个目标点 (世界 NWU, cm) */
typedef struct {
    f32 x, y;
} RemoteWaypoint;

/* 飞机→车 汇总 (每种数据独立 seq) */
typedef struct {
    f32  a_n, a_w;                          /* CMD 0x10: 世界加速度 [m/s²] */
    u32  a_seq;

    RemoteWaypoint wp[REMOTE_WP_COUNT];     /* CMD 0x30: 目标点 */
    u32  wp_seq;

    bool start;                              /* CMD 0x31: 启动 */
    u32  start_seq;

    bool stop;                               /* CMD 0x32: 停止+复位 */
    u32  stop_seq;
} car_comm_rx_t;

void          car_comm_init(void);
void          car_comm_send(f32 a_des, f32 omega_des, f32 world_x, f32 world_y);
car_comm_rx_t car_comm_get(void);

#endif
