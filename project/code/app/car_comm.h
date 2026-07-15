/**
 * car_comm.h — 车-飞机通信协议
 *
 * 接收帧 (飞机→车):
 *   CMD 0x10 ACCEL    an,aw,vst_x,vst_y (LEN=16, f32×4, cm/s², cm)
 *
 * 发送帧 (车→飞机):
 *   CMD 0x20 POS      世界位置 world_x,world_y (LEN=8, f32×2, cm)
 */
#ifndef CAR_COMM_H
#define CAR_COMM_H
#include "common.h"
#include "car_state.h"

/* ── 飞机发来的指令 + VST 位置 ── */
typedef struct {
    f32 a, omega;     /* CMD 0x10: an[cm/s²], omega[rad/s] */
    f32 vst_x, vst_y; /* 飞机 VST 位置 cm, 车端用于同步 */
    u32 seq;           /* 帧序号 */
} car_comm_rx_t;

void          car_comm_init(void);
void          car_comm_send(f32 world_x, f32 world_y);
car_comm_rx_t car_comm_get(void);

#endif
