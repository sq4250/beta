/**
 * car_comm.h — 车-飞机通信协议
 *
 * 接收帧 (飞机→车):
 *   CMD 0x10 ACCEL    an,aw (LEN=8, f32×2, cm/s², rad/s)
 *
 * 发送帧 (车→飞机):
 *   CMD 0x20 POS      世界位置 world_x,world_y (LEN=8, f32×2, cm)
 *
 * VST 由车端自己积分，飞机不下发 VST 位置。
 */
#ifndef CAR_COMM_H
#define CAR_COMM_H
#include "common.h"
#include "car_state.h"

/* ── 飞机发来的指令 ── */
typedef struct {
    f32 a, omega;     /* CMD 0x10: an[cm/s²], omega[rad/s] */
    u32 seq;           /* 帧序号 */
} car_comm_rx_t;

void          car_comm_init(void);
void          car_comm_send(f32 world_x, f32 world_y);
car_comm_rx_t car_comm_get(void);
bool          car_comm_reset_pending(void);   /* 收到 CMD 0x30 重置帧 */

#endif
