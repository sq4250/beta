/**
 * car_comm.h — 车-上位机通信协议 (Schucker-Pilot)
 *
 * 接收帧: AA 55 | 0x10 | len(8) | vn(f32) | vw(f32) | XOR   (世界系速度 cm/s, NWU)
 * 发送帧: AA 55 | 0x20 | len(8) | world_x(f32) | world_y(f32) | XOR   (世界位置 cm, NWU)
 */
#ifndef CAR_COMM_H
#define CAR_COMM_H
#include "common.h"
#include "car_state.h"

/* ── 上位机发来的数据 ── */
typedef struct {
    f32 vn, vw;       /* 世界系速度 [cm/s] (NWU: vn=北, vw=西) */
    u32 seq;           /* 帧序号 */
} car_comm_rx_t;

void          car_comm_init(void);
void          car_comm_send(f32 world_x, f32 world_y);
car_comm_rx_t car_comm_get(void);

#endif
