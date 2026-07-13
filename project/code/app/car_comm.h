/**
 * car_comm.h — 车-上位机通信协议 (Schucker-Pilot)
 *
 * 接收帧: AA 55 | 0x20 | len(8) | ax(f32) | ay(f32) | XOR   (机体系加速度 cm/s²)
 * 发送帧: AA 55 | 0x21 | len(24) | x | y | theta | v | delta | ey | XOR
 */
#ifndef CAR_COMM_H
#define CAR_COMM_H
#include "common.h"
#include "car_state.h"

/* ── 上位机发来的数据 ── */
typedef struct {
    f32 ax, ay;       /* 机体系加速度 [cm/s²] (Schucker-Pilot 约定) */
    u32 seq;           /* 帧序号 */
} car_comm_rx_t;

void          car_comm_init(void);
void          car_comm_send(const CarState *car, f32 ey);
car_comm_rx_t car_comm_get(void);

#endif
