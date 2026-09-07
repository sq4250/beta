#pragma once
#include "common.h"

/* 车→上位机 (补录分支: 仅发送, 飞机接收已剥离):
 *   CMD 0x20 STATUS   a_fwd, a_lat, x, y, theta (LEN=20) */

void car_comm_init(void);
void car_comm_send(f32 a_fwd, f32 a_lat, f32 x, f32 y, f32 theta);
