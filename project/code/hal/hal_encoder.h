/**
 * hal_encoder.h — 双后轮编码器
 */
#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H
#include "common.h"

void hal_encoder_init(void);
void hal_encoder_get(Encoder *enc);  // 读累计值+清零, 已乘标定系数

#endif
