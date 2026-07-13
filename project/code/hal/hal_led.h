/**
 * hal_led.h — LED 灯条 PWM 驱动 (TCPWM_CH14_P00_2)
 */
#ifndef HAL_LED_H
#define HAL_LED_H
#include "common.h"

void hal_led_init(void);
void hal_led_set(f32 brightness);  /* 0.0~1.0 */

#endif
