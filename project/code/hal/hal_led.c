/**
 * hal_led.c — LED 灯条 PWM 驱动 (TCPWM_CH14_P00_2, 17kHz)
 *
 * pwm_set_duty 范围 0~10000, 对应 0%~100% 占空比
 */
#include "hal_led.h"
#include "zf_driver_pwm.h"

#define LED_TCPWM_CH   TCPWM_CH14_P00_2
#define LED_FREQ_HZ    17000
#define LED_DUTY_MAX   10000u

void hal_led_init(void) {
    pwm_init(LED_TCPWM_CH, LED_FREQ_HZ, 5000);  /* 50% 占空比 */
}

void hal_led_set(f32 brightness) {
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;
    pwm_set_duty(LED_TCPWM_CH, (u32)(brightness * (f32)LED_DUTY_MAX));
}
