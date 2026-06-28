/**
 * hal_servo.c — 前轮转向舵机 (200Hz PWM)
 */
#include "hal_servo.h"
#include "config.h"
#include "zf_common_headfile.h"
#include "utils.h"

static u8  g_init = 0;
static f32 g_inv_steering_ratio;  // 1/STEERING_RATIO (init 计算)

void hal_servo_init(void) {
    if (g_init) return; g_init = 1;
    g_inv_steering_ratio = 1.0f / STEERING_RATIO;
    u32 center_duty = (u32)((f32)SERVO_CTR_US * SERVO_DUTY_PER_US);
    pwm_init(SERVO_TCPWM_CH, SERVO_FREQ_HZ, center_duty);
}

void hal_servo_set_delta(f32 delta) {
    f32 srad  = delta * g_inv_steering_ratio;
    srad      = clamp(srad, -SERVO_RAD_MAX, SERVO_RAD_MAX);
    f32 pulse = (f32)SERVO_CTR_US + srad * SERVO_US_PER_RAD;
    u32 duty  = (u32)(pulse * SERVO_DUTY_PER_US);
    pwm_set_duty(SERVO_TCPWM_CH, duty);
}
