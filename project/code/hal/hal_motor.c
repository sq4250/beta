/**
 * hal_motor.c — 双后轮电机 (PWM + 方向IO)
 */
#include "hal_motor.h"
#include "zf_common_headfile.h"

#define MOTOR_L_PWM_CH   TCPWM_CH01_P06_3
#define MOTOR_R_PWM_CH   TCPWM_CH02_P06_5
#define MOTOR_L_DIR      P06_4
#define MOTOR_R_DIR      P06_6
#define MOTOR_L_POLARITY 1
#define MOTOR_R_POLARITY 1
#define MOTOR_PWM_FREQ   17000

static void set_thr(pwm_channel_enum ch, gpio_pin_enum dir, u8 polarity, f32 thr) {
    i32 pwm = (i32)(thr * (f32)PWM_DUTY_MAX);
    if (pwm >= 0) {
        gpio_set_level(dir, polarity);
    } else {
        gpio_set_level(dir, !polarity); pwm = -pwm;
    }
    if (pwm > (i32)PWM_DUTY_MAX) pwm = (i32)PWM_DUTY_MAX;
    pwm_set_duty(ch, (u32)pwm);
}

void hal_motor_init(void) {
    pwm_init(MOTOR_L_PWM_CH, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_R_PWM_CH, MOTOR_PWM_FREQ, 0);
    gpio_init(MOTOR_L_DIR, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_DIR, GPO, 0, GPO_PUSH_PULL);
}

void hal_motor_set_thr(f32 thr_l, f32 thr_r) {
    set_thr(MOTOR_L_PWM_CH, MOTOR_L_DIR, MOTOR_L_POLARITY, thr_l);
    set_thr(MOTOR_R_PWM_CH, MOTOR_R_DIR, MOTOR_R_POLARITY, thr_r);
}
