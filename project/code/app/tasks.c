#include "zf_common_headfile.h"
#include <string.h>
#include "tasks.h"
#include "hal_tick.h"
#include "hal_imu.h"
#include "imu_660rc.h"
#include "core/estimator.h"
#include "hal_servo.h"
#include "hal_motor.h"
#include "hal_encoder.h"
#include "core/lateral.h"
#include "core/longitudinal.h"
#include "core/kinematics.h"
#include "car_comm.h"
#include "hal_led.h"
#include "utils.h"

car_state_t      g_car, g_vst;
planner_action_t g_plan;
waypoint_t      g_car_prev;
bool          g_wp_active;
f32           g_ey, g_ex, g_delta_fb;
bool          g_vst_seeded;
volatile u32  g_ms;

static imu_data_t     s_imu;
static actuator_cmd_t s_cmd;
static encoder_t     s_enc;
static imu_handle_t   s_himu;
static bool        s_ready;

static void imu_read(imu_data_t *d, imu_handle_t h) {
    hal_imu_read_all(d->gyro, d->accel, h);
    d->has_quat = hal_imu_has_quat(h);
    if (d->has_quat) hal_imu_read_quat(d->quat, h);
}

static void frame_error(f32 *ex, f32 *ey, const car_state_t *r, const car_state_t *v) {
    f32 ct = cosf(v->theta), st = sinf(v->theta);
    *ex = (v->x - r->x) * ct + (v->y - r->y) * st;
    *ey = (r->x - v->x) * st - (r->y - v->y) * ct;
}

static void tracking(actuator_cmd_t *cmd, car_state_t *vst, const car_state_t *car,
                     const planner_action_t *plan, f32 gyro_z) {
    f32 a_eff, w_eff;
    mcu_kinematics_step(vst, plan->a, plan->omega, &a_eff, &w_eff);

    f32 ex, ey;
    frame_error(&ex, &ey, car, vst);
    g_ex = ex; g_ey = ey;

    /* LQR 横向: δ = δ_vst + Δδ_fb (前馈+反馈, 无积分器) */
    f32 delta_fb = lateral_step(car, vst, gyro_z, ey);
    g_delta_fb = delta_fb;
    cmd->servo_delta = clamp(vst->delta + delta_fb, -SERVO_DELTA_MAX, SERVO_DELTA_MAX);

    /* LADRC 纵向: 用物理层有效加速度做前馈 */
    f32 thr_l, thr_r;
    /* 关油门: car 进入任意硬编码 WP 点 10cm 范围 (路过也算, 防凸起打滑; 探索点无凸起不参与) */
    bool arrived = false;
    for (u8 i = 0; i < BEACON_COUNT; i++) {
        f32 dx = car->x - BEACON_WORLD[i][0] * 0.01f;
        f32 dy = car->y - BEACON_WORLD[i][1] * 0.01f;
        if (dx * dx + dy * dy < TOL_XY * TOL_XY) { arrived = true; break; }
    }
    longitudinal_step(&thr_l, &thr_r, car->v, vst->v, a_eff, ex, car->delta, arrived);
    cmd->motor_l = thr_l;
    cmd->motor_r = thr_r;
}

static void actuators(const actuator_cmd_t *cmd) {
    hal_servo_set_delta(cmd->servo_delta);
    hal_motor_set_thr(cmd->motor_l, cmd->motor_r);
}

void tasks_init(void) {
    gpio_init(P23_7, GPO, 1, GPO_PUSH_PULL);
    hal_servo_init(); hal_motor_init(); hal_encoder_init(); hal_led_init();
    longitudinal_init(); car_comm_init();

    s_himu = hal_imu_create(&imu_660rc_driver);
    if (!s_himu) while (1);

    extern void planner_init(void);
    planner_init();

    hal_encoder_get(&s_enc);
    s_ready = true;
}

void car_control_update(void) {
    if (!s_ready) return;
    g_ms++;
    imu_read(&s_imu, s_himu);

    static u8 div;
    if (++div >= TRACKER_DIV) { div = 0; hal_encoder_get(&s_enc); }

    /* 种子必须在 estimator 之前: 此时 car 是上一帧状态, 未被本帧更新 */
    static bool vst_seeded = false;
    if (!vst_seeded && div == 0) {
        car_estimate_set_yaw_offset(wrap_pi(g_car.theta));
        g_car.theta = 0.0f;
        g_vst = g_car;
        vst_seeded = true;
        g_vst_seeded = true;
    }

    car_estimate_update(&g_car, &s_imu, &s_enc, &s_cmd);

    if (div == 0) {
        {
            f32 dx = g_vst.x - g_car.x, dy = g_vst.y - g_car.y;
            if (dx*dx + dy*dy > 0.05f * 0.05f) {  /* 误差>5cm 重同步 */
                g_vst.x = g_car.x;
                g_vst.y = g_car.y;
                g_vst.theta = g_car.theta;
                g_vst.v = g_car.v;
                g_vst.delta = g_car.delta;
            }
        }
        tracking(&s_cmd, &g_vst, &g_car, &g_plan, s_imu.gyro[2]);
        actuators(&s_cmd);
    }
}

void task_20hz_report(void) {
    f32 curv = tanf(g_vst.delta) * INV_WHEELBASE;
    car_comm_send(g_plan.a, g_vst.v * g_vst.v * curv,
                  g_car.x * 100.0f, g_car.y * 100.0f, g_car.theta);
}

static void task_10hz_debug(void) {
#if 1

    static bool hdr = true;
    if (hdr) {
        printf("#bias=%.4fdeg/s mode=%u q=%u\r\n",
               (double)(hal_imu_gyro_bias_z(s_himu) * 57.29578f), (u32)CAR_MODE, wp_count());
        printf("t[s],vst_x[m],vst_y[m],car_x[m],car_y[m],vst_th[deg],car_th[deg],"
               "eth_d[rad/s],delta_fb[rad],v_car[m/s],v_vst[m/s],f_hat[m/s2]\r\n");
        hdr = false;
    }
    f32 eth_d = bicycle_curvature(g_vst.v, g_vst.delta) - s_imu.gyro[2];
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           (f32)g_ms * 0.001f,
           (double)g_vst.x, (double)g_vst.y,
           (double)g_car.x, (double)g_car.y,
           (double)(g_vst.theta * 57.29578f), (double)(g_car.theta * 57.29578f),
           (double)eth_d,
           (double)g_delta_fb,
           (double)g_car.v, (double)g_vst.v,
           (double)longitudinal_f_hat());
#endif
}

static void task_1hz_heartbeat(void) { gpio_toggle_level(P23_7); }

task_t tasks[TASK_NUM] = {
    {task_20hz_planner, 50, 1, 0},
    {task_20hz_report,  50, 1, 0},
    {task_10hz_debug,  100, 1, 0},
    {task_1hz_heartbeat, 1000, 1, 0},
};
