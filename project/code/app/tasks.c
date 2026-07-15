/**
 * tasks.c — 四层架构胶水
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │ 状态观测 (1kHz ISR)  IMU偏航 + 编码器ZOH → v,x,y,θ,δ   │
 * │ 航点层 (20Hz main)  到达检测 + 3 窗口获取              │
 * │ 规划层 (20Hz main)  车体变换 + NN → ZOH 动作           │
 * │ 跟踪层 (200Hz ISR)  推进 VST, 时间同步 LQR+LADRC       │
 * │ 执行层 (200Hz ISR)  舵机 PWM + 电机 PWM                │
 * └─────────────────────────────────────────────────────────┘
 *
 * 手动/NN模式 (g_manual): 接收飞控指令 CMD 0x10
 *   an[cm/s²] → 纵向加速度, aw[rad/s] → 转向角速度
 */

#include "zf_common_headfile.h"
#include "tasks.h"
#include "hal_tick.h"

#include "hal_imu.h"
#include "imu_660ra.h"
#include "core/estimator.h"
#include "hal_servo.h"
#include "hal_motor.h"
#include "hal_encoder.h"

#include "core/planner.h"
#include "core/lateral.h"
#include "core/longitudinal.h"
#include "core/waypoint_mgr.h"
#include "core/kinematics.h"
#include "core/tsp.h"
#include "car_comm.h"
#include "hal_led.h"
#include "utils.h"

//===================================================文件级状态===================================================
static CarState    g_car;
static CarState    g_vst;
static ImuData     g_imu_data;
static ActuatorCmd g_cmd;
static CarState    g_vst_prev;
static PlannerAction g_plan;
static Encoder     g_enc;
static WaypointMgr g_wp_mgr;
static ImuHandle   g_imu;
static bool        g_ready;
static volatile u32 g_ms;
static f32         g_ey, g_ex;
static f32         g_v_cmd;
//===================================================文件级状态===================================================

//===================================================手动模式开关===================================================
static bool g_manual = true;
//===================================================手动模式开关===================================================


//===================================================航点===================================================
#define CAR_START_X  0.0f
#define CAR_START_Y  0.0f

static const Waypoint g_targets[] = {
    {1.21f, 0.50f}, {3.80f, 1.17f}, {2.63f, -2.0f}, {4.28f, -2.28f}, {4.53f, -0.35f},
};
//===================================================航点===================================================

//===================================================层入口声明===================================================
static void imu_read(ImuData *d, const ImuHandle imu);
static void tracking_layer_step(ActuatorCmd *cmd, CarState *vst, const CarState *car,
                                const PlannerAction *plan, f32 gyro_z);
static void actuators_apply(const ActuatorCmd *cmd);

static void task_20hz_planner(void);
static void task_10hz_debug(void);
static void task_1hz_heartbeat(void);
//===================================================层入口声明===================================================

//===================================================IMU 观测 (1kHz)===================================================

static void imu_read(ImuData *d, const ImuHandle imu) {
    hal_imu_read_all(d->gyro, d->accel, imu);
    d->has_quat = hal_imu_has_quat(imu);
    if (d->has_quat) hal_imu_read_quat(d->quat, imu);
}

//===================================================误差投影===================================================

static void ref_frame_error(f32 *ex, f32 *ey,
                            const CarState *rs, const CarState *vst) {
    f32 ct = cosf(vst->theta), st = sinf(vst->theta);
    *ex = (vst->x - rs->x) * ct + (vst->y - rs->y) * st;
    *ey = (rs->x - vst->x) * st - (rs->y - vst->y) * ct;
}

//===================================================跟踪层 (200Hz)===================================================

static void tracking_layer_step(ActuatorCmd *cmd, CarState *vst,
                                const CarState *car,
                                const PlannerAction *plan, f32 gyro_z) {
    mcu_kinematics_step(vst, plan->a, plan->omega);

    f32 e_x, e_y;
    ref_frame_error(&e_x, &e_y, car, vst);
    g_ex = e_x; g_ey = e_y;

    f32 omega_cmd = lateral_step(car, vst, plan->omega, gyro_z, e_y);
    cmd->servo_delta = clamp(car->delta + omega_cmd * CTRL_DT, -DELTA_MAX, DELTA_MAX);

    f32 a_ref = plan->a;
    if (vst->v >= V_MAX && a_ref > 0.0f) a_ref = 0.0f;
    if (vst->v <= 0.0f && a_ref < 0.0f) a_ref = 0.0f;

    f32 thr_l, thr_r;
    longitudinal_step(&thr_l, &thr_r, car->v, vst->v, a_ref, e_x, car->delta);
    cmd->motor_l = thr_l;
    cmd->motor_r = thr_r;
}

//===================================================执行层 (200Hz)===================================================

static void actuators_apply(const ActuatorCmd *cmd) {
    hal_servo_set_delta(cmd->servo_delta);
    hal_motor_set_thr(cmd->motor_l, cmd->motor_r);
}

//===================================================跟踪重启 (新会话首帧)===================================================

static void tracking_restart(void) {
    g_ready = false;

    // VST 归零 — 车端自己积分，不从飞机同步
    g_vst      = (CarState){0};
    g_vst_prev = (CarState){0};

    // 惯导归零 — 估计位置/速度/航向清零
    g_car = (CarState){0};
    estimator_reset();

    // 纵向控制器状态归零 — LESO 重新收敛
    longitudinal_init();

    // 编码器重新取基线
    hal_encoder_get(&g_enc);

    // 指令清零 — 等下一帧覆盖
    g_plan = (PlannerAction){0};

    g_ready = true;
}

//===================================================初始化===================================================

void tasks_init(void) {
    gpio_init(P23_7, GPO, 1, GPO_PUSH_PULL);
    hal_servo_init();
    hal_motor_init();
    hal_encoder_init();
    hal_led_init();
    longitudinal_init();
    car_comm_init();
    g_imu = hal_imu_create(&imu_660ra_driver);
    if (!g_imu) { while(1); }

    u32  wp_count = sizeof(g_targets) / sizeof(g_targets[0]);
    Waypoint wp_ordered[MAX_WAYPOINTS];
    tsp_solve(wp_ordered, g_targets, wp_count, CAR_START_X, CAR_START_Y);

    if (wp_count < MAX_WAYPOINTS) {
        wp_ordered[wp_count].x = CAR_START_X;
        wp_ordered[wp_count].y = CAR_START_Y;
        wp_count++;
    }

    waypoint_mgr_init(&g_wp_mgr, wp_ordered, wp_count);

    g_vst = g_car;
    g_vst_prev = g_car;

    if (!g_manual) {
        Waypoint g1, g2, g3;
        if (waypoint_mgr_get_window(&g_wp_mgr, &g1, &g2, &g3)) {
            planner_forward(&g_plan, &g_vst, &g1, &g2, &g3);
        }
    }

    hal_encoder_get(&g_enc);
    g_ready = true;
}

//===================================================ISR 控制 (1kHz)===================================================

void car_control_update(void) {
    if (!g_ready) return;
    g_ms++;

    imu_read(&g_imu_data, g_imu);

    static u8 div_trk = 0;
    if (++div_trk >= TRACKER_DIV) {
        div_trk = 0;
        hal_encoder_get(&g_enc);
    }

    car_estimate_update(&g_car, &g_imu_data, &g_enc, &g_cmd);

    if (g_ms < STARTUP_DELAY_MS) return;

    if (div_trk == 0) {
        tracking_layer_step(&g_cmd, &g_vst, &g_car, &g_plan,
                            g_imu_data.gyro[2]);
        actuators_apply(&g_cmd);
    }
}

//===================================================主循环任务===================================================

static void task_20hz_planner(void) {
    car_comm_rx_t rx = car_comm_get();

    // ── 停止跟踪 = 直接重置 VST + 惯导 ──
    if (car_comm_reset_pending()) {
        tracking_restart();
    }

    g_plan.a     = clamp(rx.a * 0.01f, -A_MANUAL_MAX, A_MANUAL_MAX);
    g_plan.omega = clamp(rx.omega, -OMEGA_DELTA_MAX, OMEGA_DELTA_MAX);

    // ── 回传惯导坐标 (m→cm) ──
    car_comm_send(g_car.x * 100.0f, g_car.y * 100.0f);
}

static void task_10hz_debug(void) {
    car_comm_rx_t rx = car_comm_get();
    printf("RX seq=%lu a=%.2f w=%.2f | PLAN a=%.2f w=%.2f\r\n",
           (unsigned long)rx.seq,
           (double)(rx.a * 0.01f), (double)rx.omega,
           (double)g_plan.a, (double)g_plan.omega);
}

static void task_1hz_heartbeat(void) {
    gpio_toggle_level(P23_7);
}

//===================================================任务表===================================================
Task tasks[TASK_NUM] = {
    {task_20hz_planner,    50,  1, 0},
    {task_10hz_debug,     100,  1, 0},
    {task_1hz_heartbeat, 1000,  1, 0},
};
