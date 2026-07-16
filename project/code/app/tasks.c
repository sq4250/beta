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
#include <string.h>
#include "tasks.h"
#include "hal_tick.h"

#include "hal_imu.h"
#include "imu_660rc.h"
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
static f32         g_gyro_yaw;             // 纯陀螺积分航向 [rad]
//===================================================文件级状态===================================================

//===================================================自动模式===================================================
static bool g_manual = false;                // false=自动NN模式, true=手动接收飞控指令
#define MAX_ROUNDS  10                       // 自动模式跑N圈
static u32 g_round = 0;                      // 当前圈数
static Waypoint g_wp_original[MAX_WAYPOINTS]; // 原始航点 (用于每圈重置)
static u32  g_wp_count = 0;                  // 原始航点数
//===================================================自动模式===================================================

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
static void task_5hz_state(void);
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

//===================================================初始化===================================================

void tasks_init(void) {
    gpio_init(P23_7, GPO, 1, GPO_PUSH_PULL);
    hal_servo_init();
    hal_motor_init();
    hal_encoder_init();
    hal_led_init();
    longitudinal_init();
    car_comm_init();
    g_imu = hal_imu_create(&imu_660rc_driver);
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

    /* 保存原始航点, 供自动模式每圈重置 */
    memcpy(g_wp_original, wp_ordered, wp_count * sizeof(Waypoint));
    g_wp_count = wp_count;

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

    g_gyro_yaw += g_imu_data.gyro[2] * ISR_DT;  /* 纯陀螺积分, 1kHz */

    if (g_ms < STARTUP_DELAY_MS) return;

    if (div_trk == 0) {
        tracking_layer_step(&g_cmd, &g_vst, &g_car, &g_plan,
                            g_imu_data.gyro[2]);
        actuators_apply(&g_cmd);
    }
}

//===================================================主循环任务===================================================

static void task_20hz_planner(void) {
    /* ── 手动模式: 接收飞控指令 ── */
    if (g_manual) {
        car_comm_rx_t rx = car_comm_get();
        g_plan.a     = clamp(rx.a * 0.01f, -A_MANUAL_MAX, A_MANUAL_MAX);
        g_plan.omega = clamp(rx.omega, -OMEGA_DELTA_MAX, OMEGA_DELTA_MAX);
        return;
    }

    /* ── 自动模式: NN规划, 跑 MAX_ROUNDS 圈后回到原点停车 ── */
    if (g_round >= MAX_ROUNDS) {
        g_plan.a     = -2.0f;   /* 减速停车, 约1s从 V_MAX→0 */
        g_plan.omega = 0.0f;
        return;
    }

    /* ① 到达检测 */
    if (waypoint_mgr_check_hit(&g_wp_mgr, &g_vst_prev, &g_vst)) {
        waypoint_mgr_mark_reached(&g_wp_mgr);
    }

    /* ② 获取窗口 + NN前向 */
    Waypoint g1, g2, g3;
    if (waypoint_mgr_get_window(&g_wp_mgr, &g1, &g2, &g3)) {
        planner_forward(&g_plan, &g_vst, &g1, &g2, &g3);
    } else {
        /* 所有航点已到达 → 本圈完成 */
        g_round++;
        if (g_round < MAX_ROUNDS) {
            waypoint_mgr_init(&g_wp_mgr, g_wp_original, g_wp_count);
            /* 新圈第一帧立即规划 */
            if (waypoint_mgr_get_window(&g_wp_mgr, &g1, &g2, &g3)) {
                planner_forward(&g_plan, &g_vst, &g1, &g2, &g3);
            }
        } else {
            /* MAX_ROUNDS 圈跑完, 减速停车 */
            g_plan.a     = -2.0f;
            g_plan.omega = 0.0f;
        }
    }

    /* ③ 记录本周期起点, 供下次到达检测使用 */
    g_vst_prev = g_vst;
}

static void task_10hz_debug(void) {
    /* 暂时关闭 */
}

static void task_5hz_state(void) {
    f32 yq = 0.0f;
    if (g_imu_data.has_quat) {
        f32 *q = g_imu_data.quat;
        yq = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]),
                    1.0f - 2.0f*(q[0]*q[0] + q[2]*q[2]));
        yq = wrap_pi(M_PI_F - yq) * 57.29578f;  /* 180°偏移 + 符号翻转, rad→deg */
    }
    printf("%.3f,%.3f,%.1f,%.1f,%.1f\r\n",
           (double)g_car.x, (double)g_car.y,
           (double)(g_car.theta * 57.29578f),
           (double)yq,
           (double)(g_gyro_yaw * 57.29578f));
}

static void task_1hz_heartbeat(void) {
    gpio_toggle_level(P23_7);
}

//===================================================任务表===================================================
Task tasks[TASK_NUM] = {
    {task_20hz_planner,    50,  1, 0},
    {task_10hz_debug,     100,  1, 0},
    {task_5hz_state,      200,  1, 0},
    {task_1hz_heartbeat, 1000,  1, 0},
};
