/**
 * tasks.c — 四层架构胶水
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │ 状态观测 (1kHz ISR)  IMU偏航 + 编码器ZOH → v,x,y,θ,δ   │
 * │ 航点层 (20Hz main)  到达检测 + 3 窗口获取              │
 * │ 规划层 (20Hz main)  车体变换 + NN → ZOH 动作           │
 * │ 跟踪层 (200Hz ISR)  只读 car, 推进 vst, 输出控制量     │
 * │ 执行层 (200Hz ISR)  舵机 PWM + 电机 PWM                │
 * └─────────────────────────────────────────────────────────┘
 */

#include "zf_common_headfile.h"
#include "tasks.h"
#include "hal_tick.h"

#include "hal_imu.h"
#include "imu_660rc.h"
#include "core/estimator.h"
#include "hal_servo.h"
#include "hal_motor.h"
#include "hal_encoder.h"

#include "core/planner.h"
#include "core/tracker.h"
#include "core/longitudinal.h"
#include "core/waypoint_mgr.h"
#include "core/kinematics.h"
#include "utils.h"

//===================================================文件级状态===================================================
static CarState    g_car;
static CarState    g_vst;
static ImuData     g_imu_data;
static ActuatorCmd g_cmd;
static CarState    g_vst_prev;      // 上周期虚拟车起点 (航点线段检测, 复用 CarState)
static PlannerAction g_plan;         // NN 动作 ZOH (20Hz 更新)
static Encoder     g_enc_zoh;       // 编码器 ZOH
static WaypointMgr g_wp_mgr;        // 航点管理 (调用方持有)
static ImuHandle   g_imu;
static u8          g_ready;
//===================================================文件级状态===================================================

//===================================================航点===================================================
static const Waypoint g_waypoints[] = {
    {4.0f, 0.0f}, {4.0f, 2.0f}, {0.0f, 2.0f}, {0.0f, 4.0f},
    {2.0f, 6.0f}, {5.0f, 5.0f}, {6.0f, 2.0f}, {3.0f,-1.0f},
};
//===================================================航点===================================================

//===================================================层入口声明===================================================
static void imu_read(ImuData *d, const ImuHandle imu);
static void tracking_layer_step(ActuatorCmd *cmd, CarState *vst, const CarState *car,
                                 const PlannerAction *plan);
static void actuators_apply(const ActuatorCmd *cmd);

static void task_20hz_planner(void);
static void task_10hz_debug(void);
static void task_1hz_heartbeat(void);
//===================================================层入口声明===================================================

//===================================================IMU 观测 (1kHz)===================================================

static void imu_read(ImuData *d, const ImuHandle imu) {
    hal_imu_read_gyro(d->gyro, imu);
    d->has_quat = hal_imu_has_quat(imu);
    if (d->has_quat) hal_imu_read_quat(d->quat, imu);
}

//===================================================跟踪层 (200Hz, 只读 car)===================================================

// 更新 vst + cmd, 不写 car
static void tracking_layer_step(ActuatorCmd *cmd, CarState *vst, const CarState *car,
                                 const PlannerAction *plan) {
    mcu_kinematics_step(vst, plan->a, plan->omega);

    f32 omega_cmd = tracker_step(car, vst, plan->omega, g_imu_data.gyro[2]);
    f32 e_x       = tracker_get_ex();
    cmd->servo_delta = clamp(car->delta + omega_cmd * CTRL_DT, -DELTA_MAX, DELTA_MAX);
    f32 thr_L, thr_R;
    longitudinal_step(&thr_L, &thr_R, car->v, vst->v, plan->a, e_x, car->delta);
    cmd->motor_l = thr_L;
    cmd->motor_r = thr_R;
}

//===================================================执行层 (200Hz)===================================================

static void actuators_apply(const ActuatorCmd *cmd) {
    hal_servo_set_delta(cmd->servo_delta);
    hal_motor_set_thr(cmd->motor_l, cmd->motor_r);
}

//===================================================初始化===================================================

void tasks_init(void) {
    hal_servo_init();
    hal_motor_init();
    hal_encoder_init();
    g_imu = hal_imu_create(&imu_660rc_driver);

    waypoint_mgr_init(&g_wp_mgr, g_waypoints,
        sizeof(g_waypoints)/sizeof(g_waypoints[0]));
    tracker_init();

    memset(&g_vst, 0, sizeof(g_vst));
    g_vst = g_car;
    g_vst_prev = g_car;

    {
        Waypoint g1, g2, g3;
        if (waypoint_mgr_get_window(&g_wp_mgr, &g1, &g2, &g3)) {
            planner_forward(&g_vst, &g1, &g2, &g3, &g_plan);
        }
    }

    g_ready = 1;
}

//===================================================ISR 控制===================================================

void car_control_update(void) {
    if (!g_ready) return;

    // ── 1kHz: IMU 采样 ──
    imu_read(&g_imu_data, g_imu);

    // ── 编码器 ZOH: 200Hz 读, 其他 tick 保持 ──
    static u8 div200 = 0;
    if (++div200 >= 5) {
        div200 = 0;
        hal_encoder_get(&g_enc_zoh);
    }

    // ── 1kHz: 状态估计 (测量 + 上周期控制量 → 5状态) ──
    car_estimate_update(&g_car, &g_imu_data, &g_enc_zoh, &g_cmd);

    // ── 200Hz: 跟踪层 + 执行层 ──
    if (div200 == 0) {
        tracking_layer_step(&g_cmd, &g_vst, &g_car, &g_plan);
        actuators_apply(&g_cmd);
    }
}

//===================================================主循环任务===================================================

static void task_20hz_planner(void) {
    if (waypoint_mgr_check_hit(&g_wp_mgr, &g_vst_prev, &g_vst)) {
        waypoint_mgr_mark_reached(&g_wp_mgr);
    }
    Waypoint g1, g2, g3;
    if (!waypoint_mgr_get_window(&g_wp_mgr, &g1, &g2, &g3)) return;
    planner_forward(&g_vst, &g1, &g2, &g3, &g_plan);
    g_vst_prev = g_vst;
}

static void task_10hz_debug(void) {
    u32 wp_done = waypoint_mgr_reached_count(&g_wp_mgr);
    f32 ey = tracker_get_ey();
    f32 omg = tracker_get_omega_cmd();
    (void)wp_done; (void)ey; (void)omg;
}

static void task_1hz_heartbeat(void) {
    // LED toggle, watchdog
}

//===================================================任务表===================================================
Task tasks[TASK_NUM] = {
    {task_20hz_planner,   50,   1, 0},
    {task_10hz_debug,    100,   1, 0},
    {task_1hz_heartbeat, 1000,  1, 0},
};
