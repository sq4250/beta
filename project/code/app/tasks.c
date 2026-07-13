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
#include "car_comm.h"
#include "utils.h"

//===================================================文件级状态===================================================
static CarState    g_car;
static CarState    g_vst;
static ImuData     g_imu_data;
static ActuatorCmd g_cmd;
static CarState    g_vst_prev;      // 上周期虚拟车起点 (航点线段检测, 复用 CarState)
static PlannerAction g_plan;         // NN 动作 ZOH (20Hz 更新)
static Encoder     g_enc;            // 编码器读数
static WaypointMgr g_wp_mgr;        // 航点管理 (调用方持有)
static ImuHandle   g_imu;
static bool        g_ready;
static volatile u32 g_ms;          // ISR tick 计数器 (1ms)
static f32         g_ey, g_ex;     // 车体坐标系跟踪误差 (ISR 写入, debug 读取)
static f32         g_ax_cmd;       // 飞控指令加速度缓存 [m/s²]
//===================================================文件级状态===================================================

//===================================================手动模式开关===================================================
static bool g_manual = true;  /* true=飞控ax直驱, false=NN自动驾驶 */
//===================================================手动模式开关===================================================

//===================================================航点===================================================
static const Waypoint g_waypoints[] = {
    {2.0f, 0.0f}, {2.0f, 2.0f}, {0.0f, 2.0f}, {0.0f, 0.0f},
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

// 真实车→虚拟车误差投影到参考系 (e = r - y = vst - rs)
static void ref_frame_error(f32 *ex, f32 *ey,
                            const CarState *rs, const CarState *vst
    ) {
    f32 ct = cosf(vst->theta), st = sinf(vst->theta);
    *ex = (vst->x - rs->x) * ct + (vst->y - rs->y) * st;
    *ey = (rs->x - vst->x) * st - (rs->y - vst->y) * ct;
}

//===================================================跟踪层 (200Hz, 时间同步)===================================================

static void tracking_layer_step(ActuatorCmd *cmd, CarState *vst,
                                const CarState *car,
                                const PlannerAction *plan, f32 gyro_z
    ) {
    mcu_kinematics_step(vst, plan->a, plan->omega);

    f32 e_x, e_y;
    ref_frame_error(&e_x, &e_y, car, vst);
    g_ex = e_x; g_ey = e_y;  // ISR 写入, debug 任务读取

    f32 omega_cmd = lateral_step(car, vst, plan->omega, gyro_z, e_y);
    cmd->servo_delta = clamp(car->delta + omega_cmd * CTRL_DT, -DELTA_MAX, DELTA_MAX);

    f32 a_ref = plan->a;
    if ((vst->v <= 0.0f && a_ref < 0.0f) || (vst->v >= V_MAX && a_ref > 0.0f))
        a_ref = 0.0f;

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
    gpio_init(P23_7, GPO, 1, GPO_PUSH_PULL);  // LED (低电平亮)
    hal_servo_init();
    hal_motor_init();
    hal_encoder_init();
    longitudinal_init();
    car_comm_init();
    g_imu = hal_imu_create(&imu_660ra_driver);
    if (!g_imu) { while(1); }  // IMU 初始化失败 → 终止, 避免盲开

    waypoint_mgr_init(&g_wp_mgr, g_waypoints,
        sizeof(g_waypoints)/sizeof(g_waypoints[0]));

    g_vst = g_car;
    g_vst_prev = g_car;

    {
        Waypoint g1, g2, g3;
        if (waypoint_mgr_get_window(&g_wp_mgr, &g1, &g2, &g3)) {
            planner_forward(&g_plan, &g_vst, &g1, &g2, &g3);
        }
    }

    hal_encoder_get(&g_enc);  // 清零标定期间累积, 确保消费者从首次 ISR 拿到的窗口 ≤1ms
    g_ready = true;
}

//===================================================ISR 控制===================================================

void car_control_update(void) {
    if (!g_ready) return;
    g_ms++;

    // ── 1kHz: IMU 采样 ──
    imu_read(&g_imu_data, g_imu);

    // ── 编码器: TRACKER_FREQ 读取, 其他 tick 保持 ──
    static u8 div_trk = 0;
    if (++div_trk >= TRACKER_DIV) {
        div_trk = 0;
        hal_encoder_get(&g_enc);
    }

    // ── 1kHz: 状态估计 (测量 + 上周期控制量 → 5状态) ──
    car_estimate_update(&g_car, &g_imu_data, &g_enc, &g_cmd);

    // ── TRACKER_FREQ: 跟踪层 + 执行层 ──
    if (div_trk == 0) {
        tracking_layer_step(&g_cmd, &g_vst, &g_car, &g_plan,
                            g_imu_data.gyro[2]);
        actuators_apply(&g_cmd);
    }
}

//===================================================主循环任务===================================================

static void task_20hz_planner(void) {
    if (g_manual) {
        /* 手动模式: 飞控 ax → plan.a, ω≡0, 跟踪层自动跟上 */
        static u32 s_last_seq = 0;
        static u32 s_stale    = 0;
        car_comm_rx_t rx = car_comm_get();
        if (rx.seq != s_last_seq) {
            s_last_seq = rx.seq;
            s_stale    = 0;
        }
        g_ax_cmd = rx.ax * 0.01f;                  /* cm/s² → m/s² */
        if (++s_stale >= 10) g_ax_cmd = -A_LONG_MAX; /* 500ms 超时 → 全力制动 */
        g_plan.a    = clamp(g_ax_cmd, -A_LONG_MAX, A_LONG_MAX);
        g_plan.omega = 0.0f;
        return;
    }

    if (waypoint_mgr_check_hit(&g_wp_mgr, &g_vst_prev, &g_vst)) {
        waypoint_mgr_mark_reached(&g_wp_mgr);
    }
    Waypoint g1, g2, g3;
    if (!waypoint_mgr_get_window(&g_wp_mgr, &g1, &g2, &g3)) {
        g_plan.a = -A_LONG_MAX; g_plan.omega = 0.0f;  // 航点耗尽 → 最大制动停车
        return;
    }
    planner_forward(&g_plan, &g_vst, &g1, &g2, &g3);
    g_vst_prev = g_vst;
}

static void task_10hz_debug(void) {
    // ── 航点最小距离追踪 (idx切换时激活, 到达后继续追踪直到报告) ──
    static u32  wp_last_reached = 0;
    static f32  wp_min_d[MAX_WAYPOINTS];
    static u32  wp_active = 0;  // bitmask: 哪些航点正在被追踪
    // 当前目标首次出现时激活追踪 + 重置 min_d
    u32 ci = g_wp_mgr.idx;
    if (ci < g_wp_mgr.count && !(wp_active & (1u << ci))) {
        wp_min_d[ci] = 1e9f;
        wp_active |= (1u << ci);
    }
    // 更新所有活跃航点的最小距离 (包括已到达但未报告的)
    for (u32 i = 0; i < g_wp_mgr.count; ++i) {
        if (!(wp_active & (1u << i))) continue;
        f32 dx = g_car.x - g_wp_mgr.wps[i].x;
        f32 dy = g_car.y - g_wp_mgr.wps[i].y;
        f32 d  = sqrtf(dx*dx + dy*dy);
        if (d < wp_min_d[i]) wp_min_d[i] = d;
    }
    // 检测新到达的航点并报告, 报告后停止追踪该WP
    u32 reached_now = waypoint_mgr_reached_count(&g_wp_mgr);
    while (wp_last_reached < reached_now) {
        u32 i = wp_last_reached;
        printf("#WP%u min_d=%.4fm\n", i, (double)wp_min_d[i]);
        wp_active &= ~(1u << i);  // 停止追踪, 释放 min_d
        wp_last_reached++;
    }

    // ── 常规状态输出 ──
    static bool hdr = true;
    if (hdr) {
        printf("#bias=%.4fdeg/s\r\n", (double)(hal_imu_gyro_bias_z(g_imu) * 57.29578f));
        if (g_manual)
            printf("t[s],x[m],y[m],th[rad],v[m/s],ax_cmd[m/s2],delta[rad]\r\n");
        else
            printf("t[s],x[m],y[m],th[rad],v[m/s],ey[m],ex[m],delta[rad],vst_x[m],vst_y[m],vst_th[rad]\r\n");
        hdr = false;
    }
    if (g_manual)
        printf("%.3f,%.3f,%.3f,%.4f,%.3f,%.4f,%.4f\r\n",
            (f32)g_ms * 0.001f,
            g_car.x, g_car.y, g_car.theta, g_car.v,
            g_ax_cmd, g_car.delta);
    else
        printf("%.3f,%.3f,%.3f,%.4f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.4f\r\n",
            (f32)g_ms * 0.001f,
            g_car.x, g_car.y, g_car.theta, g_car.v,
            g_ey, g_ex, g_car.delta,
            g_vst.x, g_vst.y, g_vst.theta);
    car_comm_send(&g_car, g_ey);
}

static void task_1hz_heartbeat(void) {
    gpio_toggle_level(P23_7);  // LED 闪烁 (0.5Hz), 证明调度器存活
}

//===================================================任务表===================================================
Task tasks[TASK_NUM] = {
    {task_20hz_planner,   50,   1, 0},
    {task_10hz_debug,    100,   1, 0},
    {task_1hz_heartbeat, 1000,  1, 0},
};
