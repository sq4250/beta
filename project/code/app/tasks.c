/**
 * tasks.c — 四模式车端控制
 *
 * ┌──────────┬──────────┬────────────────────────────────┐
 * │ 模式      │ 航点来源   │ 启动方式                       │
 * ├──────────┼──────────┼────────────────────────────────┤
 * │ 1 DIRECT    │ 无        │ CMD 0x10 收到即执行             │
 * │ 2 FULL_AUTO │ 本地 TSP  │ 上电自跑                       │
 * │ 3 REMOTE_WP │ CMD 0x30  │ CMD 0x31                      │
 * │ 4 AUTO_START│ 本地 TSP  │ CMD 0x31 (飞机"开始跟踪"按钮)   │
 * └──────────┴──────────┴────────────────────────────────┘
 *
 * 共享: wp_queue(SPSC环形缓冲) → 到达滑窗 → NN → g_plan
 *       跟踪层(200Hz ISR): LQR横向 + LADRC纵向 → 舵机+电机
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
#include "core/kinematics.h"
#if CAR_MODE == 2 || CAR_MODE == 4
#include "core/tsp.h"
#endif
#include "car_comm.h"
#include "hal_led.h"
#include "utils.h"

/* ═══════════════════════════════════════════════════════════
 *  wp_queue — SPSC 环形缓冲 (ISR/init 生产, planner 消费)
 * ═══════════════════════════════════════════════════════════ */
#define WP_MASK  (WP_QUEUE_SIZE - 1)

static Waypoint    g_wp_queue[WP_QUEUE_SIZE];
static volatile u8 g_wp_head;   /* 生产者写入索引 */
static u8          g_wp_tail;   /* 消费者读取索引 */

static u8  wp_count(void)         { return (g_wp_head - g_wp_tail) & WP_MASK; }
static void wp_push(const Waypoint *wp) { g_wp_queue[g_wp_head] = *wp; g_wp_head = (g_wp_head + 1) & WP_MASK; }
static void wp_push_n(const Waypoint *wps, u8 n) { for (u8 i = 0; i < n; i++) wp_push(&wps[i]); }
static Waypoint wp_peek(u8 off)   { return g_wp_queue[(g_wp_tail + off) & WP_MASK]; }
static void wp_pop(void)          { g_wp_tail = (g_wp_tail + 1) & WP_MASK; }
static void wp_clear(void)        { g_wp_head = g_wp_tail = 0; }

/* ═══════════════════════════════════════════════════════════
 *  文件级状态
 * ═══════════════════════════════════════════════════════════ */
static CarState      g_car;        /* 真实车状态 (估计器输出) */
static CarState      g_vst;        /* 虚拟车状态 (NN 跟踪参考) */
static ImuData       g_imu_data;
static ActuatorCmd   g_cmd;
static PlannerAction g_plan;       /* NN 输出 / 直驱动作, ZOH 到跟踪层 */
static Encoder       g_enc;
static ImuHandle     g_imu;
static bool          g_ready;
static volatile u32  g_ms;
static f32           g_ey, g_ex;          /* 跟踪误差, ISR 写入 / debug 读取 */
static f32           g_w_eff, g_a_eff;    /* 前馈量, ISR 写入 / debug 读取 */
static bool          g_wp_active;  /* 航点执行进行中 */
static Waypoint       g_vst_prev;   /* 上周期 vst 位置 (线段碰撞检测起点) */
#if CAR_MODE == 3
static Waypoint       s_vst_visited_last;      /* 车端屏蔽, 单slot, 收WP时跳过 */
static bool           s_vst_visited_valid;
#endif

/* ── 本地航点 (MODE 2/4 用) ── */
#if CAR_MODE == 2 || CAR_MODE == 4
static const Waypoint g_local_targets[LOCAL_WP_COUNT] = {
    {1.715f, 0.815f}, {3.445f, 1.43f}, {4.565f, 0.095f},
    {2.965f, -0.075f}, {3.70f, -1.48f}, {1.91f, -1.09f},
};
#endif

/* ── 飞机数据消重 seq ── */
#if CAR_MODE == 1
static u32 s_last_a_seq;
#endif
#if CAR_MODE == 3
static u32 s_last_wp_seq;
static u32 s_last_start_seq;
static u32 s_last_stop_seq;
#endif
#if CAR_MODE == 4
static u32 s_last_start_seq;
static u32 s_last_stop_seq;
#endif

/* ═══════════════════════════════════════════════════════════
 *  前向声明
 * ═══════════════════════════════════════════════════════════ */
static void imu_read(ImuData *d, const ImuHandle imu);
static void tracking_layer_step(ActuatorCmd *cmd, CarState *vst, const CarState *car,
                                const PlannerAction *plan, f32 gyro_z);
static void actuators_apply(const ActuatorCmd *cmd);
static void task_20hz_planner(void);
static void task_10hz_debug(void);
static void task_1hz_heartbeat(void);

/* ═══════════════════════════════════════════════════════════
 *  IMU 观测 (1kHz ISR)
 * ═══════════════════════════════════════════════════════════ */
static void imu_read(ImuData *d, const ImuHandle imu) {
    hal_imu_read_all(d->gyro, d->accel, imu);
    d->has_quat = hal_imu_has_quat(imu);
    if (d->has_quat) hal_imu_read_quat(d->quat, imu);
}

/* ═══════════════════════════════════════════════════════════
 *  误差投影: e_world = VST − Car → RotZ(−θ_vst) → VST 系
 *
 *  FLU 约定: X+前 Y+左 Z+上
 *  e_x > 0: 参考车在前方 (应加速)
 *  e_y > 0: 参考车在左侧 → 真车偏右 (应左转)
 * ═══════════════════════════════════════════════════════════ */
static void ref_frame_error(f32 *ex, f32 *ey, const CarState *rs, const CarState *vst) {
    f32 dx = vst->x - rs->x;                    /* e_world = r - y */
    f32 dy = vst->y - rs->y;
    f32 ct = cosf(vst->theta), st = sinf(vst->theta);
    *ex =  dx * ct + dy * st;                   /* RotZ(-theta) row 0 */
    *ey = -dx * st + dy * ct;                   /* RotZ(-theta) row 1 */
}

static void tracking_layer_step(ActuatorCmd *cmd, CarState *vst,
                                const CarState *car,
                                const PlannerAction *plan, f32 gyro_z) {
    f32 a_eff, w_eff;
    mcu_kinematics_step(vst, plan->a, plan->omega, &a_eff, &w_eff);
    g_a_eff = a_eff; g_w_eff = w_eff;

    f32 e_x, e_y;
    ref_frame_error(&e_x, &e_y, car, vst);
    g_ex = e_x; g_ey = e_y;

    /* LQR 横向: 用物理层有效 omega 做前馈 */
    f32 omega_cmd = lateral_step(car, vst, w_eff, gyro_z, e_y);
    cmd->servo_delta = clamp(car->delta + omega_cmd * CTRL_DT, -SERVO_DELTA_MAX, SERVO_DELTA_MAX);

    /* LADRC 纵向: 用物理层有效加速度做前馈 */
    f32 thr_l, thr_r;
    longitudinal_step(&thr_l, &thr_r, car->v, vst->v, a_eff, e_x, car->delta);
    cmd->motor_l = thr_l;
    cmd->motor_r = thr_r;
}

static void actuators_apply(const ActuatorCmd *cmd) {
    hal_servo_set_delta(cmd->servo_delta);
    hal_motor_set_thr(cmd->motor_l, cmd->motor_r);
}

/* ═══════════════════════════════════════════════════════════
 *  wp_planner_step — NN 规划 + 到达滑窗 (MODE 2/3/4 共用)
 *
 *  队列头部为当前目标. vst 轨迹穿入目标圆 → 弹出 → 滑窗.
 *  取前 3 个航点送入 NN, 不足则重复末点.
 *  队列耗尽 → 刹车. MODE 2 停车, MODE 3/4 保持 active 等新数据.
 * ═══════════════════════════════════════════════════════════ */
#if CAR_MODE >= 2
static void wp_planner_step(void) {
    u8 cnt = wp_count();

#if CAR_MODE == 2 || CAR_MODE == 4
    /* 到达检测 + 弹窗 */
    if (cnt > 0) {
        Waypoint cur = wp_peek(0);
        if (check_hit_substep(g_vst_prev.x, g_vst_prev.y, g_vst.x, g_vst.y, cur.x, cur.y, TOL_XY)) {
            wp_pop();
            cnt = wp_count();
        }
    }
#endif

#if CAR_MODE == 3
    if (cnt > 0) {
        Waypoint cur = wp_peek(0);
        f32 dx = g_vst.x - cur.x, dy = g_vst.y - cur.y;
        if (dx*dx + dy*dy < TOL_XY * TOL_XY) {
            s_vst_visited_last = cur;
            s_vst_visited_valid = true;
        }
    }
#endif

    if (cnt == 0) {
        g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0.0f;
        return;
    }

    Waypoint g1 = wp_peek(0), g2, g3;
    f32 v2 = 0.0f, v3 = 0.0f;
    if (cnt > 1) { g2 = wp_peek(1); v2 = 1.0f; }
    if (cnt > 2) { g3 = wp_peek(2); v3 = 1.0f; }
    planner_forward(&g_plan, &g_vst, &g1, &g2, &g3, v2, v3);
    g_vst_prev = *(Waypoint*)&g_vst;
}
#endif

/* ═══════════════════════════════════════════════════════════
 *  init — 加载本地航点 (MODE 2/4), 推入队列
 * ═══════════════════════════════════════════════════════════ */
void tasks_init(void) {
    gpio_init(P23_7, GPO, 1, GPO_PUSH_PULL);
    hal_servo_init();
    hal_motor_init();
    hal_encoder_init();
    hal_led_init();
    longitudinal_init();
    car_comm_init();
    g_imu = hal_imu_create(&imu_660rc_driver);
    if (!g_imu) { while (1); }

    wp_clear();
    g_wp_active = false;
#if CAR_MODE == 3
    s_vst_visited_valid = false;
#endif
    g_plan.a = 0.0f; g_plan.omega = 0.0f;

    /* MODE 2/4: TSP 排序本地航点 → 推入队列 → 末尾追加起点 */
#if CAR_MODE == 2 || CAR_MODE == 4
    {
        Waypoint ordered[MAX_WAYPOINTS];
        tsp_solve(ordered, g_local_targets, LOCAL_WP_COUNT, CAR_START_X, CAR_START_Y);
        wp_push_n(ordered, LOCAL_WP_COUNT);
        Waypoint home = {CAR_START_X, CAR_START_Y};
        wp_push(&home);
    }
#endif

    /* MODE 2: 上电自跑. MODE 1/3/4: 等外部指令 */
#if CAR_MODE == 2
    g_wp_active = true;
    g_vst = g_car;
    g_vst_prev = *(Waypoint*)&g_car;
    lateral_reset();
#endif

    hal_encoder_get(&g_enc);
    g_ready = true;
}

/* ═══════════════════════════════════════════════════════════
 *  ISR 入口 (1kHz)
 * ═══════════════════════════════════════════════════════════ */
void car_control_update(void) {
    if (!g_ready) return;
    g_ms++;

    imu_read(&g_imu_data, g_imu);

    static u8 div_trk = 0;
    if (++div_trk >= TRACKER_DIV) { div_trk = 0; hal_encoder_get(&g_enc); }

    car_estimate_update(&g_car, &g_imu_data, &g_enc, &g_cmd);

    if (g_ms < STARTUP_DELAY_MS) return;

    if (div_trk == 0) {
        if (g_wp_active) {
            f32 dx = g_vst.x - g_car.x, dy = g_vst.y - g_car.y;
            if (dx*dx + dy*dy > 0.25f * 0.25f) {  /* 误差>25cm 重同步 */
                g_vst.x = g_car.x;
                g_vst.y = g_car.y;
                g_vst.theta = g_car.theta;
                g_vst.v = g_car.v;
                g_vst.delta = g_car.delta;
            }
            tracking_layer_step(&g_cmd, &g_vst, &g_car, &g_plan, g_imu_data.gyro[2]);
        } else {
            g_cmd.servo_delta = 0;
            g_cmd.motor_l = 0; g_cmd.motor_r = 0;
            g_vst = g_car;
        }
        actuators_apply(&g_cmd);
    }
}

/* ═══════════════════════════════════════════════════════════
 *  规划任务 (20Hz main)
 * ═══════════════════════════════════════════════════════════ */
static void task_20hz_planner(void) {
    car_comm_rx_t rx = car_comm_get();

/* ── MODE 1 DIRECT: CMD 0x10 世界加速度 → 机体投影 → 直驱 ── */
#if CAR_MODE == 1
    {
        static u32 s_stale;
        if (rx.a_seq != s_last_a_seq) { s_last_a_seq = rx.a_seq; s_stale = 0; }
        if (++s_stale >= 10) { rx.a_n = 0.0f; rx.a_w = 0.0f; }

        f32 ct = cosf(g_car.theta), st = sinf(g_car.theta);
        g_plan.a     = clamp(rx.a_n * ct + rx.a_w * st, -A_BRAKE_MAX, A_LONG_MAX);
        g_plan.omega = 0.0f;
    }

/* ── MODE 2 FULL_AUTO: 上电自跑, 跑完停车 ── */
#elif CAR_MODE == 2
    wp_planner_step();

/* ── MODE 3 REMOTE_WP: CMD 0x30 收航点 + CMD 0x31 启动 + CMD 0x32 停止复位 ── */
#elif CAR_MODE == 3
    if (rx.stop_seq != s_last_stop_seq) {
        s_last_stop_seq = rx.stop_seq;
        wp_clear();
        g_wp_active = false;
        g_vst = g_car;
        g_vst_prev = *(Waypoint*)&g_car;
        lateral_reset();
        s_vst_visited_valid = false;
        g_plan.a = 0.0f; g_plan.omega = 0.0f;
    }
    if (rx.wp_seq != s_last_wp_seq) {
        s_last_wp_seq = rx.wp_seq;
        wp_clear();
        /* 飞机发 cm, 车用 m: 转换. 跳过 visited_last, 但不能全空 */
        u8 pushed = 0;
        for (u8 i = 0; i < REMOTE_WP_COUNT; i++) {
            Waypoint w = { rx.wp[i].x * 0.01f, rx.wp[i].y * 0.01f };
            if (s_vst_visited_valid) {
                f32 dx = w.x - s_vst_visited_last.x;
                f32 dy = w.y - s_vst_visited_last.y;
                if (dx*dx + dy*dy < TOL_XY * TOL_XY) continue;
            }
            wp_push(&w);
            pushed++;
        }
        /* 防死锁: 全被visited_last过滤 → 全push, 不能空队 */
        if (pushed == 0) {
            for (u8 i = 0; i < REMOTE_WP_COUNT; i++) {
                Waypoint w = { rx.wp[i].x * 0.01f, rx.wp[i].y * 0.01f };
                wp_push(&w);
            }
        }
        /* 收WP即启动, 不依赖 CMD 0x31 */
        if (!g_wp_active && wp_count() > 0) {
            g_wp_active = true;
            g_vst = g_car;
            g_vst_prev = *(Waypoint*)&g_car;
            lateral_reset();
            printf("AUTO-START\r\n");
        }
        printf("RX30:%u q=%u active=%d\r\n", rx.wp_seq, wp_count(), g_wp_active);
    }
    if (rx.start_seq != s_last_start_seq) {
        s_last_start_seq = rx.start_seq;
        /* 首次启动重置 vst, 运行中收到 start 不重置 (防顿挫) */
        if (!g_wp_active) {
            g_vst = g_car;
            g_vst_prev = *(Waypoint*)&g_car;
            lateral_reset();
        }
        g_wp_active = true;
    }
    if (!g_wp_active) return;
    wp_planner_step();

/* ── MODE 4 AUTO_START: 本地航点 + CMD 0x31 启动 + CMD 0x32 停止复位 ── */
#elif CAR_MODE == 4
    if (rx.stop_seq != s_last_stop_seq) {
        s_last_stop_seq = rx.stop_seq;
        /* 复位: 清队列 → 重载本地航点 → 停车 → 等下次启动 */
        wp_clear();
        Waypoint ordered[MAX_WAYPOINTS];
        tsp_solve(ordered, g_local_targets, LOCAL_WP_COUNT, CAR_START_X, CAR_START_Y);
        wp_push_n(ordered, LOCAL_WP_COUNT);
        Waypoint home = {CAR_START_X, CAR_START_Y};
        wp_push(&home);
        g_wp_active = false;
        g_vst = g_car;
        g_vst_prev = *(Waypoint*)&g_car;
        lateral_reset();
        g_plan.a = 0.0f; g_plan.omega = 0.0f;
    }
    if (rx.start_seq != s_last_start_seq) {
        s_last_start_seq = rx.start_seq;
        g_wp_active = true;
        g_vst = g_car;
        g_vst_prev = *(Waypoint*)&g_car;
        lateral_reset();
    }
    if (!g_wp_active) return;
    wp_planner_step();
#endif
}

/* ═══════════════════════════════════════════════════════════
 *  上报任务 (20Hz main) — 网络跑完立刻发
 *
 *  体轴加速度:
 *    a_fwd = g_plan.a                         (纵向, NN输出)
 *    a_lat = vst.v² × tan(vst.delta) / L      (横向, 单车模型曲率)
 * ═══════════════════════════════════════════════════════════ */
static void task_20hz_report(void) {
    f32 a_fwd = g_plan.a;
    f32 curvature = tanf(g_vst.delta) * INV_WHEELBASE;
    f32 a_lat = g_vst.v * g_vst.v * curvature;

    car_comm_send(a_fwd, a_lat, g_vst.x * 100.0f, g_vst.y * 100.0f, g_car.theta);
}

/* ═══════════════════════════════════════════════════════════
 *  调试 (10Hz main)
 * ═══════════════════════════════════════════════════════════ */
static void task_10hz_debug(void) {
    static bool hdr = true;
    if (hdr) {
        printf("#bias=%.4fdeg/s  mode=%u  q=%u\r\n",
               (double)(hal_imu_gyro_bias_z(g_imu) * 57.29578f),
               (u32)CAR_MODE, wp_count());
        printf("t[s],vst_x[m],vst_y[m],car_x[m],car_y[m],vst_th[deg],car_th[deg],servo[rad],vst_delta[rad],vst_yaw[rad/s],car_yaw[rad/s],gyro_z[rad/s],w_ff[rad/s],a_ff[m/s2],vst_v[m/s],car_v[m/s],f_hat[m/s2]\r\n");
        hdr = false;
    }
    f32 vst_yaw  = bicycle_curvature(g_vst.v, g_vst.delta);
    f32 car_yaw  = bicycle_curvature(g_car.v, g_cmd.servo_delta);
    f32 gyro_z   = g_imu_data.gyro[2];
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           (f32)g_ms * 0.001f,
           (double)g_vst.x, (double)g_vst.y,
           (double)g_car.x, (double)g_car.y,
           (double)(g_vst.theta * 57.29578f), (double)(g_car.theta * 57.29578f),
           (double)g_cmd.servo_delta, (double)g_vst.delta,
           (double)vst_yaw, (double)car_yaw, (double)gyro_z,
           (double)g_w_eff, (double)g_a_eff,
           (double)g_vst.v, (double)g_car.v,
           (double)longitudinal_f_hat());
}

static void task_1hz_heartbeat(void) {
    gpio_toggle_level(P23_7);
}

Task tasks[TASK_NUM] = {
    {task_20hz_planner,   50,   1, 0},
    {task_20hz_report,    50,   1, 0},   /* planner 之后立刻上报 */
    {task_10hz_debug,     20,   1, 0},   /* 50Hz */
    {task_1hz_heartbeat, 1000,  1, 0},
};
