/**
 * tasks.c — 四层架构胶水
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │ 状态观测 (1kHz ISR) 5状态: x,y,θ(INS) v(编码器) δ=指令 │
 * │ 航点层 (20Hz main)  到达检测 + 3窗口获取               │
 * │ 规划层 (20Hz main)  车体变换 + NN → ZOH 动作           │
 * │ 跟踪层 (200Hz ISR)  虚拟车推进 + LQR + LADRC → 控制量  │
 * │ 执行层 (200Hz ISR)  舵机PWM + 电机PWM                  │
 * └─────────────────────────────────────────────────────────┘
 */

#include "zf_common_headfile.h"
#include "tasks.h"
#include "hal_tick.h"

#include "hal_imu.h"
#include "imu_660rc.h"
#include "core/ins.h"
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
static CarState    g_car;          // 真实小车状态 (传感器→估计→跟踪)
static CarState    g_vst;          // 虚拟小车状态 (跟踪层持有, 仅同步一次)
static SensorData  g_sens;         // 传感器采样
static ActuatorCmd g_cmd;          // 执行器指令
static f32         g_vst_prev[2];  // 上周期虚拟车起点 (航点层线段检测)
static ImuHandle   g_imu;
static u8          g_ready;
//===================================================文件级状态===================================================

//===================================================航点===================================================
static const Waypoint g_waypoints[] = {
    {4.0f, 0.0f}, {4.0f, 2.0f}, {0.0f, 2.0f}, {0.0f, 4.0f},
    {2.0f, 6.0f}, {5.0f, 5.0f}, {6.0f, 2.0f}, {3.0f,-1.0f},
};
#define WP_COUNT (sizeof(g_waypoints)/sizeof(g_waypoints[0]))
//===================================================航点===================================================

//===================================================层入口声明===================================================
// 传感器→估计 (1kHz)
static void sensors_read(SensorData *s);
static void state_estimate(CarState *car, const SensorData *s);

// 跟踪层 (200Hz): 虚拟车推进 + LQR + LADRC → 控制量
static void tracking_layer_step(CarState *vst, CarState *car, ActuatorCmd *cmd);

// 5 状态观测 (1kHz): x,y,theta,v 来自 INS + 编码器; delta = 上周期舵机指令 (跟踪良好)
static void car_state_observe(CarState *car, const SensorData *s, f32 delta_cmd);

// 执行层 (200Hz)
static void actuators_apply(const ActuatorCmd *cmd);

// 主循环任务
static void task_20hz_planner(void);
static void task_10hz_debug(void);
static void task_1hz_heartbeat(void);
//===================================================层入口声明===================================================

//===================================================传感器→观测 实现===================================================

static void sensors_read(SensorData *s) {
    Encoder enc; hal_encoder_get(&enc);
    s->enc_l = enc.left;
    s->enc_r = enc.right;
}

// 5 状态集中观测: x,y,theta,v (INS+编码器) + delta (上周期舵机指令, 跟踪良好假设)
static void car_state_observe(CarState *car, const SensorData *s, f32 delta_cmd) {
    f32 vl = s->enc_l * INV_ISR_DT;
    f32 vr = s->enc_r * INV_ISR_DT;
    ins_update_odom(car, vl, vr, ISR_DT);  // v, theta, x, y
    car->delta = delta_cmd;                // 舵机跟踪良好 → 真值=指令
}

//===================================================跟踪层 实现===================================================
// 200Hz: 虚拟车离线推进 → LQR 横向 → LADRC 纵向 + 差速 → 控制量

static void tracking_layer_step(CarState *vst, CarState *car, ActuatorCmd *cmd) {
    // 虚拟车推进 (NN ZOH 动作)
    f32 nn_a = planner_nn_a();
    f32 nn_o = planner_nn_omega();
    mcu_kinematics_step(vst, nn_a, nn_o);

    // LQR 横向跟踪: 真实 vs 虚拟
    f32 omega_cmd = tracker_step(car, vst, nn_o);
    f32 e_x       = tracker_get_ex();

    // 伺服: 前轮转角积分
    cmd->servo_delta = clamp(car->delta + omega_cmd * CTRL_DT, -DELTA_MAX, DELTA_MAX);

    // 纵向 LADRC + 阿克曼差速
    f32 thr_L, thr_R;
    longitudinal_step(&thr_L, &thr_R, car->v, vst->v, nn_a, e_x, car->delta);
    cmd->motor_l = thr_L;
    cmd->motor_r = thr_R;
}

//===================================================执行层 实现===================================================

static void actuators_apply(const ActuatorCmd *cmd) {
    hal_servo_set_delta(cmd->servo_delta);
    MotorPwm pwm = {
        .left  = (i32)(cmd->motor_l * (f32)PWM_DUTY_MAX),
        .right = (i32)(cmd->motor_r * (f32)PWM_DUTY_MAX),
    };
    hal_motor_set(&pwm);
}

//===================================================初始化===================================================

void tasks_init(void) {
    hal_servo_init();
    hal_motor_init();
    hal_encoder_init();
    g_imu = hal_imu_create(&imu_660rc_driver);
    ins_init();

    waypoint_mgr_init(g_waypoints, WP_COUNT);
    planner_init();
    tracker_init();

    // 虚拟状态从真实状态同步一次, 此后跟踪层自行推进
    memset(&g_vst, 0, sizeof(g_vst));
    g_vst = g_car;
    g_vst_prev[0] = g_car.x;
    g_vst_prev[1] = g_car.y;

    // 首帧 NN 动作 (确保 ISR 启动前 ZOH 有效)
    {
        Waypoint g1, g2, g3;
        if (waypoint_mgr_get_window(&g1, &g2, &g3)) {
            planner_forward(&g_vst, &g1, &g2, &g3);
        }
    }

    g_ready = 1;
}

//===================================================ISR 控制===================================================

// 1kHz: 传感器 + 5状态观测 (每次)
//       + 偏航融合 (每10次, 100Hz)
//       + 跟踪层→执行层 (每5次, 200Hz)
void car_control_update(void) {
    if (!g_ready) return;

    sensors_read(&g_sens);                                          // ── 传感器 (1kHz) ──
    car_state_observe(&g_car, &g_sens, g_cmd.servo_delta);         // ── 5状态观测 (1kHz) ──

    static u8 div100 = 0;                                           // 100Hz 分频
    if (++div100 >= 10) {
        div100 = 0;
        ins_update_yaw(g_imu);                                      // ── 偏航融合 (100Hz) ──
    }

    static u8 div200 = 0;                                           // 200Hz 分频
    if (++div200 >= 5) {
        div200 = 0;
        tracking_layer_step(&g_vst, &g_car, &g_cmd);               // ── 跟踪层 (200Hz) ──
        actuators_apply(&g_cmd);                                    // ── 执行层 (200Hz) ──
    }
}

//===================================================主循环任务===================================================

// 20Hz: 航点层 + 规划层
static void task_20hz_planner(void) {
    // ── 航点层: 到达检测 ──
    if (waypoint_mgr_check_hit(g_vst_prev, (f32[]){g_vst.x, g_vst.y})) {
        waypoint_mgr_mark_reached();
    }

    // ── 航点层: 获取 3 窗口 ──
    Waypoint g1, g2, g3;
    if (!waypoint_mgr_get_window(&g1, &g2, &g3)) return;

    // ── 规划层: 车体变换 + NN → ZOH ──
    planner_forward(&g_vst, &g1, &g2, &g3);

    // 记本周期起点 (供下次航点层线段检测)
    g_vst_prev[0] = g_vst.x;
    g_vst_prev[1] = g_vst.y;
}

static void task_10hz_debug(void) {
    u32 wp_done = waypoint_mgr_reached_count();
    f32 ey = tracker_get_ey();
    f32 omg = tracker_get_omega_cmd();
    (void)wp_done; (void)ey; (void)omg;
}

static void task_1hz_heartbeat(void) {
    // LED toggle, watchdog
}

//===================================================任务表===================================================
Task tasks[TASK_NUM] = {
    {task_20hz_planner,   50,   1, 0},     // 20Hz: 航点 + 规划
    {task_10hz_debug,    100,   1, 0},     // 10Hz: 遥测
    {task_1hz_heartbeat, 1000,  1, 0},     // 1Hz:  心跳
};
