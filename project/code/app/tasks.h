#pragma once
#include "common.h"

#define TASK_NUM  4

typedef struct {
    void (*run)(void);
    volatile u32 period, countdown, pending;
} task_t;

extern task_t tasks[TASK_NUM];

void wp_clear(void);
void wp_push(const waypoint_t *wp);
void wp_push_n(const waypoint_t *wps, u8 n);
u8   wp_count(void);
waypoint_t wp_peek(u8 off);
void wp_pop(void);

extern car_state_t      g_car, g_vst;
extern planner_action_t g_plan;
extern waypoint_t      g_vst_prev;
extern bool          g_wp_active;
extern f32           g_ey, g_ex;
extern volatile u32  g_ms;

void tasks_init(void);
void car_control_update(void);
void task_20hz_planner(void);
void task_20hz_report(void);
