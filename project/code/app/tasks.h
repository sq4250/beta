/**
 * tasks.h — 任务表 + ISR 控制接口
 */

#ifndef TASKS_H
#define TASKS_H

#include "common.h"

// 任务数: planner(20Hz) + report(20Hz) + debug(10Hz) + heartbeat(1Hz)
#define TASK_NUM    4

typedef struct {
    void (*run)(void);
    volatile u32 period;
    volatile u32 countdown;
    volatile u32 pending;
} Task;

extern Task tasks[TASK_NUM];

void tasks_init(void);
void car_control_update(void);         // 1kHz ISR: 传感器 + 5状态观测 + 跟踪+执行(200Hz)

#endif
