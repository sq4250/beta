#include "zf_common_headfile.h"
#include "scheduler.h"
#include "tasks.h"
#include "hal_tick.h"

static void scheduler_update(void) {
    for (u8 i = 0; i < TASK_NUM; i++) {
        if (tasks[i].countdown > 0) tasks[i].countdown--;
        if (tasks[i].countdown == 0) {
            if (tasks[i].pending < 100) tasks[i].pending++;
            tasks[i].countdown = tasks[i].period;
        }
    }
}

void scheduler_init(void) {
    hal_tick_init(scheduler_update);
    tasks_init();
}

void scheduler_run(void) {
    for (u8 i = 0; i < TASK_NUM; i++) {
        while (tasks[i].pending > 0 && tasks[i].run) {
            __disable_irq();
            tasks[i].pending--;
            __enable_irq();
            tasks[i].run();
        }
    }
}
