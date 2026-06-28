/**
 * scheduler.c - 时间片任务调度器实现
 *
 * 原理:
 *   1. ISR 每 1ms 调用 scheduler_update() → 递减 countdown
 *   2. countdown==0 → pending++ 并重置 countdown=period
 *   3. 主循环 scheduler_run() 消费 pending → 调用 task->run()
 *
 * 优点: 无锁、无动态内存、ISR 只做整数运算
 */

#include "zf_common_headfile.h"
#include "scheduler.h"
#include "tasks.h"
#include "hal_tick.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     调度器 tick 更新 (ISR 上下文)
// 参数说明     void
// 返回参数     void
// 使用示例     由 hal_tick 回调自动调用
// 备注信息     内部函数，用户无需关心。遍历任务表递减 countdown，到期则 pending++
//-------------------------------------------------------------------------------------------------------------------
static void scheduler_update(void) {
    u8 i;
    for (i = 0; i < TASK_NUM; ++i) {
        if (tasks[i].countdown > 0) {
            tasks[i].countdown--;
        }
        if (tasks[i].countdown == 0) {
            if (tasks[i].pending < 100) {                                       // 防溢出：主循环阻塞时不会无限堆积
                tasks[i].pending++;
            }
            tasks[i].countdown = tasks[i].period;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     调度器初始化
// 参数说明     void
// 返回参数     void
// 使用示例     scheduler_init();
// 备注信息     初始化 tick 硬件和任务表
//-------------------------------------------------------------------------------------------------------------------
void scheduler_init(void) {
    hal_tick_init(scheduler_update);                                            // 注册 scheduler_update 为 tick 回调
    tasks_init();                                                               // 用户任务初始化
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     调度器运行 (主循环调用)
// 参数说明     void
// 返回参数     void
// 使用示例     while(1) { scheduler_run(); }
// 备注信息     按优先级顺序 (数组顺序) 消费 pending，当前任务跑完再跑下一个
//-------------------------------------------------------------------------------------------------------------------
void scheduler_run(void) {
    u8 i;
    for (i = 0; i < TASK_NUM; ++i) {
        while (tasks[i].pending > 0 && tasks[i].run != NULL) {
            tasks[i].pending--;
            tasks[i].run();
        }
    }
}
