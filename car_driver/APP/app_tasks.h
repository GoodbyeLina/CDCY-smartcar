#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "stm32f4xx.h"

/*
 * 周期任务，只放"按固定节拍做事"的循环：
 *   App_Control_Task  100 Hz，跑 App_Motion_ControlStep()
 *   App_Link_Task     10 Hz，做指令超时保护 + 回查询帧
 *
 * LED 心跳和串口心跳不在这里，在 app_debug.c 的 App_Debug_Task 里。
 */

/**
 * @brief  控制任务：每 APP_CONTROL_PERIOD_MS 调一次运动控制步进。
 * @param  pv_parameters 未使用
 * @retval 无
 * @note   优先级最高（APP_TASK_PRIO_CONTROL），周期用 vTaskDelayUntil 保证恒定，
 *         周期抖动会直接变成轮速误差。
 */
void App_Control_Task(void *pv_parameters);

/**
 * @brief  链路任务：指令超时停车，并把挂起的查询回复发出去。
 * @param  pv_parameters 未使用
 * @retval 无
 * @note   100 ms 一轮。超时门限 APP_CMD_TIMEOUT_MS = 0 时关闭超时保护。
 */
void App_Link_Task(void *pv_parameters);

#endif /* APP_TASKS_H */
