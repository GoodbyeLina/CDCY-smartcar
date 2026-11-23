#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include <stdint.h>

/*
 * 调试输出：开机横幅（ASCII）+ 周期性二进制帧（走 USART3，与协议同口）。
 *
 *   App_Debug_Banner()     上电横幅，main() 里、调度器启动前调用一次
 *   App_Debug_Task()       心跳任务：500 ms 翻 LED，每 APP_DEBUG_PERIOD_MS 发一周期帧
 *   App_Debug_Heartbeat()  按 0x0F 的位图发 0x20（16B）/ 0x21（34B）/ 0x22（6B）
 *
 * 三帧都是纯数据，字段偏移见 car_protocol.h 的注释，全部小端。
 * 不再输出 "[HB] up=..." 这类文本行。
 */

/**
 * @brief  上电横幅：版本、编译时间、串口参数、帧格式、堆余量。
 * @param  无
 * @retval 无
 * @note   必须在 BSP_UART3_Init() 之后调用；调度器未启动时也能发（不加锁）。
 */
void App_Debug_Banner(void);

/**
 * @brief  按遥测开关发送 0x20 心跳帧、0x21 电机帧、0x22 驱动帧。
 * @param  无
 * @retval 无
 * @note   发哪几帧由 0x0F 命令保存的位图决定（CAR_TLM_BIT_xxx），上电默认全开；
 *         帧内容见 car_protocol.h；APP_DEBUG_ENABLE = 0 时整个函数被编译掉。
 */
void App_Debug_Heartbeat(void);

/**
 * @brief  调试/心跳任务主体：500 ms 周期翻 LED，按周期发一周期帧。
 * @param  pv_parameters 未使用
 * @retval 无
 * @note   优先级 APP_TASK_PRIO_DEBUG，栈 APP_TASK_STACK_DEBUG。
 */
void App_Debug_Task(void *pv_parameters);

#endif /* APP_DEBUG_H */
