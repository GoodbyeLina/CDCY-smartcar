#ifndef BSP_LED_H
#define BSP_LED_H

#include "stm32f4xx.h"

/* 用户 LED：PE8，低电平点亮（APP_LED_ACTIVE_HIGH 控制逻辑极性）。
 * 心跳灯：每 500 ms 翻转一次，说明调度器和空闲任务还活着。 */

/**
 * @brief  初始化 PE8 为用户 LED 输出，并先置为熄灭。
 * @param  无
 * @retval 无
 * @note   在 main() 里、创建任务之前调用即可。
 */
void BSP_LED_Init(void);

/**
 * @brief  按逻辑值设置 LED 亮灭。
 * @param  on 0 = 灭，非 0 = 亮（内部按 APP_LED_ACTIVE_HIGH 换算电平）
 * @retval 无
 */
void BSP_LED_Write(uint8_t on);

/**
 * @brief  翻转 LED 当前状态。
 * @param  无
 * @retval 无
 * @note   读的是输出寄存器 ODR，所以和 BSP_LED_Write 混用也不会错乱。
 */
void BSP_LED_Toggle(void);

#endif /* BSP_LED_H */
