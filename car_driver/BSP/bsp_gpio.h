#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "stm32f4xx.h"

/*
 * 板级开关输入（LED 单独在 bsp_led.c，这里只放使能开关）：
 *   PD3 硬件使能开关，上拉输入，低电平 = 关闭
 *
 * 关闭时电机控制任务每个周期都会强制输出 0，属于最后一道安全闸。
 */

/**
 * @brief  把 PD3 配置为上拉输入。
 * @param  无
 * @retval 无
 * @note   上拉保证悬空时读到高电平（误判为"使能"），
 *         真正的使能判断还要看硬件开关是否拉低。
 */
void BSP_GPIO_Init(void);

/**
 * @brief  读取硬件使能开关状态。
 * @param  无
 * @retval 1 = 使能（PD3 高），0 = 关闭（PD3 低）
 */
uint8_t BSP_Enable_IsOn(void);

#endif /* BSP_GPIO_H */
