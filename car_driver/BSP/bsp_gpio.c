/***********************************************
 * car_driver : 板级开关输入
 * PD3 : 硬件使能开关（上拉输入，低电平 = 关闭）
 ***********************************************/

#include "bsp_gpio.h"

/**
 * @brief  初始化 PD3 为上拉输入。
 * @param  无
 * @retval 无
 * @note   只配置输入，不驱动任何输出；LED 在 bsp_led.c 里单独初始化。
 */
void BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/**
 * @brief  读取使能开关状态。
 * @param  无
 * @retval 1 = 使能（高电平），0 = 关闭（低电平）
 */
uint8_t BSP_Enable_IsOn(void)
{
    return (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == Bit_SET) ? 1U : 0U;
}
