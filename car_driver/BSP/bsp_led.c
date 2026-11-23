/***********************************************
 * car_driver : 用户 LED 驱动（心跳灯）
 * Pin : PE8
 * 极性: APP_LED_ACTIVE_HIGH = 0 时低电平点亮（C30D 默认接线）
 ***********************************************/

#include "bsp_led.h"
#include "app_config.h"

#define LED_PORT   GPIOE
#define LED_PIN    GPIO_Pin_8
#define LED_CLK    RCC_AHB1Periph_GPIOE

/**
 * @brief  初始化 PE8 为推挽输出，初始熄灭。
 * @param  无
 * @retval 无
 * @note   上拉输出，避免复位瞬间被外部拉低而闪一下。
 */
void BSP_LED_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(LED_CLK, ENABLE);

    gpio.GPIO_Pin   = LED_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(LED_PORT, &gpio);

    BSP_LED_Write(0U);          /* 先灭灯 */
}

/**
 * @brief  写引脚电平（物理电平，不做极性换算）。
 * @param  level 1 = 高电平，0 = 低电平
 * @retval 无
 */
static void BSP_LED_WriteRaw(uint8_t level)
{
    if (level != 0U)
    {
        GPIO_SetBits(LED_PORT, LED_PIN);
    }
    else
    {
        GPIO_ResetBits(LED_PORT, LED_PIN);
    }
}

/**
 * @brief  按逻辑值设置 LED，极性在这里统一处理。
 * @param  on 0 = 灭，非 0 = 亮
 * @retval 无
 */
void BSP_LED_Write(uint8_t on)
{
#if (APP_LED_ACTIVE_HIGH != 0)
    BSP_LED_WriteRaw((on != 0U) ? 1U : 0U);
#else
    BSP_LED_WriteRaw((on != 0U) ? 0U : 1U);
#endif
}

/**
 * @brief  翻转 LED 输出电平。
 * @param  无
 * @retval 无
 * @note   直接读 ODR，不依赖任何软件状态，所以和 BSP_LED_Write 混用安全。
 */
void BSP_LED_Toggle(void)
{
    if (GPIO_ReadOutputDataBit(LED_PORT, LED_PIN) != Bit_RESET)
    {
        GPIO_ResetBits(LED_PORT, LED_PIN);
    }
    else
    {
        GPIO_SetBits(LED_PORT, LED_PIN);
    }
}
