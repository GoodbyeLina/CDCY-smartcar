/***********************************************
 * car_driver : 转向舵机驱动（阿克曼底盘用）
 * PB15 / TIM12_CH2  : 50 Hz，脉宽 800~2200 us
 * PC9  / TIM8_CH4   : 老版主板，APP_SERVO_DUAL_OUTPUT 控制是否输出
 ***********************************************/

#include "bsp_servo.h"
#include "app_config.h"

/**
 * @brief  初始化 TIM12_CH2 / PB15，周期 20 ms（50 Hz），初始脉宽居中。
 * @param  无
 * @retval 无
 * @note   TIM12 挂在 APB1（84 MHz），预分频 84-1 得到 1 MHz 计数，
 *         周期 20000 即 20 ms；CCR 直接写微秒数。
 */
void BSP_Servo_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period        = 20000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 84 - 1;   /* 84 MHz -> 1 MHz */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = APP_SERVO_CENTER_US;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC2Init(TIM12, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM12, ENABLE);
    TIM_Cmd(TIM12, ENABLE);

#if (APP_SERVO_DUAL_OUTPUT != 0)
    /* 老版 C30D：转向信号同时接在 PC9 / TIM8_CH4 上 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period        = 20000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 168 - 1;  /* 168 MHz -> 1 MHz */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = APP_SERVO_CENTER_US;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
#endif
}

/**
 * @brief  设置舵机脉宽，超范围自动夹紧。
 * @param  pulse_us 脉宽（us）
 * @retval 无
 * @note   CCR 预装载使能，新值在下一个周期生效，不会产生毛刺。
 */
void BSP_Servo_Set(uint16_t pulse_us)
{
    if (pulse_us < APP_SERVO_MIN_US)
    {
        pulse_us = APP_SERVO_MIN_US;
    }
    if (pulse_us > APP_SERVO_MAX_US)
    {
        pulse_us = APP_SERVO_MAX_US;
    }

    TIM12->CCR2 = pulse_us;
#if (APP_SERVO_DUAL_OUTPUT != 0)
    TIM8->CCR4 = pulse_us;
#endif
}
