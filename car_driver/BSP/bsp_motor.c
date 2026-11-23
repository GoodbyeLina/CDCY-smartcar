/***********************************************
 * car_driver : 四路电机 PWM 驱动
 * A: TIM10_CH1 (PB8)  + TIM11_CH1 (PB9)
 * B: TIM9_CH1  (PE5)  + TIM9_CH2  (PE6)
 * C: TIM1_CH1  (PE9)  + TIM1_CH2  (PE11)
 * D: TIM1_CH3  (PE13) + TIM1_CH4  (PE14)
 *
 * 每路两个半桥：一个固定满占空比，另一个按 |pwm| 反相，
 * 两者之差就是电机两端的有效电压。和原工程 bsp_motor.c 完全一致。
 ***********************************************/

#include "bsp_motor.h"
#include "app_config.h"

/* 每个半桥的 CCR，宏比结构体访问快，也便于下面的正负分支对照 */
#define PWMA1   (TIM10->CCR1)
#define PWMA2   (TIM11->CCR1)
#define PWMB1   (TIM9->CCR1)
#define PWMB2   (TIM9->CCR2)
#define PWMC1   (TIM1->CCR1)
#define PWMC2   (TIM1->CCR2)
#define PWMD1   (TIM1->CCR3)
#define PWMD2   (TIM1->CCR4)

/**
 * @brief  初始化 TIM10_CH1 / PB8 为 PWM 输出。
 * @param  无
 * @retval 无
 * @note   周期固定为 APP_PWM_MAX，预分频 0，即 168 MHz / 16800 = 10 kHz。
 */
static void Motor_PwmInit_TIM10(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period        = APP_PWM_MAX - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = APP_PWM_MAX;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM10, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM10, ENABLE);
    TIM_Cmd(TIM10, ENABLE);
}

/**
 * @brief  初始化 TIM11_CH1 / PB9 为 PWM 输出。
 * @param  无
 * @retval 无
 */
static void Motor_PwmInit_TIM11(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period        = APP_PWM_MAX - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = APP_PWM_MAX;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM11, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM11, ENABLE);
    TIM_Cmd(TIM11, ENABLE);
}

/**
 * @brief  初始化 TIM9_CH1/CH2（PE5/PE6）为两路 PWM 输出，对应 B 路电机。
 * @param  无
 * @retval 无
 */
static void Motor_PwmInit_TIM9(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period        = APP_PWM_MAX - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = APP_PWM_MAX;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM9, ENABLE);
    TIM_Cmd(TIM9, ENABLE);
}

/**
 * @brief  初始化 TIM1 四个通道（PE9/PE11/PE13/PE14）为 PWM 输出，对应 C、D 路。
 * @param  无
 * @retval 无
 * @note   TIM1 是高级定时器，必须额外调 TIM_CtrlPWMOutputs 打开主输出。
 */
static void Motor_PwmInit_TIM1(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period        = APP_PWM_MAX - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = APP_PWM_MAX;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/**
 * @brief  初始化四路电机 PWM，并让所有通道停在"零占空比"。
 * @param  无
 * @retval 无
 */
void BSP_Motor_Init(void)
{
    Motor_PwmInit_TIM10();
    Motor_PwmInit_TIM11();
    Motor_PwmInit_TIM9();
    Motor_PwmInit_TIM1();

    PWMA1 = APP_PWM_MAX; PWMA2 = APP_PWM_MAX;
    PWMB1 = APP_PWM_MAX; PWMB2 = APP_PWM_MAX;
    PWMC1 = APP_PWM_MAX; PWMC2 = APP_PWM_MAX;
    PWMD1 = APP_PWM_MAX; PWMD2 = APP_PWM_MAX;
}

/**
 * @brief  设置四路电机的带符号 PWM。
 * @param  motor_a A 路 PWM，正负决定方向
 * @param  motor_b B 路 PWM
 * @param  motor_c C 路 PWM
 * @param  motor_d D 路 PWM
 * @retval 无
 * @note   以 A 为例：负值时 A2 固定满占空比、A1 = MAX + pwm（反向）；
 *         正值时 A1 固定、A2 = MAX - pwm（正向）。其余三路同理，
 *         只是通道顺序和原厂 C30D 一致，别随手调换，否则轮子反转。
 */
void BSP_Motor_SetPwm(int motor_a, int motor_b, int motor_c, int motor_d)
{
    if (motor_a < 0)
    {
        PWMA2 = APP_PWM_MAX;
        PWMA1 = APP_PWM_MAX + motor_a;
    }
    else
    {
        PWMA1 = APP_PWM_MAX;
        PWMA2 = APP_PWM_MAX - motor_a;
    }

    if (motor_b < 0)
    {
        PWMB1 = APP_PWM_MAX;
        PWMB2 = APP_PWM_MAX + motor_b;
    }
    else
    {
        PWMB2 = APP_PWM_MAX;
        PWMB1 = APP_PWM_MAX - motor_b;
    }

    if (motor_c < 0)
    {
        PWMC1 = APP_PWM_MAX;
        PWMC2 = APP_PWM_MAX + motor_c;
    }
    else
    {
        PWMC2 = APP_PWM_MAX;
        PWMC1 = APP_PWM_MAX - motor_c;
    }

    if (motor_d < 0)
    {
        PWMD2 = APP_PWM_MAX;
        PWMD1 = APP_PWM_MAX + motor_d;
    }
    else
    {
        PWMD1 = APP_PWM_MAX;
        PWMD2 = APP_PWM_MAX - motor_d;
    }
}
