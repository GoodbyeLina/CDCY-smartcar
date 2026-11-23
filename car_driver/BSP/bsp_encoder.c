/***********************************************
 * car_driver : 四路正交编码器
 * A: TIM2 (PA15/PB3)   B: TIM3 (PB4/PB5)
 * C: TIM4 (PB6/PB7)    D: TIM5 (PA0/PA1)
 *
 * 定时器工作在编码器模式 3：TI1、TI2 双边沿都计数，即 4 倍频。
 * 控制周期里读一次 CNT 并清零，差值就是这一周期的脉冲数。
 ***********************************************/

#include "bsp_encoder.h"

#define ENCODER_TIM_PERIOD  65535U      /* 16 位定时器满量程，靠"读后清零"取增量 */

/**
 * @brief  把两个引脚配成复用功能，供编码器定时器使用。
 * @param  port 端口（GPIOA/GPIOB）
 * @param  pins 引脚掩码
 * @retval 无
 */
static void Encoder_GpioInit(GPIO_TypeDef *port, uint16_t pins)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = pins;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(port, &GPIO_InitStructure);
}

/**
 * @brief  把一个定时器配成编码器模式。
 * @param  tim 定时器（TIM2/3/4/5）
 * @retval 无
 * @note   预分频 0、周期 65535，即只做脉冲计数，不做定时。
 */
static void Encoder_TimerInit(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef       TIM_ICInitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_Period        = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);

    /* 编码器模式 3：TI1 和 TI2 的上升沿、下降沿都计数（4 倍频） */
    TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInit(tim, &TIM_ICInitStructure);

    TIM_SetCounter(tim, 0);
    TIM_Cmd(tim, ENABLE);
}

/**
 * @brief  初始化四路编码器（时钟、复用、编码器模式）。
 * @param  无
 * @retval 无
 */
void BSP_Encoder_Init(void)
{
    /* Encoder A: TIM2_CH1/CH2 = PA15 / PB3 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,  GPIO_AF_TIM2);
    Encoder_GpioInit(GPIOA, GPIO_Pin_15);
    Encoder_GpioInit(GPIOB, GPIO_Pin_3);
    Encoder_TimerInit(TIM2);

    /* Encoder B: TIM3_CH1/CH2 = PB4 / PB5 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
    Encoder_GpioInit(GPIOB, GPIO_Pin_4 | GPIO_Pin_5);
    Encoder_TimerInit(TIM3);

    /* Encoder C: TIM4_CH1/CH2 = PB6 / PB7 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    Encoder_GpioInit(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
    Encoder_TimerInit(TIM4);

    /* Encoder D: TIM5_CH1/CH2 = PA0 / PA1 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    Encoder_GpioInit(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
    Encoder_TimerInit(TIM5);
}

/**
 * @brief  读取四路编码器增量并清零计数器。
 * @param  enc_a A 路增量
 * @param  enc_b B 路增量
 * @param  enc_c C 路增量
 * @param  enc_d D 路增量
 * @retval 无
 * @note   先强转 int16_t 再清零：计数器是 16 位，反转时自然得到负数。
 */
void BSP_Encoder_ReadAll(int16_t *enc_a, int16_t *enc_b, int16_t *enc_c, int16_t *enc_d)
{
    *enc_a = (int16_t)TIM2->CNT;
    *enc_b = (int16_t)TIM3->CNT;
    *enc_c = (int16_t)TIM4->CNT;
    *enc_d = (int16_t)TIM5->CNT;

    TIM2->CNT = 0;
    TIM3->CNT = 0;
    TIM4->CNT = 0;
    TIM5->CNT = 0;
}
