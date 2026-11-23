#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "stm32f4xx.h"

/*
 * 四路正交编码器（定时器编码器模式，4 倍频）：
 *   A: TIM2_CH1/CH2 = PA15 / PB3
 *   B: TIM3_CH1/CH2 = PB4  / PB5
 *   C: TIM4_CH1/CH2 = PB6  / PB7
 *   D: TIM5_CH1/CH2 = PA0  / PA1
 */

/**
 * @brief  初始化四路编码器接口，计数器清零。
 * @param  无
 * @retval 无
 * @note   编码器模式 3（TI1、TI2 双边沿都计数），得到 4 倍频脉冲。
 */
void BSP_Encoder_Init(void);

/**
 * @brief  读取四路编码器的增量并清零计数器（读后即清）。
 * @param  enc_a 输出：A 路本次增量（有符号 16 位）
 * @param  enc_b 输出：B 路本次增量
 * @param  enc_c 输出：C 路本次增量
 * @param  enc_d 输出：D 路本次增量
 * @retval 无
 * @note   必须按固定周期（控制周期）调用，增量才有速度意义。
 */
void BSP_Encoder_ReadAll(int16_t *enc_a, int16_t *enc_b, int16_t *enc_c, int16_t *enc_d);

#endif /* BSP_ENCODER_H */
