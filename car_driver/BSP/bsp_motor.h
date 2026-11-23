#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "stm32f4xx.h"

/*
 * 四路直流电机 PWM（WHEELTEC C30D 接线）：
 *   A: TIM10_CH1 (PB8)  + TIM11_CH1 (PB9)
 *   B: TIM9_CH1  (PE5)  + TIM9_CH2  (PE6)
 *   C: TIM1_CH1  (PE9)  + TIM1_CH2  (PE11)
 *   D: TIM1_CH3  (PE13) + TIM1_CH4  (PE14)
 *
 * 每路两个通道：一个给固定 APP_PWM_MAX，另一个给 APP_PWM_MAX - |pwm|，
 * 用"两个半桥的占空比差"控制方向和大小（和 WHEELTEC Set_Pwm 一致）。
 */

/**
 * @brief  初始化四路电机的 GPIO 复用与 PWM 定时器，并把输出置为停止。
 * @param  无
 * @retval 无
 * @note   调用后电机为停止状态（两通道都写 APP_PWM_MAX，占空比为 0）。
 */
void BSP_Motor_Init(void);

/**
 * @brief  设置四路电机的 PWM 值（带符号）。
 * @param  motor_a A 路 PWM，范围 -APP_PWM_MAX .. +APP_PWM_MAX
 * @param  motor_b B 路 PWM
 * @param  motor_c C 路 PWM
 * @param  motor_d D 路 PWM
 * @retval 无
 * @note   正值一个方向、负值另一个方向；调用者负责限幅。
 */
void BSP_Motor_SetPwm(int motor_a, int motor_b, int motor_c, int motor_d);

#endif /* BSP_MOTOR_H */
