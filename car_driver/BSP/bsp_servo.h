#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include "stm32f4xx.h"

/*
 * 阿克曼底盘的转向舵机（50 Hz PWM，脉宽 800~2200 us）：
 *   PB15 / TIM12_CH2   V2.x 主板
 *   PC9  / TIM8_CH4    老版 C30D，由 APP_SERVO_DUAL_OUTPUT 控制是否同时输出
 */

/**
 * @brief  初始化转向舵机 PWM，并把脉宽置中（APP_SERVO_CENTER_US）。
 * @param  无
 * @retval 无
 * @note   只有底盘识别为 CAR_AKM 时才需要调用。
 */
void BSP_Servo_Init(void);

/**
 * @brief  设置舵机脉宽。
 * @param  pulse_us 目标脉宽，单位 us；超出 APP_SERVO_MIN_US..APP_SERVO_MAX_US 会被夹紧
 * @retval 无
 */
void BSP_Servo_Set(uint16_t pulse_us);

#endif /* BSP_SERVO_H */
