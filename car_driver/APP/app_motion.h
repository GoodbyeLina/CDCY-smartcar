#ifndef APP_MOTION_H
#define APP_MOTION_H

#include "stm32f4xx.h"

/*
 * 运动控制层：速度指令 -> 运动学解算 -> 四轮目标速度 -> PI -> PWM。
 *
 * 数据流（Control 任务里每 10 ms 调一次 App_Motion_ControlStep）：
 *   App_Motion_Apply(vx, vy, wz)   上层下发的车体速度指令（mm/s、mrad/s）
 *     -> Motion_DriveMotor         按底盘型号解算成四个轮子目标速度
 *       -> Motion_ReadEncoders     读编码器增量换算实测轮速
 *         -> Motion_PiUpdate       每轮一个 PI，输出 PWM
 *           -> BSP_Motor_SetPwm    写定时器
 *
 * 安全：PD3 使能关闭 / 未 ARM / 急停时，输出强制为 0。
 */

/**
 * @brief  初始化运动控制状态：速度清零、PI 清零、电机停转。
 * @param  无
 * @retval 无
 * @note   必须在 BSP_Chassis_Init() 之后调用（要用到底盘参数）。
 */
void    App_Motion_Init(void);

/**
 * @brief  控制步进函数：解算 + PI + 输出 PWM，固定周期调用（10 ms）。
 * @param  无
 * @retval 无
 * @note   由 App_Control_Task 以 100 Hz 调用，是唯一写电机的入口。
 */
void    App_Motion_ControlStep(void);

/**
 * @brief  下发车体速度指令。
 * @param  vx_mm_s   前后速度（mm/s），正 = 前进
 * @param  vy_mm_s   左右速度（mm/s），正 = 左移（仅全向底盘有效）
 * @param  wz_mrad_s 旋转角速度（mrad/s），正 = 逆时针
 * @retval 无
 * @note   内部会限幅到 APP_MAX_*；同时清除 raw PWM 调试模式。
 */
void    App_Motion_Apply(int16_t vx_mm_s, int16_t vy_mm_s, int16_t wz_mrad_s);

/**
 * @brief  停止：速度指令清零（PI 会自然泄放）。
 * @param  无
 * @retval 无
 */
void    App_Motion_Stop(void);

/**
 * @brief  急停：解除 ARM，并清零速度指令。
 * @param  无
 * @retval 无
 * @note   急停后必须再收到 0x00 ARM 才会重新动作。
 */
void    App_Motion_EStop(void);

/**
 * @brief  重新 ARM，并清掉故障位。
 * @param  无
 * @retval 无
 */
void    App_Motion_Arm(void);

/**
 * @brief  调试模式：绕过 PI，四路电机直接吃同一个 PWM。
 * @param  pwm PWM 值（会被限幅到 ±APP_PWM_LIMIT）
 * @retval 无
 * @note   任何正常速度指令都会退出这个模式。
 */
void    App_Motion_SetRaw(int16_t pwm);

/**
 * @brief  读取当前车体速度指令。
 * @param  vx_mm_s   输出：前后速度
 * @param  vy_mm_s   输出：左右速度
 * @param  wz_mrad_s 输出：角速度
 * @retval 无
 */
void    App_Motion_Get(int16_t *vx_mm_s, int16_t *vy_mm_s, int16_t *wz_mrad_s);

/**
 * @brief  调试快照：每轮实测速度、目标速度、实际输出 PWM。
 * @param  enc_mm_s    输出：4 轮实测速度（mm/s，已 ×1000）
 * @param  target_mm_s 输出：4 轮目标速度（mm/s，已 ×1000）
 * @param  pwm         输出：4 轮 PI 输出（raw 模式下是同一个 raw 值）
 * @retval 无
 */
void    App_Motion_GetDebug(int16_t enc_mm_s[4], int16_t target_mm_s[4], int16_t pwm[4]);

/**
 * @brief  是否已 ARM。
 * @param  无
 * @retval 1 = 允许输出
 */
uint8_t App_Motion_IsArmed(void);

/**
 * @brief  当前是否有非零速度指令。
 * @param  无
 * @retval 1 = 在动
 */
uint8_t App_Motion_IsMoving(void);

/**
 * @brief  置位故障标志（可多个位或在一起）。
 * @param  fault APP_FAULT_xxx
 * @retval 无
 */
void    App_Motion_SetFault(uint8_t fault);

/**
 * @brief  清除全部故障标志。
 * @param  无
 * @retval 无
 */
void    App_Motion_ClearFault(void);

/**
 * @brief  清除单个故障位。
 * @param  fault APP_FAULT_xxx
 * @retval 无
 */
void    App_Motion_ClearFaultBit(uint8_t fault);

/**
 * @brief  读取故障标志。
 * @param  无
 * @retval APP_FAULT_xxx 按位或
 */
uint8_t App_Motion_GetFault(void);

#endif /* APP_MOTION_H */
