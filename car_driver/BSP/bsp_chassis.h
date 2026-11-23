#ifndef BSP_CHASSIS_H
#define BSP_CHASSIS_H

#include "stm32f4xx.h"

/* 底盘型号（由 PB1 电位器电压分档识别，或用 APP_CHASSIS_FORCE 强制） */
#define CAR_MEC     0   /* 麦克纳姆轮 */
#define CAR_OMNI    1   /* 全向轮 */
#define CAR_AKM     2   /* 阿克曼 */
#define CAR_DIFF    3   /* 两轮差速 */
#define CAR_4WD     4   /* 四驱 */
#define CAR_TANK    5   /* 履带 */

/* 当前底盘的机械参数，运动学解算直接读这些全局量 */
extern float   g_wheel_spacing;      /* 轮距（m） */
extern float   g_axle_spacing;       /* 轴距（m） */
extern float   g_omni_radius;        /* 全向轮中心到车心距离（m） */
extern float   g_wheel_perimeter;    /* 轮子周长（m） */
extern float   g_encoder_precision;  /* 一圈的编码器脉冲数 */

/**
 * @brief  初始化 ADC 采样 PB1，识别底盘型号并载入对应机械参数。
 * @param  无
 * @retval 无
 * @note   APP_CHASSIS_FORCE >= 0 时直接强制型号，不再看电位器。
 */
void     BSP_Chassis_Init(void);

/**
 * @brief  读取上电时识别到的底盘型号（0..5，对应 CAR_xxx）。
 * @param  无
 * @retval 型号
 */
uint8_t  BSP_Chassis_GetMode(void);

/**
 * @brief  读取上电时采样的电位器原始值（调试用）。
 * @param  无
 * @retval 0..4095
 */
uint16_t BSP_Chassis_GetPot(void);

/**
 * @brief  立刻重新采样一次电位器（不改变已识别的型号）。
 * @param  无
 * @retval 0..4095
 */
uint16_t BSP_Chassis_ReadPotLive(void);

/**
 * @brief  判断当前底盘是否全向类（麦克纳姆或全向轮），可横移。
 * @param  无
 * @retval 1 = 可以横移，0 = 不可以
 */
uint8_t  BSP_Chassis_IsOmni(void);

/**
 * @brief  判断当前底盘是否阿克曼（需要舵机转向）。
 * @param  无
 * @retval 1 = 阿克曼
 */
uint8_t  BSP_Chassis_IsAckermann(void);

/**
 * @brief  连续采样 10 次求平均，返回 PB1 的 ADC 值。
 * @param  无
 * @retval 0..4095
 * @note   阻塞式采样，不要放在高频任务里。
 */
uint16_t BSP_Chassis_ReadPot(void);

#endif /* BSP_CHASSIS_H */
