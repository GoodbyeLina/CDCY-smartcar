/***********************************************
 * car_driver : 运动控制（100 Hz PI 速度环）
 * 指令 -> 运动学解算 -> 四轮目标 -> 编码器实测 -> PI -> PWM
 ***********************************************/

#include "app_motion.h"
#include "app_config.h"
#include "bsp_chassis.h"
#include "bsp_encoder.h"
#include "bsp_gpio.h"
#include "bsp_motor.h"
#include "bsp_servo.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

typedef struct
{
    float Target;       /* 目标轮速（m/s） */
    float Motor_Pwm;    /* PI 输出（PWM 值） */
} Motion_Motor_t;

typedef struct
{
    float VX;
    float VY;
    float VZ;
} Motion_Smooth_t;

static volatile int16_t s_vx_mm_s;
static volatile int16_t s_vy_mm_s;
static volatile int16_t s_wz_mrad_s;
static volatile uint8_t s_armed;
static volatile uint8_t s_fault;
static volatile uint8_t s_raw_active;   /* 1 = 正在用 raw PWM 直驱 */
static volatile int16_t s_raw_pwm;
static volatile uint8_t s_cmd_seen;     /* 是否收到过正常速度指令 */

static Motion_Motor_t  s_motor[4];
static Motion_Smooth_t s_smooth;
static float           s_enc[4];        /* 实测轮速（m/s） */
static float           s_pi_pwm[4];
static float           s_pi_last[4];
static float           s_servo_us;

#if (APP_STARTUP_FORWARD_ENABLE != 0)
static uint8_t    s_startup_done;
static uint8_t    s_startup_started;
static TickType_t s_startup_start_tick;
#endif

/**
 * @brief  浮点限幅。
 * @param  value 输入
 * @param  low   下限
 * @param  high  上限
 * @retval 夹紧后的值
 */
static float Motion_ClampFloat(float value, float low, float high)
{
    if (value > high)
    {
        return high;
    }
    if (value < low)
    {
        return low;
    }
    return value;
}

/**
 * @brief  整数限幅。
 * @param  value 输入
 * @param  low   下限
 * @param  high  上限
 * @retval 夹紧后的值
 */
static int16_t Motion_ClampInt16(int32_t value, int32_t low, int32_t high)
{
    if (value > high)
    {
        value = high;
    }
    if (value < low)
    {
        value = low;
    }
    return (int16_t)value;
}

/**
 * @brief  清除 PI 状态和输出，防止下次启动带着残留积分冲出去。
 * @param  无
 * @retval 无
 */
static void Motion_ClearPi(void)
{
    uint8_t i;

    for (i = 0; i < 4U; i++)
    {
        s_pi_pwm[i]  = 0.0f;
        s_pi_last[i] = 0.0f;
        s_motor[i].Motor_Pwm = 0.0f;
    }
}

/**
 * @brief  目标速度清零、PI 清零、电机输出 0，并把舵机回中。
 * @param  无
 * @retval 无
 */
static void Motion_OutputZero(void)
{
    uint8_t i;

    for (i = 0; i < 4U; i++)
    {
        s_motor[i].Target = 0.0f;
    }
    Motion_ClearPi();
    BSP_Motor_SetPwm(0, 0, 0, 0);

    if (BSP_Chassis_IsAckermann())
    {
        s_servo_us = (float)APP_SERVO_CENTER_US;
        BSP_Servo_Set(APP_SERVO_CENTER_US);
    }
}

/**
 * @brief  初始化运动状态：速度 0、ARM、无故障、电机停转。
 * @param  无
 * @retval 无
 */
void App_Motion_Init(void)
{
    s_vx_mm_s   = 0;
    s_vy_mm_s   = 0;
    s_wz_mrad_s = 0;
    s_armed     = 1U;
    s_fault     = APP_FAULT_NONE;

    s_smooth.VX = 0.0f;
    s_smooth.VY = 0.0f;
    s_smooth.VZ = 0.0f;

    s_servo_us = (float)APP_SERVO_CENTER_US;
    Motion_OutputZero();
}

/**
 * @brief  下发车体速度指令（限幅后写共享变量）。
 * @param  vx_mm_s   前后速度
 * @param  vy_mm_s   左右速度
 * @param  wz_mrad_s 角速度
 * @retval 无
 * @note   临界区保护：控制任务可能随时读这几个变量。
 */
void App_Motion_Apply(int16_t vx_mm_s, int16_t vy_mm_s, int16_t wz_mrad_s)
{
    vx_mm_s   = Motion_ClampInt16(vx_mm_s,   -APP_MAX_VX_MM_S, APP_MAX_VX_MM_S);
    vy_mm_s   = Motion_ClampInt16(vy_mm_s,   -APP_MAX_VY_MM_S, APP_MAX_VY_MM_S);
    wz_mrad_s = Motion_ClampInt16(wz_mrad_s, -APP_MAX_WZ_MRAD_S, APP_MAX_WZ_MRAD_S);

    taskENTER_CRITICAL();
    s_raw_active = 0U;          /* 正常指令退出 raw 调试模式 */
    s_cmd_seen   = 1U;
    s_vx_mm_s    = vx_mm_s;
    s_vy_mm_s    = vy_mm_s;
    s_wz_mrad_s  = wz_mrad_s;
    taskEXIT_CRITICAL();
}

/**
 * @brief  停车：相当于下发 (0,0,0)。
 * @param  无
 * @retval 无
 */
void App_Motion_Stop(void)
{
    App_Motion_Apply(0, 0, 0);
}

/**
 * @brief  急停：解除 ARM 并清零速度。
 * @param  无
 * @retval 无
 * @note   之后必须先 ARM 才会动，这是刹车逻辑不是暂停。
 */
void App_Motion_EStop(void)
{
    taskENTER_CRITICAL();
    s_armed = 0U;
    s_raw_active = 0U;
    s_vx_mm_s = 0;
    s_vy_mm_s = 0;
    s_wz_mrad_s = 0;
    taskEXIT_CRITICAL();
}

/**
 * @brief  调试用：四轮开环吃同一个 PWM。
 * @param  pwm PWM 值，限幅到 ±APP_PWM_LIMIT
 * @retval 无
 */
void App_Motion_SetRaw(int16_t pwm)
{
    if (pwm > APP_PWM_LIMIT)
    {
        pwm = APP_PWM_LIMIT;
    }
    else if (pwm < -APP_PWM_LIMIT)
    {
        pwm = -(int16_t)APP_PWM_LIMIT;
    }

    Motion_ClearPi();

    taskENTER_CRITICAL();
    s_raw_pwm = pwm;
    s_raw_active = 1U;
    taskEXIT_CRITICAL();
}

/**
 * @brief  ARM，同时清故障。
 * @param  无
 * @retval 无
 */
void App_Motion_Arm(void)
{
    taskENTER_CRITICAL();
    s_fault = APP_FAULT_NONE;
    s_armed = 1U;
    taskEXIT_CRITICAL();
}

/**
 * @brief  读取速度指令。
 * @param  vx_mm_s   输出
 * @param  vy_mm_s   输出
 * @param  wz_mrad_s 输出
 * @retval 无
 */
void App_Motion_Get(int16_t *vx_mm_s, int16_t *vy_mm_s, int16_t *wz_mrad_s)
{
    taskENTER_CRITICAL();
    *vx_mm_s   = s_vx_mm_s;
    *vy_mm_s   = s_vy_mm_s;
    *wz_mrad_s = s_wz_mrad_s;
    taskEXIT_CRITICAL();
}

/**
 * @brief  是否已 ARM。
 * @param  无
 * @retval 1 = 允许输出
 */
uint8_t App_Motion_IsArmed(void)
{
    return s_armed;
}

/**
 * @brief  调试快照：实测轮速、目标轮速、输出 PWM。
 * @param  enc_mm_s    4 轮实测速度（mm/s）
 * @param  target_mm_s 4 轮目标速度（mm/s）
 * @param  pwm         4 轮输出；raw 模式下返回 raw 值
 * @retval 无
 * @note   raw 模式下 PI 不参与，所以不能报陈旧的闭环值。
 */
void App_Motion_GetDebug(int16_t enc_mm_s[4], int16_t target_mm_s[4], int16_t pwm[4])
{
    uint8_t i;

    taskENTER_CRITICAL();
    for (i = 0U; i < 4U; i++)
    {
        enc_mm_s[i]    = (int16_t)(s_enc[i] * 1000.0f);
        target_mm_s[i] = (int16_t)(s_motor[i].Target * 1000.0f);
        pwm[i]         = (s_raw_active != 0U)
                       ? s_raw_pwm
                       : (int16_t)s_motor[i].Motor_Pwm;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  是否有非零速度指令。
 * @param  无
 * @retval 1 = 在动
 */
uint8_t App_Motion_IsMoving(void)
{
    int16_t vx;
    int16_t vy;
    int16_t wz;

    App_Motion_Get(&vx, &vy, &wz);
    return ((vx != 0) || (vy != 0) || (wz != 0)) ? 1U : 0U;
}

/**
 * @brief  置故障位。
 * @param  fault APP_FAULT_xxx
 * @retval 无
 */
void App_Motion_SetFault(uint8_t fault)
{
    taskENTER_CRITICAL();
    s_fault |= fault;
    taskEXIT_CRITICAL();
}

/**
 * @brief  清全部故障。
 * @param  无
 * @retval 无
 */
void App_Motion_ClearFault(void)
{
    taskENTER_CRITICAL();
    s_fault = APP_FAULT_NONE;
    taskEXIT_CRITICAL();
}

/**
 * @brief  清单个故障位。
 * @param  fault APP_FAULT_xxx
 * @retval 无
 */
void App_Motion_ClearFaultBit(uint8_t fault)
{
    taskENTER_CRITICAL();
    s_fault &= (uint8_t)(~fault);
    taskEXIT_CRITICAL();
}

/**
 * @brief  读故障位。
 * @param  无
 * @retval APP_FAULT_xxx 按位或
 */
uint8_t App_Motion_GetFault(void)
{
    return s_fault;
}

/**
 * @brief  全向底盘的三轴速度平滑（每周期最多变化 0.01 m/s）。
 * @param  vx 目标 X 速度（m/s）
 * @param  vy 目标 Y 速度（m/s）
 * @param  vz 目标角速度（rad/s）
 * @retval 无
 * @note   目标为 0 时按 0.9 的比例衰减，避免突然停车的冲击。
 */
static void Motion_Smooth(float vx, float vy, float vz)
{
    const float step = 0.01f;

    if (vx > 0.0f)
    {
        s_smooth.VX += step;
    }
    else if (vx < 0.0f)
    {
        s_smooth.VX -= step;
    }
    else
    {
        s_smooth.VX *= 0.9f;
    }

    if (vy > 0.0f)
    {
        s_smooth.VY += step;
    }
    else if (vy < 0.0f)
    {
        s_smooth.VY -= step;
    }
    else
    {
        s_smooth.VY *= 0.9f;
    }

    if (vz > 0.0f)
    {
        s_smooth.VZ += step;
    }
    else if (vz < 0.0f)
    {
        s_smooth.VZ -= step;
    }
    else
    {
        s_smooth.VZ *= 0.9f;
    }

    s_smooth.VX = Motion_ClampFloat(s_smooth.VX, -fabsf(vx), fabsf(vx));
    s_smooth.VY = Motion_ClampFloat(s_smooth.VY, -fabsf(vy), fabsf(vy));
    s_smooth.VZ = Motion_ClampFloat(s_smooth.VZ, -fabsf(vz), fabsf(vz));
}

/**
 * @brief  把四个轮子的目标速度限制在 APP_WHEEL_TARGET_LIMIT 内。
 * @param  无
 * @retval 无
 */
static void Motion_ClampTargets(void)
{
    uint8_t i;

    for (i = 0; i < 4U; i++)
    {
        s_motor[i].Target = Motion_ClampFloat(s_motor[i].Target,
                                              -APP_WHEEL_TARGET_LIMIT,
                                               APP_WHEEL_TARGET_LIMIT);
    }
}

/**
 * @brief  按底盘型号把车体速度解算成四个轮子的目标速度。
 * @param  vx 前后速度（m/s）
 * @param  vy 左右速度（m/s）
 * @param  vz 角速度（rad/s）
 * @retval 无
 * @note   全向底盘先做平滑；阿克曼额外算舵机角度。
 */
static void Motion_DriveMotor(float vx, float vy, float vz)
{
    uint8_t mode;

    mode = BSP_Chassis_GetMode();

    if (BSP_Chassis_IsOmni())
    {
        Motion_Smooth(vx, vy, vz);
        vx = s_smooth.VX;
        vy = s_smooth.VY;
        vz = s_smooth.VZ;
    }

    s_motor[0].Target = 0.0f;
    s_motor[1].Target = 0.0f;
    s_motor[2].Target = 0.0f;
    s_motor[3].Target = 0.0f;

    if (mode == CAR_MEC)
    {
        float k = g_axle_spacing + g_wheel_spacing;

        s_motor[0].Target = +vy + vx - vz * k;
        s_motor[1].Target = -vy + vx - vz * k;
        s_motor[2].Target = +vy + vx + vz * k;
        s_motor[3].Target = -vy + vx + vz * k;
    }
    else if (mode == CAR_OMNI)
    {
        s_motor[0].Target =  vy + g_omni_radius * vz;
        s_motor[1].Target = -0.8660254f * vx - 0.5f * vy + g_omni_radius * vz;
        s_motor[2].Target = +0.8660254f * vx - 0.5f * vy + g_omni_radius * vz;
        s_motor[3].Target = 0.0f;
    }
    else if (mode == CAR_AKM)
    {
        float angle_r;
        float radius;
        float angle_servo;

        angle_r = Motion_ClampFloat(vz, -0.49f, 0.32f);

        if ((vx == 0.0f) && (vz != 0.0f))
        {
            /* 原地转：前轮只能打角度、自己产生不了自转，只能让后轮对转，
             * 绕后桥中心枢转。前进命令一旦非 0 就回到下面正常打舵逻辑。
             * 注意：前轮这时是回正的，转起来会被横向拖着刮擦——阿克曼
             * "能转但费胎"的折中，路面太涩时别长时间这么转。 */
            s_motor[0].Target = -vz * g_wheel_spacing / 2.0f;
            s_motor[1].Target = +vz * g_wheel_spacing / 2.0f;

            s_servo_us = (float)APP_SERVO_CENTER_US;
        }
        else
        {
            if (angle_r != 0.0f)
            {
                radius = g_axle_spacing / tanf(angle_r) - 0.5f * g_wheel_spacing;
                s_motor[0].Target = vx * (radius - 0.5f * g_wheel_spacing) / radius;
                s_motor[1].Target = vx * (radius + 0.5f * g_wheel_spacing) / radius;
            }
            else
            {
                s_motor[0].Target = vx;
                s_motor[1].Target = vx;
            }

            /* 三次多项式拟合转向角到舵机脉宽的映射 */
            angle_servo = -0.628f * powf(angle_r, 3.0f)
                        + 1.269f * powf(angle_r, 2.0f)
                        - 1.772f * angle_r
                        + 1.755f;
            s_servo_us = 1500.0f + (angle_servo - 1.755f) * 636.56f;
        }
    }
    else if (mode == CAR_DIFF)
    {
        s_motor[0].Target = vx - vz * g_wheel_spacing / 2.0f;
        s_motor[1].Target = vx + vz * g_wheel_spacing / 2.0f;
    }
    else if (mode == CAR_4WD)
    {
        float k = (g_wheel_spacing + g_axle_spacing) / 2.0f;

        s_motor[0].Target = vx - vz * k;
        s_motor[1].Target = vx - vz * k;
        s_motor[2].Target = vx + vz * k;
        s_motor[3].Target = vx + vz * k;
    }
    else /* CAR_TANK */
    {
        s_motor[0].Target = vx - vz * g_wheel_spacing / 2.0f;
        s_motor[1].Target = vx + vz * g_wheel_spacing / 2.0f;
    }

    Motion_ClampTargets();
}

/**
 * @brief  读四路编码器增量，按型号修正符号，换算成 m/s。
 * @param  无
 * @retval 无
 * @note   scale = 控制频率 × 轮周长 / 每圈脉冲数，
 *         即"一个脉冲对应多少 m/s"。
 */
static void Motion_ReadEncoders(void)
{
    int16_t raw_a;
    int16_t raw_b;
    int16_t raw_c;
    int16_t raw_d;
    int32_t a;
    int32_t b;
    int32_t c;
    int32_t d;
    float   scale;
    uint8_t mode;

    BSP_Encoder_ReadAll(&raw_a, &raw_b, &raw_c, &raw_d);

    a = raw_a;
    b = raw_b;
    c = raw_c;
    d = raw_d;
    mode = BSP_Chassis_GetMode();

    /* 不同底盘轮子安装方向不同，这里统一成"前进为正" */
    switch (mode)
    {
        case CAR_MEC:
            c = -c;
            d = -d;
            break;

        case CAR_OMNI:
            a = -a;
            b = -b;
            c = -c;
            break;

        case CAR_AKM:
        case CAR_DIFF:
        case CAR_TANK:
            b = -b;
            break;

        case CAR_4WD:
            c = -c;
            d = -d;
            break;

        default:
            break;
    }

    scale = ((float)APP_CONTROL_HZ * g_wheel_perimeter) / g_encoder_precision;

    s_enc[0] = (float)a * scale;
    s_enc[1] = (float)b * scale;
    s_enc[2] = (float)c * scale;
    s_enc[3] = (float)d * scale;
}

/**
 * @brief  单轮 PI 计算。
 * @param  index   轮序号 0..3
 * @param  encoder 实测轮速（m/s）
 * @param  target  目标轮速（m/s）
 * @retval 输出 PWM
 * @note   目标为 0 时积分项按 0.9 泄放，而不是冻结在上次的值——
 *         否则饱和后目标归零，误差为 0，PWM 会一直保持。
 */
static int Motion_PiUpdate(uint8_t index, float encoder, float target)
{
    float bias;
    float pwm;

    if (target == 0.0f)
    {
        s_pi_pwm[index] *= 0.90f;
        s_pi_last[index] = 0.0f;

        if ((s_pi_pwm[index] < 1.0f) && (s_pi_pwm[index] > -1.0f))
        {
            s_pi_pwm[index] = 0.0f;
        }

        return (int)s_pi_pwm[index];
    }

    bias = target - encoder;
    pwm  = s_pi_pwm[index]
         + APP_VELOCITY_KP * (bias - s_pi_last[index])
         + APP_VELOCITY_KI * bias;
    s_pi_last[index] = bias;

    /* 抗积分饱和：输出被削顶后不再继续累加 */
    if (pwm > (float)APP_PWM_LIMIT)
    {
        pwm = (float)APP_PWM_LIMIT;
    }
    else if (pwm < -(float)APP_PWM_LIMIT)
    {
        pwm = -(float)APP_PWM_LIMIT;
    }

    s_pi_pwm[index] = pwm;

    return (int)pwm;
}

/**
 * @brief  控制周期入口：安全检查 -> 解算 -> PI -> 写 PWM。
 * @param  无
 * @retval 无
 * @note   调用周期必须恒定为 APP_CONTROL_PERIOD_MS，否则轮速换算会错。
 */
void App_Motion_ControlStep(void)
{
    int16_t vx_mm;
    int16_t vy_mm;
    int16_t wz_mrad;
    int     pwm_a;
    int     pwm_b;
    int     pwm_c;
    int     pwm_d;
    uint8_t mode;

#if (APP_STARTUP_FORWARD_ENABLE != 0)
    /* 上电开环前进：等使能开关打开后开始计时，期间没有收到任何指令才执行。
     * 一旦收到正常指令立即放弃，避免和上位机抢控制权。 */
    if ((s_startup_done == 0U) && (s_cmd_seen == 0U))
    {
        if (s_startup_started == 0U)
        {
            if (BSP_Enable_IsOn() != 0U)
            {
                s_startup_start_tick = xTaskGetTickCount();
                s_startup_started    = 1U;
            }
        }

        if (s_startup_started != 0U)
        {
            if ((xTaskGetTickCount() - s_startup_start_tick)
                    < pdMS_TO_TICKS(APP_STARTUP_FORWARD_MS))
            {
                App_Motion_SetRaw(APP_STARTUP_TEST_PWM);
            }
            else
            {
                s_startup_done = 1U;
                App_Motion_Stop();
            }
        }
    }
#endif

    App_Motion_Get(&vx_mm, &vy_mm, &wz_mrad);

    /* 安全闸 1：硬件使能开关关闭 -> 强制停车 */
    if (BSP_Enable_IsOn() == 0U)
    {
        App_Motion_Stop();
        Motion_OutputZero();
        return;
    }

    /* 安全闸 2：急停后未 ARM -> 强制停车 */
    if (s_armed == 0U)
    {
        Motion_OutputZero();
        return;
    }

    /* 调试模式：四轮直接吃 raw PWM，跳过 PI */
    if (s_raw_active != 0U)
    {
        BSP_Motor_SetPwm((int)s_raw_pwm, (int)s_raw_pwm,
                         (int)s_raw_pwm, (int)s_raw_pwm);
        return;
    }

    Motion_DriveMotor((float)vx_mm / 1000.0f,
                      (float)vy_mm / 1000.0f,
                      (float)wz_mrad / 1000.0f);
    Motion_ReadEncoders();

    s_motor[0].Motor_Pwm = (float)Motion_PiUpdate(0, s_enc[0], s_motor[0].Target);
    s_motor[1].Motor_Pwm = (float)Motion_PiUpdate(1, s_enc[1], s_motor[1].Target);
    s_motor[2].Motor_Pwm = (float)Motion_PiUpdate(2, s_enc[2], s_motor[2].Target);
    s_motor[3].Motor_Pwm = (float)Motion_PiUpdate(3, s_enc[3], s_motor[3].Target);

    pwm_a = (int)s_motor[0].Motor_Pwm;
    pwm_b = (int)s_motor[1].Motor_Pwm;
    pwm_c = (int)s_motor[2].Motor_Pwm;
    pwm_d = (int)s_motor[3].Motor_Pwm;

    /* 不同底盘的电机安装方向不同，这里做最后的符号修正 */
    mode = BSP_Chassis_GetMode();
    switch (mode)
    {
        case CAR_MEC:
            BSP_Motor_SetPwm(-pwm_a, -pwm_b, pwm_c, pwm_d);
            break;

        case CAR_OMNI:
            BSP_Motor_SetPwm(pwm_a, pwm_b, pwm_c, pwm_d);
            break;

        case CAR_AKM:
        case CAR_DIFF:
        case CAR_TANK:
            BSP_Motor_SetPwm(-pwm_a, pwm_b, pwm_c, pwm_d);
            break;

        case CAR_4WD:
            BSP_Motor_SetPwm(-pwm_a, -pwm_b, pwm_c, pwm_d);
            break;

        default:
            BSP_Motor_SetPwm(0, 0, 0, 0);
            break;
    }

    if (mode == CAR_AKM)
    {
        if (s_servo_us < (float)APP_SERVO_MIN_US)
        {
            s_servo_us = (float)APP_SERVO_MIN_US;
        }
        if (s_servo_us > (float)APP_SERVO_MAX_US)
        {
            s_servo_us = (float)APP_SERVO_MAX_US;
        }
        BSP_Servo_Set((uint16_t)s_servo_us);
    }
}
