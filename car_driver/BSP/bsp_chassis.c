/***********************************************
 * car_driver : 底盘型号识别与机械参数
 * ADC1_IN9 = PB1，电位器电压分 6 档对应 6 种底盘。
 * 也可以用 app_config.h 的 APP_CHASSIS_FORCE 强制指定型号。
 ***********************************************/

#include "bsp_chassis.h"
#include "app_config.h"

#define CHASSIS_POT_CHANNEL   9U        /* ADC1_IN9 = PB1 */
#define CHASSIS_PI            3.1415926f

float   g_wheel_spacing;
float   g_axle_spacing;
float   g_omni_radius;
float   g_wheel_perimeter;
float   g_encoder_precision;

static uint8_t  s_car_mode;             /* 本次运行的底盘型号 */
static uint16_t s_pot_raw;              /* 上电时的电位器采样值 */

/**
 * @brief  初始化 ADC1（单次转换、12 位、软件触发）。
 * @param  无
 * @retval 无
 * @note   ADC 时钟 = APB2/4 = 21 MHz，符合 12 位最大 36 MHz 的要求。
 */
static void Chassis_AdcInit(void)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
    ADC_CommonInitTypeDef  ADC_CommonInitStructure;
    ADC_InitTypeDef        ADC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);

    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode         = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode   = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion      = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);
}

/**
 * @brief  软件触发一次单通道转换并等待结果。
 * @param  channel ADC 通道号
 * @retval 转换结果；超时返回 0
 * @note   带 100 万次循环的guard，避免 ADC 没起来时死等。
 */
static uint16_t Chassis_AdcRead(uint8_t channel)
{
    uint32_t guard = 1000000U;

    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_480Cycles);
    ADC_SoftwareStartConv(ADC1);
    while ((ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) && (guard > 0U))
    {
        guard--;
    }
    if (guard == 0U)
    {
        return 0U;
    }
    return ADC_GetConversionValue(ADC1);
}

/**
 * @brief  采样 10 次取平均，读出电位器值。
 * @param  无
 * @retval 0..4095
 */
uint16_t BSP_Chassis_ReadPot(void)
{
    uint32_t sum = 0;
    uint8_t  i;

    for (i = 0; i < 10U; i++)
    {
        sum += Chassis_AdcRead(CHASSIS_POT_CHANNEL);
    }
    return (uint16_t)(sum / 10U);
}

/**
 * @brief  按型号装载机械参数（轮距、轴距、轮周长等）。
 * @param  mode 底盘型号 CAR_xxx
 * @retval 无
 * @note   末尾统一算编码器精度：13 线霍尔 × 4 倍频 × 30 减速比。
 */
static void Chassis_Apply(uint8_t mode)
{
    switch (mode)
    {
        case CAR_MEC:
            g_wheel_spacing = 0.0930f;
            g_axle_spacing  = 0.0850f;
            g_omni_radius   = 0.0f;
            g_wheel_perimeter = 0.075f * CHASSIS_PI;
            break;

        case CAR_OMNI:
            g_wheel_spacing = 0.0f;
            g_axle_spacing  = 0.0f;
            g_omni_radius   = 0.290f;
            g_wheel_perimeter = 0.060f * CHASSIS_PI;
            break;

        case CAR_AKM:
            g_wheel_spacing = 0.162f;
            g_axle_spacing  = 0.144f;
            g_omni_radius   = 0.0f;
            g_wheel_perimeter = 0.065f * CHASSIS_PI;
            break;

        case CAR_DIFF:
            g_wheel_spacing = 0.177f;
            g_axle_spacing  = 0.0f;
            g_omni_radius   = 0.0f;
            g_wheel_perimeter = 0.065f * CHASSIS_PI;
            break;

        case CAR_4WD:
            g_wheel_spacing = 0.187f;
            g_axle_spacing  = 0.173f;
            g_omni_radius   = 0.0f;
            g_wheel_perimeter = 0.065f * CHASSIS_PI;
            break;

        case CAR_TANK:
        default:
            g_wheel_spacing = 0.235f;
            g_axle_spacing  = 0.222f;
            g_omni_radius   = 0.0f;
            g_wheel_perimeter = 0.043f * CHASSIS_PI;
            break;
    }

    /* 13 线霍尔编码器，4 倍频，1:30 减速箱 */
    g_encoder_precision = 4.0f * 13.0f * 30.0f;
}

/**
 * @brief  初始化 ADC、识别底盘型号、装载机械参数。
 * @param  无
 * @retval 无
 * @note   电位器 0..4095 分 6 档，除数 346 与原厂分压电路一致。
 */
void BSP_Chassis_Init(void)
{
    uint16_t pot;
    uint8_t  mode;
    uint8_t  divisor;

    Chassis_AdcInit();

    pot       = BSP_Chassis_ReadPot();
    s_pot_raw = pot;
    divisor   = (uint8_t)(2048U / 6U + 5U);   /* 346，和原厂电位器分压一致 */
    mode      = (uint8_t)(pot / divisor);
    if (mode > 5U)
    {
        mode = 5U;
    }

#if (APP_CHASSIS_FORCE >= 0)
    mode = (uint8_t)APP_CHASSIS_FORCE;
#endif

    s_car_mode = mode;
    Chassis_Apply(mode);
}

/**
 * @brief  返回当前底盘型号。
 * @param  无
 * @retval CAR_xxx
 */
uint8_t BSP_Chassis_GetMode(void)
{
    return s_car_mode;
}

/**
 * @brief  返回上电时采样的电位器原始值（调试用）。
 * @param  无
 * @retval 0..4095
 */
uint16_t BSP_Chassis_GetPot(void)
{
    return s_pot_raw;
}

/**
 * @brief  现场重新采样一次电位器，不改变型号。
 * @param  无
 * @retval 0..4095
 */
uint16_t BSP_Chassis_ReadPotLive(void)
{
    return BSP_Chassis_ReadPot();
}

/**
 * @brief  判断是否全向底盘（可横移）。
 * @param  无
 * @retval 1 = MEC/OMNI
 */
uint8_t BSP_Chassis_IsOmni(void)
{
    return ((s_car_mode == CAR_MEC) || (s_car_mode == CAR_OMNI)) ? 1U : 0U;
}

/**
 * @brief  判断是否阿克曼底盘（需要舵机）。
 * @param  无
 * @retval 1 = AKM
 */
uint8_t BSP_Chassis_IsAckermann(void)
{
    return (s_car_mode == CAR_AKM) ? 1U : 0U;
}
