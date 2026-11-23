/***********************************************
 * car_driver : 调试图（开机横幅 + 二进制心跳）
 * Port  : USART3, 115200-8-N-1（和协议共用一个口）
 *
 * 现在串口上不再有 "[HB] up=.." 这种文本行，改成两帧纯数据，
 * 由上位机按代码号解析：
 *   0x20 心跳帧 16 字节：up_s, heap, rx, ovf, ok, bad
 *   0x21 电机帧 34 字节：mode, en, armed, fault, vx, vy, wz,
 *                        target[4], speed[4], pwm[4]
 *   0x22 驱动帧  6 字节：motion(前进/后退/停止), reserved, speed, turn
 * 字段偏移和字节序见 car_protocol.h 的说明，全部小端。
 *
 * 开机横幅仍然是一次性 ASCII，方便人眼确认版本和接线。
 ***********************************************/

#include <stdio.h>
#include <string.h>

#include "app_debug.h"
#include "app_config.h"
#include "app_motion.h"
#include "app_protocol.h"
#include "car_telemetry.h"
#include "bsp_chassis.h"
#include "bsp_gpio.h"
#include "bsp_led.h"
#include "bsp_uart3.h"

#include "FreeRTOS.h"
#include "task.h"

/* 横幅单行最大长度；横幅是唯一还在用文本的地方 */
#define APP_DEBUG_LINE_SIZE 160

/**
 * @brief  底盘型号的短名字，打印用。
 * @param  mode CAR_xxx
 * @retval 3~4 个字符的名字
 */
static const char *Debug_ModeName(uint8_t mode)
{
    switch (mode)
    {
        case CAR_MEC:  return "MEC";
        case CAR_OMNI: return "OMNI";
        case CAR_AKM:  return "AKM";
        case CAR_DIFF: return "DIFF";
        case CAR_4WD:  return "4WD";
        default:       return "TANK";
    }
}

/**
 * @brief  以一帧 0x30 INFO 发一段 ASCII 文本（超过 34 字节自动分帧）。
 * @param  text 以 '\0' 结尾的字符串
 * @retval 无
 * @note   串口上不再有裸文本：所有输出都是带帧头/校验的帧，
 *         上位机也就不会把二进制帧误当文字显示。
 */
static void Debug_SendText(const char *text)
{
    unsigned int len;

    if (text == NULL)
    {
        return;
    }

    len = (unsigned int)strlen(text);
    while (len > 0U)
    {
        unsigned int chunk = (len > (unsigned int)CAR_PROTO_MAX_DATA)
                           ? (unsigned int)CAR_PROTO_MAX_DATA
                           : len;

        App_Protocol_SendFrame(CAR_CODE_INFO, (const uint8_t *)text, (uint8_t)chunk);
        text += chunk;
        len  -= chunk;
    }
}

/**
 * @brief  上电横幅：版本、底盘型号、使能状态、串口参数、帧格式。
 * @param  无
 * @retval 无
 * @note   在 main() 里、调度器启动前调用；此时发送不取互斥量。
 */
void App_Debug_Banner(void)
{
    char line[APP_DEBUG_LINE_SIZE];
    int  n;

    n = snprintf(line, sizeof(line), "car_driver v%u.%u.%u\n",
                 (unsigned)APP_FW_VERSION_MAJOR,
                 (unsigned)APP_FW_VERSION_MINOR,
                 (unsigned)APP_FW_VERSION_PATCH);
    if (n > 0) { Debug_SendText(line); }

    n = snprintf(line, sizeof(line), "chassis %u %s pot=%u\n",
                 (unsigned)BSP_Chassis_GetMode(),
                 Debug_ModeName(BSP_Chassis_GetMode()),
                 (unsigned)BSP_Chassis_GetPot());
    if (n > 0) { Debug_SendText(line); }

    n = snprintf(line, sizeof(line), "enable %u arm %u heap %u\n",
                 (unsigned)BSP_Enable_IsOn(),
                 (unsigned)App_Motion_IsArmed(),
                 (unsigned)xPortGetFreeHeapSize());
    if (n > 0) { Debug_SendText(line); }

    n = snprintf(line, sizeof(line), "uart3 115200 8N1 PD8=TX\n");
    if (n > 0) { Debug_SendText(line); }

    n = snprintf(line, sizeof(line), "hb 0x20/16B mot 0x21/34B drv 0x22/6B tlm 0x0F/1B\n");
    if (n > 0) { Debug_SendText(line); }
}

/**
 * @brief  按遥测开关发送 0x20 心跳帧、0x21 电机帧和 0x22 驱动帧。
 * @param  无
 * @retval 无
 * @note   发哪几帧由 0x0F 命令保存的位图决定（CAR_TLM_BIT_xxx），
 *         上电默认三位全开；字段偏移见 car_protocol.h。
 */
void App_Debug_Heartbeat(void)
{
#if (APP_DEBUG_ENABLE != 0)
    CarTlm_Heartbeat hb;
    CarTlm_Motor     mot;
    int16_t          enc[4];
    int16_t          tgt[4];
    int16_t          pwm[4];
    int16_t          vx;
    int16_t          vy;
    int16_t          wz;
    uint8_t          i;
    uint8_t          mask;

    mask = App_Protocol_GetTelemetryMask();

    /* ---- 0x20 心跳帧：16 字节（位 0） ---- */
    if ((mask & (uint8_t)CAR_TLM_BIT_HB) != 0U)
    {
        hb.up_s = (uint16_t)(xTaskGetTickCount() / (TickType_t)configTICK_RATE_HZ);
        hb.heap = (uint16_t)xPortGetFreeHeapSize();
        hb.rx   = BSP_UART3_GetRxByteCount();
        hb.ovf  = BSP_UART3_GetOverflowCount();
        hb.ok   = (uint16_t)App_Protocol_GetFrameCount();
        hb.bad  = (uint16_t)App_Protocol_GetErrorCount();

        App_Protocol_SendFrame(CAR_CODE_HB, (const uint8_t *)&hb,
                               (uint8_t)sizeof(hb));
    }

    /* ---- 0x21 电机帧：34 字节（位 1） ---- */
    if ((mask & (uint8_t)CAR_TLM_BIT_MOT) != 0U)
    {
        App_Motion_Get(&vx, &vy, &wz);
        App_Motion_GetDebug(enc, tgt, pwm);

        mot.mode   = BSP_Chassis_GetMode();
        mot.enable = BSP_Enable_IsOn();
        mot.armed  = App_Motion_IsArmed();
        mot.fault  = App_Motion_GetFault();
        mot.vx     = vx;
        mot.vy     = vy;
        mot.wz     = wz;

        for (i = 0U; i < 4U; i++)
        {
            mot.target[i] = tgt[i];
            mot.speed[i]  = enc[i];
            mot.pwm[i]    = pwm[i];
        }

        App_Protocol_SendFrame(CAR_CODE_MOT, (const uint8_t *)&mot,
                               (uint8_t)sizeof(mot));
    }

    /* ---- 0x22 驱动帧：6 字节（位 2，上位机设定的方向 + 速度 + 角速度） ---- */
    if ((mask & (uint8_t)CAR_TLM_BIT_DRIVE) != 0U)
    {
        App_Protocol_SendDrive();
    }
#endif
}

/**
 * @brief  调试/心跳任务：500 ms 一周期，翻 LED，按周期发三帧数据。
 * @param  pv_parameters 未使用
 * @retval 无
 * @note   LED 周期固定 500 ms；数据帧周期由 APP_DEBUG_PERIOD_MS 决定。
 *         APP_LED_MODE: 0 = 心跳翻转，1 = 强制灭，2 = 强制亮。
 */
void App_Debug_Task(void *pv_parameters)
{
    TickType_t last_wake;
#if (APP_DEBUG_ENABLE != 0) && (APP_DEBUG_PERIOD_MS > 0)
    uint32_t   hb_div = 0U;
#endif

    (void)pv_parameters;

#if (APP_LED_MODE == 1)
    BSP_LED_Write(0U);
#elif (APP_LED_MODE == 2)
    BSP_LED_Write(1U);
#endif

    last_wake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));

#if (APP_LED_MODE == 0)
        BSP_LED_Toggle();
#elif (APP_LED_MODE == 1)
        BSP_LED_Write(0U);
#else
        BSP_LED_Write(1U);
#endif

#if (APP_DEBUG_ENABLE != 0) && (APP_DEBUG_PERIOD_MS > 0)
        hb_div++;
        if (hb_div >= ((APP_DEBUG_PERIOD_MS + 499U) / 500U))
        {
            hb_div = 0U;
            App_Debug_Heartbeat();
        }
#endif
    }
}
