/***********************************************
 * car_driver : 周期任务
 * Control : 100 Hz 运动控制
 * Link    : 10 Hz 指令超时保护 + 查询应答
 ***********************************************/

#include "app_tasks.h"
#include "app_config.h"
#include "app_motion.h"
#include "app_protocol.h"

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief  控制任务：固定 10 ms 调一次运动控制步进。
 * @param  pv_parameters 未使用
 * @retval 无
 * @note   last_wake 在任务启动时取一次，之后 vTaskDelayUntil 自动累加，
 *         不会因为函数执行时间而漂移。
 */
void App_Control_Task(void *pv_parameters)
{
    TickType_t last_wake;

    (void)pv_parameters;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        App_Motion_ControlStep();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));
    }
}

/**
 * @brief  链路任务：检测指令超时并停车，同时处理查询应答。
 * @param  pv_parameters 未使用
 * @retval 无
 * @note   1) 只有"曾经收到过帧"之后才做超时判断，避免上电即报警；
 *         2) 超时后先停车再置故障位；
 *         3) 查询应答放在这里发，不在协议任务里发，保证发送方只有一处。
 */
void App_Link_Task(void *pv_parameters)
{
    TickType_t last_wake;

    (void)pv_parameters;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));

#if (APP_CMD_TIMEOUT_MS > 0)
        if ((App_Protocol_HasSeenFrame() != 0U) &&
            ((xTaskGetTickCount() - (TickType_t)App_Protocol_GetLastFrameTick())
                > pdMS_TO_TICKS(APP_CMD_TIMEOUT_MS)))
        {
            if (App_Motion_IsMoving() != 0U)
            {
                App_Motion_Stop();
            }
            App_Motion_SetFault(APP_FAULT_TIMEOUT);
        }
#endif

        (void)App_Protocol_ServiceStatus();
    }
}
