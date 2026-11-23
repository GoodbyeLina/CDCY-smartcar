/***********************************************
 * car_driver : 小车驱动（STM32F407VE / WHEELTEC C30D）
 * Port  : USART3, PD8 = TX, PD9 = RX (AF7), 115200-8-N-1
 *
 * 分层：
 *   BSP   bsp_gpio / bsp_led / bsp_uart3 / bsp_motor / bsp_servo /
 *         bsp_encoder / bsp_chassis        —— 只碰寄存器，不含业务
 *   APP   app_protocol  —— 帧解析 + 按代码号分发
 *         app_motion    —— 运动学解算 + PI 速度环
 *         app_tasks     —— 100 Hz 控制任务 / 10 Hz 链路任务
 *         app_debug     —— 开机横幅 + 心跳打印
 *   USER  main / 中断向量 / 系统时钟
 *
 * 任务与优先级：
 *   CTRL  6  100 Hz 控制，最高，不能被打印阻塞
 *   PROTO 5  串口接收解析
 *   LINK  4  指令超时保护 + 查询应答
 *   DBG   1  LED 心跳 + 串口心跳
 *
 * 注意：不要调用 delay_init()。SysTick 重装值由 FreeRTOS 在
 * xPortStartScheduler() 里配置，提前开 1 ms 节拍会在
 * pxCurrentTCB == NULL 时进 xTaskIncrementTick()。
 ***********************************************/

#include "stm32f4xx.h"
#include "app_config.h"
#include "app_debug.h"
#include "app_motion.h"
#include "app_protocol.h"
#include "app_tasks.h"
#include "bsp_chassis.h"
#include "bsp_encoder.h"
#include "bsp_gpio.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart3.h"

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief  程序入口：按层初始化外设、创建任务、启动调度器。
 * @param  无
 * @retval 不会返回（调度器启动后由任务接管）
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* ---- BSP：先让硬件进入确定状态，再谈控制 ---- */
    BSP_LED_Init();             /* PE8 先灭灯 */
    BSP_GPIO_Init();            /* PD3 使能开关 */
    BSP_Motor_Init();           /* 四路 PWM，输出 0 */
    BSP_Encoder_Init();         /* 四路编码器计数 */
    BSP_Chassis_Init();         /* ADC 识别底盘型号 */

    if (BSP_Chassis_IsAckermann())
    {
        BSP_Servo_Init();       /* 只有阿克曼才需要舵机 */
    }

    /* ---- APP：运动状态清零，再起协议和任务 ---- */
    App_Motion_Init();
    BSP_UART3_Init();
    App_Protocol_Init();        /* 内含创建 PROTO 任务并注册通知 */

    if (xTaskCreate(App_Control_Task, "CTRL", APP_TASK_STACK_CONTROL,
                    NULL, APP_TASK_PRIO_CONTROL, NULL) != pdPASS)
    {
        __disable_irq();
        for (;;) { }
    }

    if (xTaskCreate(App_Link_Task, "LINK", APP_TASK_STACK_LINK,
                    NULL, APP_TASK_PRIO_LINK, NULL) != pdPASS)
    {
        __disable_irq();
        for (;;) { }
    }

    if (xTaskCreate(App_Debug_Task, "DBG", APP_TASK_STACK_DEBUG,
                    NULL, APP_TASK_PRIO_DEBUG, NULL) != pdPASS)
    {
        __disable_irq();
        for (;;) { }
    }

    /* 横幅在调度器启动前打印：此时 BSP_UART3_Send 不取互斥量，不会阻塞 */
    App_Debug_Banner();

    vTaskStartScheduler();

    for (;;)
    {
    }
}

/**
 * @brief  FreeRTOS 栈溢出钩子：停机并关中断，方便调试器抓现场。
 * @param  task      溢出任务句柄
 * @param  task_name 任务名
 * @retval 不返回
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;

    __disable_irq();
    for (;;)
    {
    }
}

/**
 * @brief  FreeRTOS 堆分配失败钩子：停机并关中断。
 * @param  无
 * @retval 不返回
 */
void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    for (;;)
    {
    }
}
