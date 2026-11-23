/***********************************************
 * bsp_uart3.c : USART3 板级驱动
 *
 * 硬件 : PD8 = TX, PD9 = RX (AF7), 115200-8-N-1
 *
 * 收 : USART3 中断只负责把字节塞进环形缓冲区，并唤醒接收任务；
 *      任务里用 BSP_UART3_Read() 取走，协议解析在任务上下文完成。
 * 发 : BSP_UART3_Send() 阻塞发送，内部用互斥量防止多任务字节交错。
 ***********************************************/

#include "bsp_uart3.h"
#include "app_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* 环形缓冲区：单生产者（中断）/ 单消费者（接收任务）。 */
static uint8_t          s_ring[APP_RX_RING_SIZE];
static volatile uint16_t s_head;        /* 中断写指针 */
static volatile uint16_t s_tail;        /* 任务读指针 */
static volatile uint32_t s_overflow;    /* 溢出字节数（缓冲区满或 ORE） */
static void            *s_notify_task;  /* 收到字节后要唤醒的任务句柄 */

/* 嗅探：最近 8 个原始字节，用来判断 PC 到底有没有发数据。 */
static volatile uint32_t s_rx_bytes;    /* 中断累计收到的字节数 */
static volatile uint8_t  s_sniff[8];    /* 最近 8 字节的环形记录 */
static volatile uint8_t  s_sniff_idx;   /* 嗅探写指针 */

/* 发送锁：多个任务可能同时往同一条线上打印，没有锁字节会互相穿插。 */
static SemaphoreHandle_t s_tx_lock;

/**
 * @brief  初始化 USART3：时钟、GPIO、串口参数、接收中断和发送互斥量。
 * @param  无
 * @retval 无
 * @note   1) 需在接收任务创建后、真正开始收数据前调用，否则中断先于
 *            任务句柄注册，前几个字节只进缓冲区不唤醒；
 *         2) 互斥量在这里创建，heap_4 的堆是静态数组，调度器启动前也可用；
 *         3) 中断抢占优先级设为 6，满足 FreeRTOS 的要求：只有数值 >=
 *            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY（本工程为 5）
 *            的中断才允许调用 FromISR 接口；
 *         4) 调用前需先执行 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)，
 *            否则优先级分组的含义不一致。
 */
void BSP_UART3_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    if (s_tx_lock == NULL)
    {
        s_tx_lock = xSemaphoreCreateMutex();
    }

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

/**
 * @brief  注册"收到字节时被唤醒"的任务句柄。
 * @param  task_handle 目标任务句柄（TaskHandle_t 强制转成 void * 传入）
 * @retval 无
 * @note   中断里只对非 NULL 的句柄调用 vTaskNotifyGiveFromISR()；
 *         传 NULL 等于关闭唤醒，此时字节仍然会进环形缓冲区，
 *         只是没有任何任务会被通知。
 */
void BSP_UART3_SetNotifyTask(void *task_handle)
{
    s_notify_task = task_handle;
}

/**
 * @brief  从接收环形缓冲区取出最多 max_len 个字节（非阻塞）。
 * @param  buf     输出缓冲区，容量至少 max_len 字节
 * @param  max_len 本次最多取出多少字节
 * @retval 实际取出的字节数；返回 0 表示当前没有数据
 * @note   只允许一个任务调用（单消费者模型）：中断只改 s_head，
 *         本函数只改 s_tail，因此读写指针不会互相踩，无需关中断。
 */
uint16_t BSP_UART3_Read(uint8_t *buf, uint16_t max_len)
{
    uint16_t n = 0;

    while ((s_tail != s_head) && (n < max_len))
    {
        buf[n] = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1U) & (APP_RX_RING_SIZE - 1U));
        n++;
    }
    return n;
}

/**
 * @brief  阻塞发送一段数据，返回时最后一个字节已完全移出移位寄存器。
 * @param  buf 待发送数据
 * @param  len 字节数
 * @retval 无
 * @note   1) 调度器启动前不加锁，此时不可能有任务竞争；
 *         2) 调度器启动后先取 TX 互斥量，portMAX_DELAY 无限等，不会丢数据，
 *            但也意味着持锁任务如果卡死，其他打印任务会跟着卡住；
 *         3) 全程忙等 TXE/TC 标志，调用者被阻塞约 len*10/115200 秒
 *            （115200-8-N-1 下每字节 10 bit，约 87 us）。
 */
void BSP_UART3_Send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint8_t  locked = 0U;

    if ((s_tx_lock != NULL) &&
        (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        if (xSemaphoreTake(s_tx_lock, portMAX_DELAY) == pdPASS)
        {
            locked = 1U;
        }
    }

    for (i = 0; i < len; i++)
    {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
        {
        }
        USART_SendData(USART3, buf[i]);
    }
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
    }

    if (locked != 0U)
    {
        xSemaphoreGive(s_tx_lock);
    }
}

/**
 * @brief  读取累计溢出字节数。
 * @param  无
 * @retval 溢出计数（缓冲区满丢弃 + 硬件 ORE 丢字节）
 * @note   可作为链路是否丢数据的指标；原工程把它计入状态帧的 err 字段。
 */
uint32_t BSP_UART3_GetOverflowCount(void)
{
    return s_overflow;
}

/**
 * @brief  读取 USART3 中断累计收到的字节总数。
 * @param  无
 * @retval 字节数（不论帧是否合法）
 * @note   主要给调试用：这个值在变说明线序、波特率、硬件连接没问题，
 *         剩下的问题就出在协议层。
 */
uint32_t BSP_UART3_GetRxByteCount(void)
{
    return s_rx_bytes;
}

/**
 * @brief  导出最近收到的最多 8 个原始字节（按旧 → 新的顺序）。
 * @param  out 输出缓冲区，容量至少 8 字节
 * @retval 实际导出的字节数（0..8）；刚上电、还没收到数据时返回 0
 * @note   用来区分"PC 发的是二进制帧 68 03 ..."还是
 *         "ASCII 文本 '6' '8' ...（即 36 38 20）"，排查时很直观。
 */
uint8_t BSP_UART3_GetSniff(uint8_t *out)
{
    uint8_t n;
    uint8_t i;

    n = (s_sniff_idx < 8U) ? s_sniff_idx : 8U;
    for (i = 0U; i < n; i++)
    {
        out[i] = s_sniff[(uint8_t)((s_sniff_idx - n + i) & 0x07U)];
    }
    return n;
}

/**
 * @brief  USART3 接收中断服务函数：只搬运字节，不做任何协议解析。
 * @param  无
 * @retval 无
 * @note   1) 读 DR 会同时清掉 RXNE 和 ORE 标志；
 *         2) ORE 表示硬件已经丢过字节，该字节不放进缓冲区，直接计溢出；
 *         3) 缓冲区满时丢弃新字节并计溢出，保留旧数据（不会覆盖未读内容）；
 *         4) 只有注册了任务句柄、且调度器已启动时，才发任务通知并触发
 *            PendSV 让更高优先级的任务立刻运行；
 *         5) 嗅探数组不管溢出与否都会记录，方便看"最后收到的 8 个字节"。
 */
void USART3_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    uint32_t   sr;

    sr = USART3->SR;
    if ((sr & (USART_FLAG_RXNE | USART_FLAG_ORE)) != 0U)
    {
        uint8_t  byte;
        uint16_t next;

        byte = (uint8_t)USART3->DR;     /* 读 DR：同时清 RXNE 和 ORE */

        s_rx_bytes++;
        s_sniff[(uint8_t)(s_sniff_idx & 0x07U)] = byte;
        s_sniff_idx++;

        if ((sr & USART_FLAG_ORE) != 0U)
        {
            s_overflow++;
        }
        else
        {
            next = (uint16_t)((s_head + 1U) & (APP_RX_RING_SIZE - 1U));
            if (next != s_tail)
            {
                s_ring[s_head] = byte;
                s_head = next;
            }
            else
            {
                s_overflow++;
            }

            if ((s_notify_task != NULL) &&
                (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
            {
                vTaskNotifyGiveFromISR((TaskHandle_t)s_notify_task, &woken);
                portYIELD_FROM_ISR(woken);
            }
        }
    }
}
