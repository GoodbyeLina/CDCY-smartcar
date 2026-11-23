#ifndef BSP_UART3_H
#define BSP_UART3_H

#include "stm32f4xx.h"

/* USART3: PD8 = TX, PD9 = RX, AF7, 115200 8-N-1 */

/**
 * @brief  初始化 USART3：时钟、GPIO、串口参数、接收中断、发送互斥量。
 * @param  无
 * @retval 无
 * @note   调用前需先配置 NVIC 优先级分组；需在接收任务创建之后调用。
 */
void BSP_UART3_Init(void);

/**
 * @brief  注册接收任务，收到字节时中断会唤醒它。
 * @param  task_handle 任务句柄（TaskHandle_t 转 void *），NULL 表示不唤醒
 * @retval 无
 */
void BSP_UART3_SetNotifyTask(void *task_handle);

/**
 * @brief  从接收环形缓冲区取数据（非阻塞，单消费者）。
 * @param  buf     输出缓冲区
 * @param  max_len 本次最多取多少字节
 * @retval 实际取到的字节数，0 表示暂无数据
 */
uint16_t BSP_UART3_Read(uint8_t *buf, uint16_t max_len);

/**
 * @brief  阻塞发送数据，返回时最后一个字节已发送完成。
 * @param  buf 数据指针
 * @param  len 字节数
 * @retval 无
 * @note   调度器启动后内部会取 TX 互斥量，多任务可安全共用。
 */
void BSP_UART3_Send(const uint8_t *buf, uint16_t len);

/**
 * @brief  读取累计溢出字节数（缓冲区满 + 硬件 ORE）。
 * @param  无
 * @retval 溢出计数
 */
uint32_t BSP_UART3_GetOverflowCount(void);

/**
 * @brief  读取中断累计收到的字节总数。
 * @param  无
 * @retval 字节数（不论帧是否合法）
 */
uint32_t BSP_UART3_GetRxByteCount(void);

/**
 * @brief  导出最近最多 8 个原始字节（旧 → 新），用于排查收发。
 * @param  out 输出缓冲区（至少 8 字节）
 * @retval 实际导出的字节数（0..8）
 */
uint8_t  BSP_UART3_GetSniff(uint8_t *out);

#endif /* BSP_UART3_H */
