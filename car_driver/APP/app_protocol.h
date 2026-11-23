#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>
#include "app_config.h"
#include "car_protocol.h"    /* 协议常量、代码号、状态机（与上位机共用同一份） */

/*
 * 串口帧格式（主机 <-> 小车 双向同一套）：
 *
 *   +------+------+------+-----------+------+------+
 *   | 0x68 | LEN  | CODE | DATA[n]   | SUM  | 0x16 |
 *   +------+------+------+-----------+------+------+
 *
 *   LEN = 1 + n，即"代码号 + 数据内容"的字节数
 *   SUM = CODE 到 DATA 全部字节的和，按字节不进位（& 0xFF）
 *
 * 协议的定义和解析状态机都在 ..\include\car_protocol.h / .c 里，
 * 这里只做名字映射和 APP 层接口，避免两处各写一份常量。
 */
#define APP_PROTO_HEAD        CAR_PROTO_HEAD
#define APP_PROTO_TAIL        CAR_PROTO_TAIL
#define APP_PROTO_LEN_MIN     CAR_PROTO_MIN_LEN
#define APP_PROTO_LEN_MAX     CAR_PROTO_MAX_LEN

#define APP_PROTO_CODE_ARM         CAR_CODE_ARM
#define APP_PROTO_CODE_FORWARD     CAR_CODE_FORWARD
#define APP_PROTO_CODE_BACKWARD    CAR_CODE_BACKWARD
#define APP_PROTO_CODE_LEFT        CAR_CODE_LEFT
#define APP_PROTO_CODE_RIGHT       CAR_CODE_RIGHT
#define APP_PROTO_CODE_STOP        CAR_CODE_STOP
#define APP_PROTO_CODE_SPIN_L      CAR_CODE_SPIN_L
#define APP_PROTO_CODE_SPIN_R      CAR_CODE_SPIN_R
#define APP_PROTO_CODE_VECTOR      CAR_CODE_VECTOR
#define APP_PROTO_CODE_SET_DEF     CAR_CODE_SET_DEF
#define APP_PROTO_CODE_QUERY       CAR_CODE_QUERY
#define APP_PROTO_CODE_ESTOP       CAR_CODE_ESTOP
#define APP_PROTO_CODE_RAW         CAR_CODE_RAW
#define APP_PROTO_CODE_SET_SPEED   CAR_CODE_SET_SPEED
#define APP_PROTO_CODE_SET_TURN    CAR_CODE_SET_TURN
#define APP_PROTO_CODE_SET_TLM     CAR_CODE_SET_TLM
#define APP_PROTO_CODE_STATUS      CAR_CODE_STATUS

/**
 * @brief  初始化协议模块：清状态、建任务、把任务注册给串口驱动。
 * @param  无
 * @retval 无
 * @note   必须在 BSP_UART3_Init() 之后、vTaskStartScheduler() 之前调用。
 */
void App_Protocol_Init(void);

/**
 * @brief  从环形缓冲区取一次数据并喂给解析器（取到空为止）。
 * @param  无
 * @retval 无
 */
void App_Protocol_Process(void);

/**
 * @brief  接收任务主体：等中断通知 -> 取数据 -> 解析；半帧超时自动复位。
 * @param  pv_parameters 未使用
 * @retval 无
 */
void App_Protocol_Task(void *pv_parameters);

/**
 * @brief  主动发送一帧状态（0x8A）。
 * @param  无
 * @retval 无
 */
void App_Protocol_SendStatus(void);

/**
 * @brief  主动发送一帧驱动状态（0x22），内容是保存下来的上位机设定。
 * @param  无
 * @retval 无
 * @note   发送前会按当前运动层指令刷新一次，保证帧内容与实际一致。
 */
void App_Protocol_SendDrive(void);

/**
 * @brief  设置遥测帧开关（0x0F 命令保存下来的位图）。
 * @param  mask CAR_TLM_BIT_xxx 按位或；未定义的位忽略
 * @retval 无
 * @note   由 0x0F 命令调用；调试任务每轮读它决定发哪几帧。
 */
void App_Protocol_SetTelemetryMask(uint8_t mask);

/**
 * @brief  读取遥测帧开关。
 * @param  无
 * @retval CAR_TLM_BIT_xxx 按位或（上电默认 CAR_TLM_BIT_ALL）
 */
uint8_t App_Protocol_GetTelemetryMask(void);

/**
 * @brief  按帧格式组帧并发送（状态帧、调试帧都走这里）。
 * @param  code     代码号
 * @param  data     数据内容，可为 NULL
 * @param  data_len 数据字节数，不得超过 CAR_PROTO_MAX_DATA
 * @retval 无
 * @note   内部走 TX 互斥量，多任务可安全调用。
 */
void App_Protocol_SendFrame(uint8_t code, const uint8_t *data, uint8_t data_len);

/**
 * @brief  如果有挂起的查询请求，就发一帧状态。
 * @param  无
 * @retval 1 = 本次发出了状态帧，0 = 没有请求
 */
uint8_t App_Protocol_ServiceStatus(void);

/**
 * @brief  合法帧总数。
 * @param  无
 * @retval 帧数
 */
uint32_t App_Protocol_GetFrameCount(void);

/**
 * @brief  错误总数（校验/帧尾/长度/未知代码 + 串口溢出）。
 * @param  无
 * @retval 错误数
 */
uint32_t App_Protocol_GetErrorCount(void);

/**
 * @brief  校验和错误的次数。
 * @param  无
 * @retval 次数
 */
uint32_t App_Protocol_GetSumErrorCount(void);

/**
 * @brief  帧尾错误的次数。
 * @param  无
 * @retval 次数
 */
uint32_t App_Protocol_GetTailErrorCount(void);

/**
 * @brief  长度字段非法的次数。
 * @param  无
 * @retval 次数
 */
uint32_t App_Protocol_GetLenErrorCount(void);

/**
 * @brief  清零全部统计计数。
 * @param  无
 * @retval 无
 */
void App_Protocol_ResetStats(void);

/**
 * @brief  是否收到过至少一帧合法指令。
 * @param  无
 * @retval 1 = 收到过
 */
uint8_t App_Protocol_HasSeenFrame(void);

/**
 * @brief  最近一帧合法指令的时间戳（系统节拍）。
 * @param  无
 * @retval Tick 值
 */
uint32_t App_Protocol_GetLastFrameTick(void);

/**
 * @brief  最近一帧的代码号和数据首字（int16）。
 * @param  code 输出：代码号；从未收到过时为 0xFF
 * @param  data 输出：数据前两字节按小端拼成的 int16
 * @retval 无
 */
void App_Protocol_GetLastCmd(uint8_t *code, int16_t *data);

#endif /* APP_PROTOCOL_H */
