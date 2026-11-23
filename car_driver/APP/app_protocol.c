/***********************************************
 * car_driver : 协议解析与分发（APP 层）
 * Port  : USART3, 115200-8-N-1
 *
 * 数据流：
 *   USART3 中断 -> 环形缓冲区 -> App_Protocol_Task
 *                -> CarProto_FeedByte（car_protocol.c，与上位机共用）
 *                -> 校验通过 -> Protocol_Dispatch（按代码号执行）
 *
 * 一帧：68 | LEN | CODE | DATA[n] | SUM | 16
 *   LEN = 1 + n；SUM = (CODE + DATA...) & 0xFF
 *
 * 注意：帧格式常量、代码号、状态机都不在本文件定义，
 *       统一在 ..\include\car_protocol.h / .c 里，上位机编译同一份。
 ***********************************************/

#include "app_protocol.h"
#include "app_config.h"
#include "app_motion.h"
#include "bsp_chassis.h"
#include "bsp_uart3.h"
#include "car_telemetry.h"

#include "FreeRTOS.h"
#include "task.h"

/* 解析器：状态机在 car_protocol.c，这里只负责喂字节、收结果 */
static CarProto_Parser s_parser;
static TickType_t      s_last_byte_tick;   /* 最后一字节的时间，用于半帧超时 */

static TaskHandle_t    s_task;             /* 接收任务句柄 */

/* 协议层之外的错误计数（帧本身的问题都在 s_parser 里统计） */
static uint32_t s_bad_code;
static uint32_t s_bad_len;     /* 代码号对、但数据长度不符 */

/* 链路状态，给 Link 任务和调试用 */
static volatile uint8_t  s_status_req;     /* 0x0A 查询请求，等待链路任务发出 */
static volatile uint8_t  s_seen_frame;
static volatile uint32_t s_last_frame_tick;
static uint8_t           s_last_code = 0xFFU;
static int16_t           s_last_data;

/* 0x01/0x02/0x03/0x04 数据为 0 时使用的默认速度 */
static int16_t s_default_vx_mm_s;
static int16_t s_default_wz_mrad_s;

/*
 * 上位机下发的驱动设定（0x22 驱动状态帧的内容）。
 * 每次收到命令后、以及每帧发出前按运动层当前指令刷新一次，
 * 所以它始终等于"当前生效的设定"，不是一份可能过期的副本。
 */
static CarTlm_Drive s_drive;

/* 遥测帧开关（0x0F 命令保存的位图）：位 0/1/2 = 0x20 / 0x21 / 0x22 */
static uint8_t s_tlm_mask;

/**
 * @brief  从两字节小端拼出 int16。
 * @param  p 指向 2 字节数据
 * @retval 有符号结果
 */
static int16_t Protocol_ReadInt16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * @brief  取绝对值并防止 -32768 溢出。
 * @param  value 输入
 * @retval 绝对值（最大 32767）
 */
static int16_t Protocol_Abs16(int16_t value)
{
    int32_t wide = value;

    if (wide < 0)
    {
        wide = -wide;
    }
    if (wide > 32767)
    {
        wide = 32767;
    }
    return (int16_t)wide;
}

/**
 * @brief  按帧格式组帧并通过 USART3 发出。
 * @param  code     代码号
 * @param  data     数据内容，可为 NULL
 * @param  data_len 数据字节数
 * @retval 无
 */
void App_Protocol_SendFrame(uint8_t code, const uint8_t *data, uint8_t data_len)
{
    CarProto_FrameBuf frame;

    frame = CarProto_MakeFrame(code, data, (unsigned int)data_len);
    if (frame.len > 0U)
    {
        BSP_UART3_Send(frame.buf, (uint16_t)frame.len);
    }
}

/**
 * @brief  发送状态帧 0x8A：vx,vy,wz + armed + fault + err。
 * @param  无
 * @retval 无
 */
void App_Protocol_SendStatus(void)
{
    CarTlm_Status st;

    App_Motion_Get(&st.vx, &st.vy, &st.wz);
    st.armed = App_Motion_IsArmed();
    st.fault = App_Motion_GetFault();
    st.err   = (uint16_t)App_Protocol_GetErrorCount();

    App_Protocol_SendFrame(APP_PROTO_CODE_STATUS, (const uint8_t *)&st,
                           (uint8_t)sizeof(st));
}

/**
 * @brief  按运动层当前指令刷新驱动设定结构体。
 * @param  无
 * @retval 无
 * @note   前进/后退状态直接由线速度符号推出：正 = 前进，负 = 后退，0 = 停止。
 *         速度值取运动层限幅后的实际值，而不是主机原始下发值。
 */
static void Protocol_UpdateDriveState(void)
{
    int16_t vx;
    int16_t vy;
    int16_t wz;

    App_Motion_Get(&vx, &vy, &wz);

    s_drive.speed    = vx;
    s_drive.turn     = wz;
    s_drive.reserved = 0U;

    if (vx > 0)
    {
        s_drive.motion = (uint8_t)CAR_MOTION_FORWARD;
    }
    else if (vx < 0)
    {
        s_drive.motion = (uint8_t)CAR_MOTION_BACKWARD;
    }
    else
    {
        s_drive.motion = (uint8_t)CAR_MOTION_STOP;
    }
}

/**
 * @brief  发送驱动状态帧 0x22：前进/后退状态 + 线速度 + 角速度。
 * @param  无
 * @retval 无
 */
void App_Protocol_SendDrive(void)
{
    Protocol_UpdateDriveState();

    App_Protocol_SendFrame(CAR_CODE_DRIVE, (const uint8_t *)&s_drive,
                           (uint8_t)sizeof(s_drive));
}

/**
 * @brief  设置遥测帧开关。
 * @param  mask CAR_TLM_BIT_xxx 按位或
 * @retval 无
 * @note   只保留已定义的 3 位，其余位直接丢弃，避免以后加位时旧主机误开新帧。
 */
void App_Protocol_SetTelemetryMask(uint8_t mask)
{
    s_tlm_mask = (uint8_t)(mask & CAR_TLM_BIT_ALL);
}

/**
 * @brief  读取遥测帧开关。
 * @param  无
 * @retval CAR_TLM_BIT_xxx 按位或
 */
uint8_t App_Protocol_GetTelemetryMask(void)
{
    return s_tlm_mask;
}

/**
 * @brief  按代码号执行一条已校验的指令。
 * @param  code     代码号
 * @param  data     数据内容
 * @param  data_len 数据字节数
 * @retval 0 = 已执行；-1 = 被拒绝（未知代码/长度不符/底盘不支持）
 */
static int Protocol_Dispatch(uint8_t code, const uint8_t *data, uint8_t data_len)
{
    int16_t vx;
    int16_t vy;
    int16_t wz;
    int16_t value;
    int     expected;
    int     ok = 0;

    expected = CarProto_ExpectedDataLen(code);
    if (expected < 0)
    {
        App_Motion_SetFault(APP_FAULT_BAD_CODE);
        s_bad_code++;
        return -1;
    }

    if ((int)data_len != expected)
    {
        App_Motion_SetFault(APP_FAULT_BAD_LEN);
        s_bad_len++;
        return -1;
    }

    switch (code)
    {
        case APP_PROTO_CODE_ARM:
            App_Motion_Arm();
            break;

        case APP_PROTO_CODE_FORWARD:
            value = Protocol_Abs16(Protocol_ReadInt16(data));
            if (value == 0)
            {
                value = s_default_vx_mm_s;
            }
            App_Motion_Apply(value, 0, 0);
            break;

        case APP_PROTO_CODE_BACKWARD:
            value = Protocol_Abs16(Protocol_ReadInt16(data));
            if (value == 0)
            {
                value = s_default_vx_mm_s;
            }
            App_Motion_Apply(-value, 0, 0);
            break;

        case APP_PROTO_CODE_LEFT:
            value = Protocol_Abs16(Protocol_ReadInt16(data));
            if (value == 0)
            {
                /* 阿克曼下这个值是前轮转角（mrad），不是角速度，用专用的默认值 */
                value = (BSP_Chassis_IsAckermann() != 0U)
                      ? (int16_t)APP_DEFAULT_STEER_MRAD
                      : s_default_wz_mrad_s;
            }
            App_Motion_Get(&vx, &vy, &wz);
            App_Motion_Apply(vx, 0, value);
            break;

        case APP_PROTO_CODE_RIGHT:
            value = Protocol_Abs16(Protocol_ReadInt16(data));
            if (value == 0)
            {
                value = (BSP_Chassis_IsAckermann() != 0U)
                      ? (int16_t)APP_DEFAULT_STEER_MRAD
                      : s_default_wz_mrad_s;
            }
            App_Motion_Get(&vx, &vy, &wz);
            App_Motion_Apply(vx, 0, -value);
            break;

        case APP_PROTO_CODE_STOP:
            App_Motion_Stop();
            break;

        case APP_PROTO_CODE_SPIN_L:
            /* 原地转只对停着的车有意义：还在走就拒绝，避免"拧着走" */
            App_Motion_Get(&vx, &vy, &wz);
            if (vx != 0)
            {
                App_Motion_SetFault(APP_FAULT_NOTSUP);
                ok = -1;
                break;
            }
            value = Protocol_Abs16(Protocol_ReadInt16(data));
            if (value == 0)
            {
                value = s_default_wz_mrad_s;
            }
            App_Motion_Apply(0, 0, value);
            break;

        case APP_PROTO_CODE_SPIN_R:
            App_Motion_Get(&vx, &vy, &wz);
            if (vx != 0)
            {
                App_Motion_SetFault(APP_FAULT_NOTSUP);
                ok = -1;
                break;
            }
            value = Protocol_Abs16(Protocol_ReadInt16(data));
            if (value == 0)
            {
                value = s_default_wz_mrad_s;
            }
            App_Motion_Apply(0, 0, -value);
            break;

        case APP_PROTO_CODE_VECTOR:
            vx = Protocol_ReadInt16(data);
            vy = Protocol_ReadInt16(data + 2);
            wz = Protocol_ReadInt16(data + 4);
            App_Motion_Apply(vx, vy, wz);
            break;

        case APP_PROTO_CODE_SET_DEF:
            value = Protocol_Abs16(Protocol_ReadInt16(data));
            if (value > APP_MAX_VX_MM_S)
            {
                value = APP_MAX_VX_MM_S;
            }
            s_default_vx_mm_s = value;

            value = Protocol_Abs16(Protocol_ReadInt16(data + 2));
            if (value > APP_MAX_WZ_MRAD_S)
            {
                value = APP_MAX_WZ_MRAD_S;
            }
            s_default_wz_mrad_s = value;
            break;

        case APP_PROTO_CODE_QUERY:
            s_status_req = 1U;
            break;

        case APP_PROTO_CODE_ESTOP:
            App_Motion_EStop();
            break;

        case APP_PROTO_CODE_RAW:
            App_Motion_SetRaw(Protocol_ReadInt16(data));
            break;

        case APP_PROTO_CODE_SET_SPEED:
            /* 带符号原值下发：正 = 前进，负 = 后退；角速度保持当前值 */
            App_Motion_Get(&vx, &vy, &wz);
            App_Motion_Apply(Protocol_ReadInt16(data), 0, wz);
            break;

        case APP_PROTO_CODE_SET_TURN:
            /* 带符号原值下发：正 = 逆时针，负 = 顺时针；线速度保持当前值 */
            App_Motion_Get(&vx, &vy, &wz);
            App_Motion_Apply(vx, 0, Protocol_ReadInt16(data));
            break;

        case APP_PROTO_CODE_SET_TLM:
            /* 1 字节位图：位 0/1/2 = 0x20 / 0x21 / 0x22，置 1 = 发送 */
            App_Protocol_SetTelemetryMask(data[0]);
            break;

        default:
            App_Motion_SetFault(APP_FAULT_BAD_CODE);
            s_bad_code++;
            ok = -1;
            break;
    }

    return ok;
}

/**
 * @brief  一帧收全且校验通过后的收尾：记录、分发。
 * @param  无
 * @retval 无
 * @note   帧内容直接从共享解析器里取，不再自己维护 payload 缓冲。
 */
static void Protocol_OnFrameDone(void)
{
    uint8_t        code;
    const uint8_t *data;
    uint8_t        n;

    s_seen_frame      = 1U;
    s_last_frame_tick = (uint32_t)xTaskGetTickCount();
    App_Motion_ClearFaultBit(APP_FAULT_TIMEOUT);

    code = CarProto_Code(&s_parser);
    data = CarProto_Data(&s_parser);
    n    = (uint8_t)CarProto_DataLen(&s_parser);

    s_last_code = code;
    s_last_data = (n >= 2U) ? Protocol_ReadInt16(data) : (int16_t)0;

    /* 执行结果立即回一帧：代码号与命令相同，数据 1 字节（0 成功 / 0xFF 失败） */
    {
        uint8_t ack = (Protocol_Dispatch(code, data, n) == 0) ? (uint8_t)CAR_ACK_OK
                                                              : (uint8_t)CAR_ACK_FAIL;

        /* 命令已生效，顺手刷新一份设定，供 0x22 帧随时取用 */
        Protocol_UpdateDriveState();
        App_Protocol_SendFrame(code, &ack, 1U);
    }
}

/**
 * @brief  单字节喂入解析状态机。
 * @param  byte 字节
 * @retval 无
 * @note   状态机实现在 car_protocol.c，和上位机编译的是同一份。
 */
static void App_Protocol_FeedByte(uint8_t byte)
{
    s_last_byte_tick = xTaskGetTickCount();

    if (CarProto_FeedByte(&s_parser, byte) != 0)
    {
        Protocol_OnFrameDone();
    }
}

/**
 * @brief  多字节喂入解析状态机。
 * @param  buf 数据指针
 * @param  len 字节数
 * @retval 无
 */
static void App_Protocol_Feed(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == NULL) || (len == 0U))
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        App_Protocol_FeedByte(buf[i]);
    }
}

/**
 * @brief  把环形缓冲区里的数据取空并解析。
 * @param  无
 * @retval 无
 */
void App_Protocol_Process(void)
{
    uint8_t  chunk[32];
    uint16_t count;

    for (;;)
    {
        count = BSP_UART3_Read(chunk, (uint16_t)sizeof(chunk));
        if (count == 0U)
        {
            break;
        }

        App_Protocol_Feed(chunk, count);
    }
}

/**
 * @brief  接收任务：取数据、解析、半帧超时复位。
 * @param  pv_parameters 未使用
 * @retval 无
 * @note   先取再等通知，避免"取空后新字节已在缓冲区、通知也给了"导致白等一轮；
 *         APP_PROTOCOL_GAP_MS 超时用于丢弃半截帧，防止垃圾数据卡住状态机。
 */
void App_Protocol_Task(void *pv_parameters)
{
    (void)pv_parameters;

    for (;;)
    {
        App_Protocol_Process();

        if ((s_parser.state != CAR_PARSE_WAIT_HEAD) &&
            ((xTaskGetTickCount() - s_last_byte_tick) > pdMS_TO_TICKS(APP_PROTOCOL_GAP_MS)))
        {
            CarProto_ResetState(&s_parser);   /* 半帧超时，回到等帧头 */
        }

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(APP_PROTOCOL_GAP_MS));
    }
}

/**
 * @brief  初始化协议模块：清状态、建任务、注册通知。
 * @param  无
 * @retval 无
 */
void App_Protocol_Init(void)
{
    CarProto_Init(&s_parser);
    s_last_byte_tick = 0U;

    s_bad_code        = 0U;
    s_bad_len         = 0U;
    s_status_req      = 0U;
    s_seen_frame      = 0U;
    s_last_frame_tick = 0U;
    s_last_code       = 0xFFU;
    s_last_data       = 0;

    s_default_vx_mm_s   = APP_DEFAULT_VX_MM_S;
    s_default_wz_mrad_s = APP_DEFAULT_WZ_MRAD_S;

    s_drive.motion   = (uint8_t)CAR_MOTION_STOP;
    s_drive.reserved = 0U;
    s_drive.speed    = 0;
    s_drive.turn     = 0;

    s_tlm_mask = (uint8_t)CAR_TLM_BIT_ALL;   /* 上电默认三帧全发 */

    s_task = NULL;
    (void)xTaskCreate(App_Protocol_Task,
                      "PROTO",
                      APP_TASK_STACK_PROTOCOL,
                      NULL,
                      APP_TASK_PRIO_PROTOCOL,
                      &s_task);

    if (s_task != NULL)
    {
        BSP_UART3_SetNotifyTask(s_task);
    }
}

/**
 * @brief  处理挂起的查询请求。
 * @param  无
 * @retval 1 = 已发送状态帧；0 = 没有请求
 */
uint8_t App_Protocol_ServiceStatus(void)
{
    if (s_status_req == 0U)
    {
        return 0U;
    }

    s_status_req = 0U;
    App_Protocol_SendStatus();
    return 1U;
}

uint32_t App_Protocol_GetFrameCount(void)
{
    return (uint32_t)s_parser.okCount;
}

uint32_t App_Protocol_GetErrorCount(void)
{
    return (uint32_t)(s_parser.badSumCount + s_parser.badTailCount
                    + s_parser.badLenCount)
         + s_bad_code
         + s_bad_len
         + BSP_UART3_GetOverflowCount();
}

uint32_t App_Protocol_GetSumErrorCount(void)
{
    return (uint32_t)s_parser.badSumCount;
}

uint32_t App_Protocol_GetTailErrorCount(void)
{
    return (uint32_t)s_parser.badTailCount;
}

uint32_t App_Protocol_GetLenErrorCount(void)
{
    return (uint32_t)s_parser.badLenCount;
}

void App_Protocol_ResetStats(void)
{
    CarProto_Init(&s_parser);      /* 帧统计都在共享解析器里 */
    s_bad_code = 0U;
    s_bad_len  = 0U;
}

uint8_t App_Protocol_HasSeenFrame(void)
{
    return s_seen_frame;
}

uint32_t App_Protocol_GetLastFrameTick(void)
{
    return s_last_frame_tick;
}

void App_Protocol_GetLastCmd(uint8_t *code, int16_t *data)
{
    *code = s_last_code;
    *data = s_last_data;
}
