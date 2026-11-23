/***********************************************
 * car_protocol.c : 帧组包 + 解析状态机（上位机与下位机共用）
 *
 * 逐字节喂：中断里只把字节丢进来，状态机自己找帧头、收长度、
 * 累加校验、对帧尾；只有完整合法的一帧才返回 1。
 * 半帧卡住由调用方按超时调用 CarProto_ResetState() 复位。
 *
 * ★ 2026-11-22 重建版，依据见 car_protocol.h 顶部说明。
 ***********************************************/

#include <stddef.h>

#include "car_protocol.h"

/**
 * @brief  清状态机与统计计数，并置为空闲。
 * @param  parser 解析器
 * @retval 无
 */
void CarProto_Init(CarProto_Parser *parser)
{
    unsigned int i;

    if (parser == NULL)
    {
        return;
    }

    parser->state      = CAR_PARSE_WAIT_HEAD;
    parser->len        = 0U;
    parser->code       = 0U;
    parser->data_len   = 0U;
    parser->data_index = 0U;
    parser->sum        = 0U;

    for (i = 0U; i < CAR_PROTO_MAX_DATA; i++)
    {
        parser->data[i] = 0U;
    }

    parser->okCount     = 0U;
    parser->badSumCount = 0U;
    parser->badTailCount = 0U;
    parser->badLenCount = 0U;
}

/**
 * @brief  只复位状态机，保留统计计数（半帧超时用）。
 * @param  parser 解析器
 * @retval 无
 */
void CarProto_ResetState(CarProto_Parser *parser)
{
    if (parser == NULL)
    {
        return;
    }

    parser->state      = CAR_PARSE_WAIT_HEAD;
    parser->len        = 0U;
    parser->code       = 0U;
    parser->data_len   = 0U;
    parser->data_index = 0U;
    parser->sum        = 0U;
}

/**
 * @brief  喂入一个字节。
 * @param  parser 解析器
 * @param  byte   收到的字节
 * @retval 1 = 收全一帧且校验通过；0 = 还没收全 / 本字节被丢弃
 */
int CarProto_FeedByte(CarProto_Parser *parser, uint8_t byte)
{
    if (parser == NULL)
    {
        return 0;
    }

    switch (parser->state)
    {
        case CAR_PARSE_WAIT_HEAD:
            if (byte == (uint8_t)CAR_PROTO_HEAD)
            {
                parser->state = CAR_PARSE_WAIT_LEN;
            }
            break;

        case CAR_PARSE_WAIT_LEN:
            if ((byte < (uint8_t)CAR_PROTO_MIN_LEN) || (byte > (uint8_t)CAR_PROTO_MAX_LEN))
            {
                parser->badLenCount++;
                parser->state = CAR_PARSE_WAIT_HEAD;
                break;
            }

            parser->len        = byte;
            parser->data_len   = (uint8_t)(byte - 1U);
            parser->data_index = 0U;
            parser->sum        = 0U;
            parser->state      = CAR_PARSE_WAIT_CODE;
            break;

        case CAR_PARSE_WAIT_CODE:
            parser->code  = byte;
            parser->sum   = byte;
            parser->state = (parser->data_len > 0U) ? CAR_PARSE_WAIT_DATA
                                                    : CAR_PARSE_WAIT_SUM;
            break;

        case CAR_PARSE_WAIT_DATA:
            parser->data[parser->data_index] = byte;
            parser->data_index++;
            parser->sum = (uint8_t)(parser->sum + byte);

            if (parser->data_index >= parser->data_len)
            {
                parser->state = CAR_PARSE_WAIT_SUM;
            }
            break;

        case CAR_PARSE_WAIT_SUM:
            if (byte != parser->sum)
            {
                parser->badSumCount++;
                parser->state = CAR_PARSE_WAIT_HEAD;
                break;
            }
            parser->state = CAR_PARSE_WAIT_TAIL;
            break;

        case CAR_PARSE_WAIT_TAIL:
            if (byte != (uint8_t)CAR_PROTO_TAIL)
            {
                parser->badTailCount++;
                parser->state = CAR_PARSE_WAIT_HEAD;
                break;
            }

            parser->okCount++;
            parser->state = CAR_PARSE_WAIT_HEAD;
            return 1;

        default:
            parser->state = CAR_PARSE_WAIT_HEAD;
            break;
    }

    return 0;
}

uint8_t CarProto_Code(const CarProto_Parser *parser)
{
    return (parser == NULL) ? 0U : parser->code;
}

const uint8_t *CarProto_Data(const CarProto_Parser *parser)
{
    return (parser == NULL) ? NULL : parser->data;
}

uint8_t CarProto_DataLen(const CarProto_Parser *parser)
{
    return (parser == NULL) ? 0U : parser->data_len;
}

/**
 * @brief  按帧格式组一帧。
 * @param  code     代码号
 * @param  data     数据内容，可为 NULL（此时 data_len 必须为 0）
 * @param  data_len 数据字节数
 * @retval 组好的帧；len == 0 表示失败（数据超长或指针为空）
 */
CarProto_FrameBuf CarProto_MakeFrame(uint8_t code, const uint8_t *data,
                                    unsigned int data_len)
{
    CarProto_FrameBuf frame;
    unsigned int i;
    uint8_t      sum;

    frame.len = 0U;

    if (data_len > CAR_PROTO_MAX_DATA)
    {
        return frame;
    }
    if ((data == NULL) && (data_len > 0U))
    {
        return frame;
    }

    frame.buf[0] = (uint8_t)CAR_PROTO_HEAD;
    frame.buf[1] = (uint8_t)(1U + data_len);
    frame.buf[2] = code;

    sum = code;
    for (i = 0U; i < data_len; i++)
    {
        frame.buf[3U + i] = data[i];
        sum = (uint8_t)(sum + data[i]);
    }

    frame.buf[3U + data_len] = sum;
    frame.buf[4U + data_len] = (uint8_t)CAR_PROTO_TAIL;
    frame.len                = data_len + 5U;

    return frame;
}

/**
 * @brief  查某代码号的合法数据长度。
 * @param  code 代码号
 * @retval >= 0 = 该命令应有的数据字节数；-1 = 未知代码
 */
int CarProto_ExpectedDataLen(uint8_t code)
{
    switch (code)
    {
        case CAR_CODE_ARM:       return 0;
        case CAR_CODE_FORWARD:   return 2;
        case CAR_CODE_BACKWARD:  return 2;
        case CAR_CODE_LEFT:      return 2;
        case CAR_CODE_RIGHT:     return 2;
        case CAR_CODE_STOP:      return 0;
        case CAR_CODE_SPIN_L:    return 2;
        case CAR_CODE_SPIN_R:    return 2;
        case CAR_CODE_VECTOR:    return 6;
        case CAR_CODE_SET_DEF:   return 4;
        case CAR_CODE_QUERY:     return 0;
        case CAR_CODE_ESTOP:     return 0;
        case CAR_CODE_RAW:       return 2;
        case CAR_CODE_SET_SPEED: return 2;
        case CAR_CODE_SET_TURN:  return 2;
        case CAR_CODE_SET_TLM:   return 1;
        default:                 return -1;
    }
}
