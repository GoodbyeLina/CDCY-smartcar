/***********************************************
 * car_protocol : 串口帧格式 / 代码号 / 解析状态机（上位机与下位机共用）
 *
 * 一帧：
 *   +------+------+------+-----------+------+------+
 *   | 0x68 | LEN  | CODE | DATA[n]   | SUM  | 0x16 |
 *   +------+------+------+-----------+------+------+
 *
 *   LEN = 1 + n（代码号 + 数据内容 的字节数）
 *   SUM = (CODE + DATA...) & 0xFF
 *
 * 约定：
 *   - 全部小端；int16 字段低字节在前。
 *   - 主机每发一条命令，小车立即回一帧 ACK：代码号与命令相同，
 *     数据 1 字节，CAR_ACK_OK(0x00) 表示已执行，CAR_ACK_FAIL(0xFF) 表示被拒。
 *   - 只有"收全且校验通过"的帧才会让 CarProto_FeedByte 返回 1，
 *     校验失败的帧只累加计数并重新等帧头。
 *
 * ★ 2026-11-22 重建说明：
 *   本文件原属工程上层 ..\include（与上位机共用的协议层），随货压缩包里没有。
 *   这里依据 car_driver v1.1.2 全部调用点（APP/app_protocol.c、app_debug.c、
 *   app_tasks.c）与 _build_output\car_protocol.crf 的符号表重建。
 *   0x0A/0x0C/0x0F/0x20/0x21/0x22/0x30/0x8A 有源码注释作证；
 *   0x00–0x09 按 Protocol_Dispatch 的 switch 顺序推定。
 *   若拿到原版 include\ 目录，请直接覆盖本文件，不要以重建版为准。
 ***********************************************/

#ifndef CAR_PROTOCOL_H
#define CAR_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 帧格式常量 ---------------- */
#define CAR_PROTO_HEAD          0x68U   /* 帧头 */
#define CAR_PROTO_TAIL          0x16U   /* 帧尾 */
#define CAR_PROTO_MAX_DATA      48U     /* 单帧数据区上限 */
#define CAR_PROTO_MIN_LEN       1U      /* LEN 最小：只有代码号 */
#define CAR_PROTO_MAX_LEN       (1U + CAR_PROTO_MAX_DATA)
#define CAR_PROTO_MIN_FRAME     (CAR_PROTO_MIN_LEN + 4U)   /* 头+len+…+sum+尾 */
#define CAR_PROTO_MAX_FRAME     (CAR_PROTO_MAX_LEN + 4U)

/* ---------------- 代码号：主机 -> 小车 ---------------- */
#define CAR_CODE_ARM            0x00U   /* 解除急停（重新使能运动） */
#define CAR_CODE_FORWARD        0x01U   /* 前进，数据 = 速度 mm/s，0 = 默认 */
#define CAR_CODE_BACKWARD       0x02U   /* 后退，数据 = 速度 mm/s，0 = 默认 */
#define CAR_CODE_LEFT           0x03U   /* 左转，数据 = 角速度 mrad/s（阿克曼为前轮转角 mrad） */
#define CAR_CODE_RIGHT          0x04U   /* 右转，同上取负 */
#define CAR_CODE_STOP           0x05U   /* 停车，无数据 */
#define CAR_CODE_SPIN_L         0x06U   /* 原地左转，数据 = 角速度 mrad/s */
#define CAR_CODE_SPIN_R         0x07U   /* 原地右转，同上取负 */
#define CAR_CODE_VECTOR         0x08U   /* 矢量运动：vx, vy, wz 各 int16 */
#define CAR_CODE_SET_DEF        0x09U   /* 设默认量：vx(int16) + wz(int16) */
#define CAR_CODE_QUERY          0x0AU   /* 查询状态，回 0x8A */
#define CAR_CODE_ESTOP          0x0BU   /* 急停，无数据 */
#define CAR_CODE_RAW            0x0CU   /* 直接给 PWM（调试），int16 */
#define CAR_CODE_SET_SPEED      0x0DU   /* 只改线速度（带符号），int16 */
#define CAR_CODE_SET_TURN       0x0EU   /* 只改角速度（带符号），int16 */
#define CAR_CODE_SET_TLM        0x0FU   /* 遥测开关位图，1 字节 */

/* 主机可下发命令的最大代码号（超过即为未知代码） */
#define CAR_CODE_HOST_MAX       0x0FU

/* ---------------- 代码号：小车 -> 主机 ---------------- */
#define CAR_CODE_HB             0x20U   /* 心跳，16 字节 CarTlm_Heartbeat */
#define CAR_CODE_MOT            0x21U   /* 电机，34 字节 CarTlm_Motor */
#define CAR_CODE_DRIVE          0x22U   /* 驱动设定，6 字节 CarTlm_Drive */
#define CAR_CODE_INFO           0x30U   /* ASCII 文本，分帧发送 */
#define CAR_CODE_STATUS         0x8AU   /* 状态应答，CarTlm_Status */

/* ---------------- 遥测开关位（0x0F 命令的位图） ---------------- */
#define CAR_TLM_BIT_HB          0x01U   /* 位 0：0x20 心跳帧 */
#define CAR_TLM_BIT_MOT         0x02U   /* 位 1：0x21 电机帧 */
#define CAR_TLM_BIT_DRIVE       0x04U   /* 位 2：0x22 驱动帧 */
#define CAR_TLM_BIT_ALL         0x07U   /* 三位全开（上电默认） */

/* ---------------- 运动方向（0x22 帧的 motion 字段） ---------------- */
#define CAR_MOTION_STOP         0U
#define CAR_MOTION_FORWARD      1U
#define CAR_MOTION_BACKWARD     2U

/* ---------------- ACK 结果 ---------------- */
#define CAR_ACK_OK              0x00U   /* 已执行 */
#define CAR_ACK_FAIL            0xFFU   /* 被拒绝 */

/* ---------------- 解析状态 ---------------- */
typedef enum
{
    CAR_PARSE_WAIT_HEAD = 0,   /* 等帧头 */
    CAR_PARSE_WAIT_LEN,        /* 等长度 */
    CAR_PARSE_WAIT_CODE,       /* 等代码号 */
    CAR_PARSE_WAIT_DATA,       /* 收数据区 */
    CAR_PARSE_WAIT_SUM,        /* 等校验和 */
    CAR_PARSE_WAIT_TAIL        /* 等帧尾 */
} CarProto_State;

/* ---------------- 解析器 ---------------- */
typedef struct
{
    CarProto_State state;      /* 当前状态，等于 CAR_PARSE_WAIT_HEAD 表示空闲 */
    uint8_t  len;              /* 本帧 LEN */
    uint8_t  code;             /* 本帧代码号 */
    uint8_t  data_len;         /* 本帧数据字节数 = len - 1 */
    uint8_t  data_index;       /* 数据区已收字节数 */
    uint8_t  sum;              /* 累加校验 */
    uint8_t  data[CAR_PROTO_MAX_DATA];

    uint32_t okCount;          /* 完整合法帧数 */
    uint32_t badSumCount;      /* 校验和错 */
    uint32_t badTailCount;     /* 帧尾错 */
    uint32_t badLenCount;      /* 长度非法 */
} CarProto_Parser;

/* ---------------- 组帧结果 ---------------- */
typedef struct
{
    uint8_t      buf[CAR_PROTO_MAX_FRAME];
    unsigned int len;          /* 0 = 组帧失败（数据超长） */
} CarProto_FrameBuf;

/* ---------------- 接口 ---------------- */
void            CarProto_Init(CarProto_Parser *parser);
void            CarProto_ResetState(CarProto_Parser *parser);
int             CarProto_FeedByte(CarProto_Parser *parser, uint8_t byte);
uint8_t         CarProto_Code(const CarProto_Parser *parser);
const uint8_t  *CarProto_Data(const CarProto_Parser *parser);
uint8_t         CarProto_DataLen(const CarProto_Parser *parser);
CarProto_FrameBuf CarProto_MakeFrame(uint8_t code, const uint8_t *data,
                                    unsigned int data_len);
int             CarProto_ExpectedDataLen(uint8_t code);

#ifdef __cplusplus
}
#endif

#endif /* CAR_PROTOCOL_H */
