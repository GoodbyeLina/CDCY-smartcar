/***********************************************
 * car_telemetry : 遥测帧结构体（小车 -> 上位机）
 *
 * 所有结构体都按 1 字节对齐直接当帧数据发出去，
 * 所以字段顺序、类型宽度就是线上字节布局，改动等于改协议。
 * 全部小端。
 *
 *   0x8A STATUS   CarTlm_Status   （查询应答）
 *   0x20 HB       CarTlm_Heartbeat 16 B
 *   0x21 MOT      CarTlm_Motor     34 B
 *   0x22 DRIVE    CarTlm_Drive      6 B
 *
 * ★ 2026-11-22 重建版：字节数由 app_debug.c / app_protocol.c 的注释与
 *   SendFrame(... sizeof(st)) 的实参推出，见 car_protocol.h 顶部说明。
 ***********************************************/

#ifndef CAR_TELEMETRY_H
#define CAR_TELEMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 0x8A 状态应答
 *   offset 0  vx      int16  线速度 mm/s
 *   offset 2  vy      int16  横移 mm/s（麦轮/全向有效，阿克曼恒 0）
 *   offset 4  wz      int16  角速度 mrad/s
 *   offset 6  armed   uint8  1 = 已使能运动
 *   offset 7  fault   uint8  故障位图（APP_FAULT_xxx）
 *   offset 8  err     uint16 错误计数
 */
typedef struct
{
    int16_t  vx;
    int16_t  vy;
    int16_t  wz;
    uint8_t  armed;
    uint8_t  fault;
    uint16_t err;
} CarTlm_Status;

/*
 * 0x20 心跳（16 字节）
 *   offset  0 up_s   uint16 开机秒数
 *   offset  2 heap   uint16 剩余堆字节
 *   offset  4 rx     uint32 收到的总字节数
 *   offset  8 ovf    uint32 串口溢出次数
 *   offset 12 ok     uint16 合法帧数
 *   offset 14 bad    uint16 错误帧数
 */
typedef struct
{
    uint16_t up_s;
    uint16_t heap;
    uint32_t rx;
    uint32_t ovf;
    uint16_t ok;
    uint16_t bad;
} CarTlm_Heartbeat;

/*
 * 0x21 电机（34 字节）
 *   offset  0 mode     uint8  底盘型号 CAR_xxx
 *   offset  1 enable   uint8  硬件使能开关
 *   offset  2 armed    uint8  1 = 已使能运动
 *   offset  3 fault    uint8  故障位图
 *   offset  4 vx       int16  mm/s
 *   offset  6 vy       int16  mm/s
 *   offset  8 wz       int16  mrad/s
 *   offset 10 target[4] int16 四个轮子的目标速度 mm/s
 *   offset 18 speed[4]  int16 四个轮子实测速度 mm/s
 *   offset 26 pwm[4]    int16 四个轮子的 PWM 输出
 */
typedef struct
{
    uint8_t mode;
    uint8_t enable;
    uint8_t armed;
    uint8_t fault;
    int16_t vx;
    int16_t vy;
    int16_t wz;
    int16_t target[4];
    int16_t speed[4];
    int16_t pwm[4];
} CarTlm_Motor;

/*
 * 0x22 驱动设定（6 字节）
 *   offset 0 motion   uint8  CAR_MOTION_STOP / FORWARD / BACKWARD
 *   offset 1 reserved uint8  保留，恒 0
 *   offset 2 speed    int16  线速度 mm/s（带符号）
 *   offset 4 turn     int16  角速度 mrad/s（带符号）
 */
typedef struct
{
    uint8_t motion;
    uint8_t reserved;
    int16_t speed;
    int16_t turn;
} CarTlm_Drive;

#ifdef __cplusplus
}
#endif

#endif /* CAR_TELEMETRY_H */
