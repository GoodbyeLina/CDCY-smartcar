#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Firmware version: bump when behaviour changes.
 * 1.1.0 - raw PWM test (0x0C), open-loop power-on run, per-frame ACK
 * 1.1.1 - fix power-on run never executing (tick-0 underflow); gate on PD3
 * 1.1.2 - power-on run disabled again, car stays still until commanded */
#define APP_FW_VERSION_MAJOR    1
#define APP_FW_VERSION_MINOR    1
#define APP_FW_VERSION_PATCH    2

/* Control loop: 100 Hz, 10 ms period */
#define APP_CONTROL_HZ          100
#define APP_CONTROL_PERIOD_MS   10

/* Motion defaults and safety limits */
#define APP_DEFAULT_VX_MM_S     300
#define APP_DEFAULT_WZ_MRAD_S   1000
#define APP_MAX_VX_MM_S         1000
#define APP_MAX_VY_MM_S         800
#define APP_MAX_WZ_MRAD_S       3000

/* AKM（阿克曼）下 0x03/0x04 不带数据时用的前轮转角，单位 mrad（200 ≈ 11.5°）。
 * 这个值在 AKM 里表示"转角"而不是"角速度"，填大了会被机械限位夹到 0.32 rad。 */
#define APP_DEFAULT_STEER_MRAD  200

/* No valid frame for this long -> stop (0 disables) */
#define APP_CMD_TIMEOUT_MS      1000

/* Power-on run: open-loop forward for APP_STARTUP_FORWARD_MS, then stop.
 * DISABLED (1.1.2): the car must stay still until a host command arrives. */
#define APP_STARTUP_FORWARD_ENABLE 0
#define APP_STARTUP_FORWARD_MS     10000
#define APP_STARTUP_TEST_PWM       16800    /* 16800/16800 = 100% duty, all 4 channels */
#define APP_STARTUP_FORWARD_VX     APP_DEFAULT_VX_MM_S

/* Periodic binary status frame on USART3 (0 disables).
 * Disabled in this debug build: text debug lines share USART3 and the binary
 * 0x8A stream would make the terminal unreadable.  Query reply (0x0A) still works. */
#define APP_TEST_TX_PERIOD_MS      0

/* Text debug lines on USART3, every APP_DEBUG_PERIOD_MS.
 * Set APP_DEBUG_ENABLE to 0 (or the period to 0) for silent operation. */
#define APP_DEBUG_ENABLE           1
#define APP_DEBUG_PERIOD_MS        1000

/* Protocol limits */
#define APP_FRAME_MAX_DATA      16
#define APP_FRAME_BUF_SIZE      40
#define APP_RX_RING_SIZE        256
#define APP_PROTOCOL_GAP_MS     20

/* Motor / speed loop */
#define APP_PWM_MAX             16800
#define APP_PWM_LIMIT           16800
#define APP_WHEEL_TARGET_LIMIT  3.5f
#define APP_VELOCITY_KP         700.0f
#define APP_VELOCITY_KI         700.0f

/* Chassis: -1 = read PB1 potentiometer, 0..5 = force a model.
 * Forced to 2 (AKM): rear A/B motors drive, the servo steers the front wheels. */
#define APP_CHASSIS_FORCE       2

/* User LED on PE8.
 * APP_LED_MODE:
 *   0 = heartbeat, toggle every 500 ms (normal firmware behaviour)
 *   1 = force OFF  (diagnostic: drive the LED off and keep re-asserting it)
 *   2 = force ON   (diagnostic: drive the LED on)
 * APP_LED_ACTIVE_HIGH: 1 = high level lights the LED (C30D default),
 *                      0 = low level lights it (inverted hardware). */
#define APP_LED_MODE            0
#define APP_LED_ACTIVE_HIGH     0

/* Servo */
#define APP_SERVO_MIN_US        800
#define APP_SERVO_MAX_US        2200
#define APP_SERVO_CENTER_US     1500
#define APP_SERVO_DUAL_OUTPUT   1

/* Tasks: control must stay the highest priority user task */
#define APP_TASK_PRIO_CONTROL   6
#define APP_TASK_PRIO_PROTOCOL  5
#define APP_TASK_PRIO_LINK      4
#define APP_TASK_PRIO_MONITOR   1
#define APP_TASK_PRIO_DEBUG     1       /* LED 心跳 + 串口心跳打印 */

#define APP_TASK_STACK_CONTROL  256
#define APP_TASK_STACK_PROTOCOL 384
#define APP_TASK_STACK_LINK     256
#define APP_TASK_STACK_MONITOR  128
#define APP_TASK_STACK_DEBUG    256     /* printf 链路较深，128 不够 */

/* Fault bits reported by the status frame */
#define APP_FAULT_NONE          0x00
#define APP_FAULT_BAD_SUM       0x01
#define APP_FAULT_BAD_TAIL      0x02
#define APP_FAULT_BAD_LEN       0x04
#define APP_FAULT_BAD_CODE      0x08
#define APP_FAULT_TIMEOUT       0x10
#define APP_FAULT_NOTSUP        0x20    /* 当前底盘/状态下不支持这条动作 */
#define APP_FAULT_UART_OVR      0x40
#define APP_FAULT_UNKNOWN       0x80

#endif /* APP_CONFIG_H */
