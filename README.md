# CDCY-smartcar

阿克曼转向智能小车的 **STM32 底盘固件**（下位机）。目标平台：**WHEELTEC C30D 主控板 / STM32F407VET6**，FreeRTOS 分层架构，通过串口与上位机（RK3588S / ROS 2）通信。

> 固件版本 **v1.1.2**。上位机侧不需要任何厂商私有协议猜测 —— 本仓库就是协议规范。

---

## 目录结构

```
.
├── car_driver/              Keil MDK 工程（打开 car_driver.uvprojx）
│   ├── APP/                 应用层：协议分发、运动学 + PI 速度环、任务、调试
│   ├── BSP/                 驱动层：电机 PWM、编码器、舵机、串口、底盘识别、GPIO、LED
│   ├── USER/                main.c、中断向量
│   ├── SYSTEM/              delay / sys
│   ├── CORE/                CMSIS 内核
│   ├── FWLIB/               STM32F4 标准外设库
│   └── FreeRTOS/            内核 + 移植层
├── include/                 ★ 与上位机共用的协议层（工程以 ..\include 引用）
│   ├── car_protocol.h/.c    帧格式、解析状态机、组帧
│   └── car_telemetry.h      遥测结构体（心跳 / 电机 / 驱动反馈 / 状态）
└── docs/
    └── 源码导读.md          逐目录解读、协议全表、已知问题、编译说明
```

`car_driver/` 与 `include/` 必须保持当前相对位置 —— 工程通过 `..\include` 找到协议层。

---

## 硬件

| 项 | 参数 |
|---|---|
| 车型 | 阿克曼（电位器 PB1 选型，代码内可强制为 AKM） |
| 主控 | STM32F407VET6，168 MHz |
| 电机驱动 | AT8236，4 路通道（本车用 A/B 两路，左右后轮） |
| 编码器 | 霍尔 13 线 × 减速比 30 × 4 倍频 = **1560 计数/圈** |
| 轮胎周长 | 0.065 m × π |
| 轴距 / 轮距 | 0.144 m / 0.162 m |
| 转向舵机 | PB15 / TIM12_CH2（50 Hz），脉宽 800–2200 µs，中位 1500 µs |
| 上位机串口 | **USART3，PD8=TX / PD9=RX，115200 8-N-1** |
| 使能开关 | PD3（高电平使能）；急停后需 `0x00 ARM` 恢复 |

---

## 编译

需要 **Keil MDK-ARM**（本工程用 5.29 + ARMCC V5.06 update 6 验证）+ STM32F4xx DFP 芯片包。

```bash
# 命令行构建（产物在 car_driver/_build_output/）
UV4.exe -b car_driver/car_driver.uvprojx -j0 -o build.log
```

实测结果：`Code=26196 RO-data=776 RW-data=276 ZI-data=22772`，**0 Error / 0 Warning**。

---

## 串口协议

### 帧格式

```
0x68 | LEN | CODE | DATA[n] | SUM | 0x16
        └ LEN = 1 + n（CODE 之后、SUM 之前的字节数）
              SUM = (CODE + DATA…) & 0xFF
```

每条合法命令都会回 **ACK**：`0x68 | 0x02 | CODE | 0x00 | SUM | 0x16`（0x00 成功 / 0xFF 失败）。
半帧超时 20 ms 后自动复位解析状态。

### 命令表

| CODE | 命令 | DATA | 说明 |
|---|---|---|---|
| 0x00 | ARM | — | 解除急停 |
| 0x01 | FORWARD | int16 | 前进，单位 mm/s |
| 0x02 | BACKWARD | int16 | 后退 |
| 0x03 | LEFT | int16 | 阿克曼：**前轮转角 mrad** |
| 0x04 | RIGHT | int16 | 同上 |
| 0x05 | STOP | — | 停车 |
| 0x06 | SPIN_L | int16 | 仅 vx=0 时允许 |
| 0x07 | SPIN_R | int16 | 同上 |
| 0x08 | VECTOR | 3×int16 | vx / vy / wz（wz 在 AKM 下被当前轮转角） |
| 0x09 | SET_DEF | 4 B | 设置默认值 |
| 0x0A | QUERY | — | 查询状态 |
| 0x0B | ESTOP | — | 急停 |
| 0x0C | RAW | int16 | 四轮直接给 PWM（调试） |
| 0x0D | SET_SPEED | int16 | 设定线速度 mm/s |
| 0x0E | SET_TURN | int16 | 设定前轮转角 mrad |
| 0x0F | SET_TLM | uint8 | 遥测位图：bit0=心跳 bit1=电机 bit2=驱动 |
| 0x20 | HB | 16 B | 心跳（运行秒数 / 堆 / 收字节 / 溢出 / 好帧 / 坏帧） |
| 0x21 | MOT | 34 B | 电机反馈（模式 / 使能 / armed / 故障 / vx,vy,wz / 4×目标·速度·PWM） |
| 0x22 | DRIVE | 6 B | 驱动反馈（运动方向 / 速度 / 转角） |
| 0x30 | INFO | ASCII | 开机横幅等文本（按 MAX_DATA 分帧） |
| 0x8A | STATUS | — | 状态应答（vx,vy,wz,armed,fault,err） |

### 控制频率与限幅

- 控制环 **100 Hz**；`APP_DEFAULT_VX_MM_S 300`、`APP_MAX_VX_MM_S 1000`、`APP_MAX_WZ_MRAD_S 3000`
- PWM 满量程 16800（10 kHz），PI 参数 `KP = KI = 700`

---

## ⚠️ 已知问题（详见 `docs/源码导读.md`）

1. **JTAG 未关闭**：PA15/PB3/PB4 被用作 TIM2/TIM3 编码器输入，但工程里没有 SWJ 重映射代码 —— 复位后默认 JTAG 全开时，编码器 A、B 两路不计数，闭环退化为开环。需要时在 `main()` 前加 `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);`。
2. **阿克曼语义**：`0x03/0x04` 的数值是**前轮转角（mrad）**，不是角速度；`0x06 VECTOR` 的 `wz` 也被当转角（内部 clamp −0.49…0.32 rad）。ROS `/cmd_vel` 桥接需自行换算 `steer = atan(L·ω/v)`。
3. **上电默认 `armed = 1`**：只要使能开关 ON，收到命令就会动。做台架实验务必**车轮悬空**。
4. `include/` 下的三个文件是依据 v1.1.2 全部调用点 + `.crf` 符号表**重建**的；`0x00–0x09` 代码号按分发顺序推定（其余有注释作证）。若手上有原版 `include/`，直接覆盖即可。

---

## License

[Apache License 2.0](LICENSE)
