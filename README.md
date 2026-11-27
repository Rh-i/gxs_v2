# gxs_v2 下位机

STM32H723 三全向轮底盘固件。负责速度环、底盘运动学、遥控与上位机指令的仲裁，
以及出生点和安全区颜色的设置与显示。

## 硬件

| 项     | 参数                                             |
| ------ | ------------------------------------------------ |
| MCU    | STM32H723VGT6，550 MHz，1 MB Flash / 128 KB DTCM |
| 电机   | 3 路 H 桥，PWM 10 kHz（ARR 27499）               |
| 编码器 | 500 线 × 4 倍频，减速比 1:30，轮径 6.5 cm       |
| 舵机   | 2 路，TIM5_CH1 / CH2，500~2500 µs，270°        |
| 遥控   | R9DS，SBUS                                       |
| 显示   | SSD1306 128×64，I2C1，地址 0x3C                 |
| 按键   | KEY1 = PE4，KEY2 = PE5，按下为低                 |

串口分配：

| 串口    | 引脚               | 波特率 | 用途                 |
| ------- | ------------------ | ------ | -------------------- |
| USART1  | PB6 = TX，PB7 = RX | 921600 | 上位机协议           |
| UART5   | PB5 / PC12         | 100000 | R9DS SBUS            |
| UART8   | PE0 / PE1          | 921600 | 调试打印 + VOFA 波形 |
| USART10 | PE2 / PE3          | 921600 | 预留                 |

## 任务

FreeRTOS（CMSIS_V2），tick 1 kHz。优先级从高到低：

| 任务            | 周期       | 优先级      | 职责                                           |
| --------------- | ---------- | ----------- | ---------------------------------------------- |
| `motor_task`  | 1 kHz      | High1 (41)  | 三台电机的速度环，唯一调用 `update()` 的地方 |
| `host_task`   | 事件驱动   | High (40)   | 上位机串口收发，空闲时睡 `portMAX_DELAY`     |
| `key_task`    | 50 ms      | Normal (24) | KEY1 / KEY2 消抖与短按、长按识别               |
| `servo_task`  | 50 Hz      | Normal (24) | 舵机目标角推进                                 |
| `remote_task` | 100 Hz     | Normal (24) | 读 R9DS，按 CH5 决定控制源                     |
| `oled_task`   | 40 ms 节拍 | Low (8)     | 给节拍；画面与刷屏判断在 `module/field_page` |
| 库 `sys_task` | 10 ms      | 52          | ICBSP 的串口 / 总线巡检                        |
| 定时器服务      | —         | 20          | `FIELD_STATUS` 重发                          |

`motor_monitor_task`（1 kHz，VOFA 波形）默认注释掉，需要时在 `app_main.cpp` 打开。

## 功能

**控制源**。CH5 三挡：`+100` 上位机、`0` 强制停车、`-100` 遥控。遥控器掉线时自动交给
上位机并先停一次车。控制源不是上位机时，运动指令回 `ACK(3)`，正在跑的动作回
`MOVE_DONE(3)`。

**速度环**。1 kHz 任务里原子化采样编码器增量与 dt，一阶低通 100 Hz，三台电机各有独立
PID。速度环输出的是占空比，`out_max` 为 100。

**运动学**。三麦轮，轮心到中心 10 cm，平移限幅 130 cm/s、自转限幅 13 rad/s。电机顺序
为 M1 右前、M2 左前、M3 正后。

**上位机协议**。帧格式、指令表、回帧、测试字节串都在
[`user/docs/上位机协议.md`](user/docs/上位机协议.md)。要点：无版本字段，`AA 55` 帧头，
`0D 0A` 帧尾，`SUM` 从 `TYPE` 起累加。

**场地页**。画面与刷屏判断在 `user/module/field_page.cpp`，`oled_task` 只给 40 ms 节拍，画面不变时不刷。
OLED 显示四角出生点与红蓝两处安全区，右侧显示当前选中项。KEY1 短按换出生点，
KEY2 短按换颜色，长按 500 ms 推送 `FIELD_STATUS`（100 ms 一帧，共 10 帧，收到上位机的
`0xAC` 提前停止）；确认成功后右侧显示 `已发送` 一秒。

## 联调

上位机接 USART1。测试时用 CH343P（USB 厂商号 `1a86`）转到电脑，一般枚举成
`/dev/ttyACM0`，参数 921600 8N1。

```bash
python3 user/tools/proto_test.py find                              # 找哪个口在跑协议
python3 user/tools/proto_test.py -p /dev/ttyACM0 ping --count 5    # 判活
python3 user/tools/proto_test.py -p /dev/ttyACM0 monitor           # 解码打印所有回帧
python3 user/tools/proto_test.py -p /dev/ttyACM0 selftest          # 全流程自检
```

其余命令（速度、位移、舵机、急停、场地确认）见协议文档 §12。

## 调试

UART8 是调试口，`print_uart(...)` 的输出和 VOFA 波形共用这一路。VOFA 用 JustFloat 四通道，
观察对象由 `motor_monitor_index` 选择（0/1/2 对应三台电机，≥3 停发），测试波形由
`motor_test` 控制（0 关闭、1 方波、2 正弦、4 开环占空比）。两者都需要在调试器里改
RAM，改完不用重新烧录。测试波形不影响遥控 / 上位机，测试时自己别同时下发运动指令。

## 目录

```
Core/           CubeMX 生成的初始化与中断
Drivers/        STM32 HAL 与 CMSIS
Middlewares/Third_Party/ICBSP   板级库子模块（串口、PWM、编码器、电机、舵机、按键、OLED）
user/board/     外设实例与 board_init()
user/module/    底盘运动学、上位机协议执行器、场地页（field_page）、模块层全局实例（chassis / host / field_page）
user/app/       任务实现与任务创建（画面、协议逻辑不在这一层）
Flash/          OpenOCD / J-Link 烧录脚本
user/tools/     proto_test.py 协议测试工具
cmake/          工具链与 CubeMX 接入
```
