/**
 * @file board_cfg.hpp
 * @brief 板级实例配置：本板有哪些外设实例、绑定哪些 CubeMX 句柄、参数是什么
 *
 * @details
 *
 * 【板级四个文件的分工】
 *
 *   | 文件                  | 放什么                             | 什么时候改                     |
 *   | --------------------- | ---------------------------------- | ------------------------------ |
 *   | `hardware_config.h`   | 库的全部宏（开关/优先级/队列/超时） | 调库的行为、换板               |
 *   | `board_cfg.hpp`（本文件） | 外设实例（inline 全局变量）与句柄绑定 | 加/删外设、改量程波特率等参数   |
 *   | `board_instances.hpp` | 实例名清单（X-macro）              | **每次加实例都要同步**         |
 *   | `board_cfg.cpp`       | `board_init()`：谁先 init、谁后 init | 改初始化顺序、加设备 init       |
 *
 *   业务任务与更高层的编排在 `user/app/`，本目录只管“板子上有什么、怎么起来”。
 *
 * 【初始化时序（从复位到业务跑起来）】
 *
 *   main()
 *     → HAL_Init / SystemClock_Config
 *     → MX_GPIO_Init / MX_DMA_Init / MX_TIMx_Init / MX_USARTx_UART_Init / MX_FDCANx_Init …（CubeMX 生成）
 *        这些只**配置**外设，不启动收发
 *     → MX_FREERTOS_Init()
 *         → app_init()                       ← user/app/app_main.cpp
 *             1) system_init()               ← 库：建 sys_state、创建 sys / can_rx 任务
 *             2) board_init()                ← **本目录**：各实例 init()、设备 init()、注册 CAN 节点
 *             3) system_start()              ← 库：有发送节点才建 can_tx，然后置运行位
 *     → osKernelStart()                      调度器启动，任务才开始跑
 *
 *   所以 `board_init()` 里的“顺序”只对**初始化**有意义：此时调度器还没跑，
 *   本文件里的 `init()` 都不允许阻塞等事件、也不允许依赖别的任务。
 *
 * 【三条硬规矩（踩了就很难查）】
 *
 *   1. **串口 / SPI 实例必须放 `.dma_buffer` 段，且段名按变量唯一**
 *      H7 的 DMA1/DMA2 访问不到 ITCM / DTCM，实例里的接收/发送缓冲如果是普通全局变量
 *      （被链接到 DTCM），DMA 搬过去就是脏数据或直接卡住。CMake 配置阶段有段检查会拦。
 *
 *      段名必须写成 `.dma_buffer.<变量名>`，**不能只写 `.dma_buffer`**：实例是
 *      C++17 `inline` 变量，多个 TU 各 emit 一份弱定义，段名相同则弱定义里的数据
 *      会一起进同一个输出段（符号去重了、数据没去），表现为段里多出一份无符号的
 *      副本，或者链接器 + `--gc-sections` 剔掉真数据。按变量命名后每个变量独占一个
 *      COMDAT 段，重复副本会被正常丢弃。
 *
 *   2. **`BspUartT` 只有头文件，Transport 与缓冲尺寸随便组合**
 *      DMA 模式的 RX DMA 由 `init()` 自动改成循环模式；CubeMX 里配 Normal 也可以。
 *
 *   3. **本文件每加一个实例，`board_instances.hpp` 必须同步加名字**
 *      串口实例漏写时 `init()` 返回 BAD_ARG；CAN / SPI 实例漏写目前不报错，只是 HAL 回调不会被分发
 *      ——表现是“数据收不到”，不是“编译失败”。
 *
 * 【加一个外设实例的完整步骤（以再加一路 CAN2 为例）】
 *
 *   1. CubeMX：开 FDCAN2 的 RX FIFO0 中断，确认经典 CAN 模式，重新生成
 *   2. 本文件「外设实例」分节下面加：
 *          inline BspCanT<16, 16> bsp_can2({.hfdcan = &hfdcan2, .name = "CAN2"});
 *   3. `board_instances.hpp` 的 `ICBSP_CAN_INSTANCES` 里加 `X(bsp_can2)`
 *   4. `board_cfg.cpp` 的 `board_init()` 里加一行 `ICBSP_CHECK(bsp_can2.init() == Status::OK);`
 */

#ifndef __BOARD_CFG_HPP__
#define __BOARD_CFG_HPP__

// 下面三个生成区（includes / peripherals / devices）由 tools/integrate.py 按 .ioc 与接入选择生成，区外的内容不动。
// 手写工程直接改；改过的生成区再用 integrate.py 生成会被拒绝（先删掉或恢复）。
// CubeMX 生成的句柄头文件：**按实际用到的外设增删**。
// <ICBSP-GEN includes>
// 由 tools/integrate.py 按 .ioc 与接入选择生成：要改就改选择后重新运行；手改本区会被检测到、拒绝覆盖
#include "bsp/bsp_dwt.hpp" // IWYU pragma: keep
#include "bsp/bsp_encoder.hpp"
#include "bsp/bsp_gpio.hpp"
#include "bsp/bsp_key.hpp"
#include "bsp/bsp_pwm.hpp"
#include "bsp/bsp_series.hpp" // IWYU pragma: keep
#include "bsp/bsp_uart.hpp"
#include "device/actuator/device_servo.hpp"
#include "device/motor/device_motor.hpp"
#include "device/remote/device_r9ds.hpp"
#include "device/sensor/device_wt102.hpp"
#include "main.h" // IWYU pragma: keep（GPIO 标签宏）
#include "tim.h"
#include "usart.h"
// </ICBSP-GEN includes 7b269904>

/** @brief 板级初始化：各实例 init()，实现在 board_cfg.cpp */
void board_init(void);


/* ==================== 外设实例 ==================== */
// 外设实例是 inline 全局变量：整个工程只有一份定义，任何包含本头文件的 TU 都能直接用。
//
// 为什么必须 `constexpr` 构造 + 平凡析构：inline 变量要求常量初始化，
// 否则每个包含本头文件的 .cpp 都会生成一份构造代码，还会注册 atexit。
// 需要释放资源时显式调 `deinit()`，不要写析构函数。

// <ICBSP-GEN peripherals>
/// USART1：**通信串口**（PB6/PB7，921600 8N1，DMA 收发，缓冲加大到 512）
inline ICBSP_DMA_BUFFER(bsp_uart1) BspUartT<uart_mode::dma, 512, 512> bsp_uart1({.huart = &huart1, .mode = UartDir::TxRx, .baudrate = 921600U});
/// USART2：WT102 陀螺仪（PA2/PA3，230400 8N1，DMA 只收）
inline ICBSP_DMA_BUFFER(bsp_uart2) BspUartT<uart_mode::dma, 256, 128> bsp_uart2({.huart = &huart2, .mode = UartDir::RxOnly, .baudrate = 230400U});
/// UART5：R9DS 遥控（PB5/PC12，SBUS：100000、8 数据位 + 偶校验 + 2 停止位，DMA 只收）
inline ICBSP_DMA_BUFFER(bsp_uart5) BspUartT<uart_mode::dma, 256, 128> bsp_uart5({.huart = &huart5, .mode = UartDir::RxOnly, .baudrate = 100000U});
/// UART8：**调试打印串口**（PE0/PE1，921600 8N1，DMA 收发；也承载 VOFA JustFloat 波形）
inline ICBSP_DMA_BUFFER(bsp_uart8) BspUartT<uart_mode::dma, 128, 128> bsp_uart8({.huart = &huart8, .mode = UartDir::TxRx, .baudrate = 921600U});
/// USART10：**上下位机通讯串口**（PE2/PE3，921600 8N1，DMA 收发，缓冲加大到 512）
inline ICBSP_DMA_BUFFER(bsp_uart10) BspUartT<uart_mode::dma, 512, 512> bsp_uart10({.huart = &huart10, .mode = UartDir::TxRx, .baudrate = 921600U});
/// TIM2_CH3 / CH4：电机1 H 桥（PB10/PB11，10 kHz）
inline BspPwm bsp_pwm_tim2_ch3({.htim = &htim2, .channel = TIM_CHANNEL_3, .timer_clk_hz = 275000000UL, .prescaler = 0U, .period = 27499U});
inline BspPwm bsp_pwm_tim2_ch4({.htim = &htim2, .channel = TIM_CHANNEL_4, .timer_clk_hz = 275000000UL, .prescaler = 0U, .period = 27499U});
/// TIM12_CH1 / CH2：电机2 H 桥（PB14/PB15，10 kHz）
inline BspPwm bsp_pwm_tim12_ch1({.htim = &htim12, .channel = TIM_CHANNEL_1, .timer_clk_hz = 275000000UL, .prescaler = 0U, .period = 27499U});
inline BspPwm bsp_pwm_tim12_ch2({.htim = &htim12, .channel = TIM_CHANNEL_2, .timer_clk_hz = 275000000UL, .prescaler = 0U, .period = 27499U});
/// TIM8_CH1 / CH2：电机3 H 桥（PC6/PC7，10 kHz）
inline BspPwm bsp_pwm_tim8_ch1({.htim = &htim8, .channel = TIM_CHANNEL_1, .timer_clk_hz = 275000000UL, .prescaler = 0U, .period = 27499U});
inline BspPwm bsp_pwm_tim8_ch2({.htim = &htim8, .channel = TIM_CHANNEL_2, .timer_clk_hz = 275000000UL, .prescaler = 0U, .period = 27499U});
/// TIM5_CH1 / CH2：舵机（PA0/PA1，50 Hz / 20 ms，1 计数 = 1 µs）
inline BspPwm bsp_pwm_tim5_ch1({.htim = &htim5, .channel = TIM_CHANNEL_1, .timer_clk_hz = 275000000UL, .prescaler = 274U, .period = 19999U});
inline BspPwm bsp_pwm_tim5_ch2({.htim = &htim5, .channel = TIM_CHANNEL_2, .timer_clk_hz = 275000000UL, .prescaler = 274U, .period = 19999U});
/// TIM3：编码器（PA6/PA7）→ 电机1
inline BspEncoder bsp_encoder_tim3({.htim = &htim3, .invert = false});
/// TIM1：编码器（PE9/PE11）→ 电机2
inline BspEncoder bsp_encoder_tim1({.htim = &htim1, .invert = false});
/// TIM4：编码器（PD12/PD13）→ 电机3
inline BspEncoder bsp_encoder_tim4({.htim = &htim4, .invert = false});
/// KEY1：PE4，GPIO 输入（板上已外部上拉），按下为低。key_task 按 200 ms 轮询：消抖 1×=200 ms、长按 5×=1 s
inline BspKey key1({.port = KEY1_GPIO_Port, .pin = KEY1_Pin, .active_low = true, .debounce_cnt = 1U, .long_press_cnt = 5U});
/// KEY2：PE5，参数同 key1（外部上拉）
inline BspKey key2({.port = KEY2_GPIO_Port, .pin = KEY2_Pin, .active_low = true, .debounce_cnt = 1U, .long_press_cnt = 5U});
/// LED1：PC13，GPIO 输出（CubeMX 里初始电平 Reset；板上低电平点亮时也就对应“亮”）
inline BspGpio led1({.port = LED1_GPIO_Port, .pin = LED1_Pin});

/// 调试打印串口：UART8（PE0/PE1）
#define ICBSP_UART_PRINT_ENABLE 1
#define print_uart(...) bsp_uart8.printf(__VA_ARGS__)
// </ICBSP-GEN peripherals 5129401a>


/* ==================== */
/* ==================== 设备实例（手写） ==================== */
// 设备层：包装外设，提供“按物理量”的接口；不拥有下面的外设实例。
// 顺序要求：设备必须在本文件里外设实例之后定义，并且在 board_init() 里外设 init() 之后 init()。
// 本工程全部手写（PID / 舵机量程等参数无法由生成器表达），生成区留空。

// <ICBSP-GEN devices>
// 生成区留空：设备实例见下方手写部分。
// </ICBSP-GEN devices 54f5bc5d>
/// 电机1：PWM TIM2 + 编码器 TIM3（500 线 × 4 倍频、1:30、6.5 cm 轮）
extern DeviceMotor motor1;
/// 电机2：PWM TIM12 + 编码器 TIM1
extern DeviceMotor motor2;
/// 电机3：PWM TIM8 + 编码器 TIM4
extern DeviceMotor motor3;
/// 舵机1：TIM5_CH1（270°，500~2500 µs）
extern DeviceServo servo1;
/// 舵机2：TIM5_CH2（270°，500~2500 µs）
extern DeviceServo servo2;
/// WT102 陀螺仪：USART2（230400）
extern DeviceWt102T<decltype(bsp_uart2)> wt102;
/// R9DS 遥控：UART5（SBUS）
extern DeviceR9dsT<decltype(bsp_uart5)> r9ds;

/* ==================== */

#endif // __BOARD_CFG_HPP__
