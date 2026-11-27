/**
 * @file board_cfg.cpp
 * @brief `board_init()`：把 board_cfg.hpp 里的实例按正确顺序逐个 init()
 *
 * @details
 *
 * 【本文件只放一件事】`board_init()` 的函数体。
 *   实例都定义在 `board_cfg.hpp`（inline 全局变量），配置和流程分开：
 *     - 加外设 / 改参数  → `board_cfg.hpp` + `board_instances.hpp`
 *     - 改顺序 / 加设备  → 本文件
 *
 * 【为什么每行都 ICBSP_CHECK】
 *   `init()` 返回非 OK 说明硬件或参数不对（CubeMX 没生成句柄、DMA 没配、实例没写进清单、CAN 起不来、
 *   芯片 ID 读不到…）。`ICBSP_CHECK(x.init() == Status::OK)` 总会求值（不要写进 configASSERT：断言可能被定义成空），
 *   失败就地停机，`fault_record` 会记下**代码行号**，
 *   一眼能看出是哪一路；继续跑下去只会在半初始化的系统上产生更难查的现象。
 *   想改成“降级运行”也可以，但那样必须在 sys_state 里记错误：
 *     `if (bsp_uart5.init() != Status::OK) { (void)sys_state.error(Status::IO_ERROR); }`
 *   这样 `system_start()` 才知道有子系统挂了（它的 waiting 逻辑依赖 sys_state）。
 *
 * 【顺序有讲究（本文件的顺序不能乱）】
 *   1. 计时基准 `bsp_dwt` 由库在 `system_init()` 里初始化，宿主不用管
 *   2. 总线与外设（CAN / UART / SPI / PWM）之间没有依赖，按 `board_cfg.hpp` 的定义顺序写即可
 *   3. **设备放在它用的总线之后**：
 *      - CAN 设备（DjiMotor、DmMotor）要往 `CanRxNodeT` / `CanTxNodeT` 里注册，并挂到总线的状态节点下
 *      - 串口设备（R9DS、Emm_V5 组）、SPI 设备（BMI088）、PWM 设备（舵机、蜂鸣器）同理，放在各自的外设之后
 *   4. 全部在 `system_start()` 之前：`can_tx_task` 按“有没有发送节点”决定建不建，运行态之后再注册节点会被拒绝
 */

#include "board_cfg.hpp"
#include "service/system/fault_record.hpp" // ICBSP_CHECK

#include "i2c.h" // hi2c1（OLED）

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)


/* ==================== 设备实例 ==================== */
// 生成区（本工程为空：设备需要 PID / 量程等手写参数，见下一节）
// <ICBSP-GEN devices>
// 本工程的设备实例全部手写在下面的“设备实例（手写）”中，生成区留空。
// </ICBSP-GEN devices b7dd9c1b>
/* ==================== */

/* ==================== 设备实例（手写） ==================== */

namespace
{
/** 三个电机共用同一套机械参数：500 线编码器、TI12（4 倍频）、1:30 减速、6.5 cm 轮 */
constexpr float MOTOR_ENCODER_LINES      = 500.0f;
constexpr float MOTOR_ENCODER_MULTIPLIER = 4.0f;
constexpr float MOTOR_GEAR_RATIO         = 30.0f;
constexpr float MOTOR_WHEEL_DIAMETER_CM  = 6.5f;

// 速度环 PID 三台各写各的（见下面 motor1 / motor2 / motor3 的 .pid），调试时只改自己那台，互不影响。
// PID::Config 位置参数：kp, ki, kd, out_max
// 当前值：motor1 25/0.05、motor2 20/0.04、motor3 23/0.05，out_max 100

/** 舵机默认角度：270° 舵机的中位（对应 1500 µs） */
constexpr float SERVO_MID_DEG = 135.0f;
} // namespace

/// 电机1：PWM TIM2_CH3/CH4 + 编码器 TIM3
DeviceMotor motor1({.pwm_a              = &bsp_pwm_tim2_ch3,
                    .pwm_b              = &bsp_pwm_tim2_ch4,
                    .encoder            = &bsp_encoder_tim3,
                    .encoder_lines      = MOTOR_ENCODER_LINES,
                    .encoder_multiplier = MOTOR_ENCODER_MULTIPLIER,
                    .gear_ratio         = MOTOR_GEAR_RATIO,
                    .wheel_diameter_cm  = MOTOR_WHEEL_DIAMETER_CM,
                    .speed_lpf_hz       = 100.0f, // 速度反馈低通 100 Hz
                    .pid                = PID::Config(22.0f, 0.06f, 0.0f, 100.0f)});

/// 电机2：PWM TIM12_CH1/CH2 + 编码器 TIM1
DeviceMotor motor2({.pwm_a              = &bsp_pwm_tim12_ch1,
                    .pwm_b              = &bsp_pwm_tim12_ch2,
                    .encoder            = &bsp_encoder_tim1,
                    .encoder_lines      = MOTOR_ENCODER_LINES,
                    .encoder_multiplier = MOTOR_ENCODER_MULTIPLIER, 
                    .gear_ratio         = MOTOR_GEAR_RATIO,
                    .wheel_diameter_cm  = MOTOR_WHEEL_DIAMETER_CM,
                    .speed_lpf_hz       = 100.0f, // 速度反馈低通 100 Hz
                    .pid                = PID::Config(17.0f, 0.07f, 0.0f, 100.0f)});

/// 电机3：PWM TIM8_CH1/CH2 + 编码器 TIM4
DeviceMotor motor3({.pwm_a              = &bsp_pwm_tim8_ch1,
                    .pwm_b              = &bsp_pwm_tim8_ch2,
                    .encoder            = &bsp_encoder_tim4,
                    .encoder_lines      = MOTOR_ENCODER_LINES,
                    .encoder_multiplier = MOTOR_ENCODER_MULTIPLIER,
                    .gear_ratio         = MOTOR_GEAR_RATIO,
                    .wheel_diameter_cm  = MOTOR_WHEEL_DIAMETER_CM,
                    .speed_lpf_hz       = 100.0f, // 速度反馈低通 100 Hz
                    .pid                = PID::Config(17.0f, 0.07f, 0.0f, 100.0f)});

/// 舵机1：TIM5_CH1，270° 舵机，500~2500 µs 对应 0~270°
DeviceServo servo1({.pwm = &bsp_pwm_tim5_ch1, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f, .range_deg = 270.0f});

/// 舵机2：TIM5_CH2，参数同 servo1
DeviceServo servo2({.pwm = &bsp_pwm_tim5_ch2, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f, .range_deg = 270.0f});

/// R9DS 遥控接收机：UART5，SBUS
DeviceR9dsT<decltype(bsp_uart5)> r9ds(bsp_uart5);

/// OLED：I²C1，SSD1306 128×64（显存 1152 B，放静态区）
IicOled oled;

/* ==================== */


/* ==================== 板级初始化 ==================== */
void board_init(void)
{
  // ① 计时基准 `bsp_dwt` 已由 `system_init()` 内部 init()，这里不再重复初始化。

  // ② 总线与外设：失败会停在对应行号，fault_record 里能看到是哪一路。
  // <ICBSP-GEN init>
  ICBSP_CHECK(bsp_uart1.init() == Status::OK);        // USART1  上位机通信 921600
  ICBSP_CHECK(bsp_uart5.init() == Status::OK);        // UART5   R9DS SBUS 100000 8E2
  ICBSP_CHECK(bsp_uart8.init() == Status::OK);        // UART8   调试打印 + VOFA 921600
  ICBSP_CHECK(bsp_uart10.init() == Status::OK);       // USART10 预留（现上下位机走 USART1）
  ICBSP_CHECK(bsp_pwm_tim2_ch3.init() == Status::OK); // 电机1 H 桥
  ICBSP_CHECK(bsp_pwm_tim2_ch4.init() == Status::OK);
  ICBSP_CHECK(bsp_pwm_tim12_ch1.init() == Status::OK); // 电机2 H 桥
  ICBSP_CHECK(bsp_pwm_tim12_ch2.init() == Status::OK);
  ICBSP_CHECK(bsp_pwm_tim8_ch1.init() == Status::OK); // 电机3 H 桥
  ICBSP_CHECK(bsp_pwm_tim8_ch2.init() == Status::OK);
  ICBSP_CHECK(bsp_pwm_tim5_ch1.init() == Status::OK); // 舵机
  ICBSP_CHECK(bsp_pwm_tim5_ch2.init() == Status::OK);
  ICBSP_CHECK(bsp_encoder_tim3.init() == Status::OK); // 电机1 编码器
  ICBSP_CHECK(bsp_encoder_tim1.init() == Status::OK); // 电机2 编码器
  ICBSP_CHECK(bsp_encoder_tim4.init() == Status::OK); // 电机3 编码器
  ICBSP_CHECK(key1.init() == Status::OK);             // PE4 按键（上拉，按下为低）
  ICBSP_CHECK(key2.init() == Status::OK);             // PE5 按键
  // </ICBSP-GEN init dc07c491>

  // ③ 设备：必须在自己用的外设 init() 之后（编码器要先跑起来，否则闭环会一路积分到限幅）。
  ICBSP_CHECK(motor1.init() == Status::OK); // TIM2 + TIM3
  ICBSP_CHECK(motor2.init() == Status::OK); // TIM12 + TIM1
  ICBSP_CHECK(motor3.init() == Status::OK); // TIM8 + TIM4
  ICBSP_CHECK(servo1.init() == Status::OK); // TIM5_CH1
  ICBSP_CHECK(servo2.init() == Status::OK); // TIM5_CH2

  // 舵机上电默认中位：这里只登记目标，真正的 PWM 由 servo_task 的周期 update() 写入
  servo1.set_angle(SERVO_MID_DEG);
  servo2.set_angle(SERVO_MID_DEG);
  ICBSP_CHECK(r9ds.init() == Status::OK);  // UART5

  // ④ 可选外设：OLED（I²C1）。没接屏/接线错时只静默继续，由 oled_task 低频重试，不影响整机。
  (void)oled.init(&hi2c1, 0x3C, IicOled::Rotation::ROT_0);
}

/* ==================== */


/* ==================== 中断转调（生成的设备用到 EXTI 时在这里生成 HAL_GPIO_EXTI_Callback；宿主别处就不要再定义它） ==================== */
// <ICBSP-GEN callbacks>
// 由 tools/integrate.py 按 .ioc 与接入选择生成：要改就改选择后重新运行；手改本区会被检测到、拒绝覆盖
// </ICBSP-GEN callbacks acb74ce2>
/* ==================== */
