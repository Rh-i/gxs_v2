/**
 * @file motor_monitor_task.cpp
 * @brief 电机监控任务：把「一台（可切换）电机」的目标 / 反馈 / PID 输出按 JustFloat 发给 VOFA+
 * @date 2026-11-24
 *
 * @note 观察哪一台由 `motor_monitor_index` 决定，运行中可改，下一拍生效：
 *         - 调试器：`set var motor_monitor_index = 1`
 *         - EmberProbe / Live Watch：直接写变量 `motor_monitor_index`
 *       取值 0/1/2 = motor1/2/3，**≥ 3 = 不发**（停掉波形）。
 *
 * @note 自带测试波形（同一台电机，运行中可改 `motor_test_wave`）：
 *         0 = 关（底盘照常听遥控 / 上位机）
 *         1 = 方波 ±50 cm/s，周期 2 s（1 s 正 / 1 s 负）
 *         2 = 正弦波 +50 cm/s，周期 2 s，与方波同相
 *         4 = **开环档**：绕过 PID，直接给 ±20% 占空比（同相位），用来对比正反向驱动/被控对象是否对称
 *       测试开启期间 `remote_task` 不再写底盘（否则它 100 Hz 的 chassis.stop()
 *       会把目标冲掉）；**测试时不要同时发上位机运动指令**。
 *       切回 0 或换台时，本任务会把上一台 `stop()` 掉，不留残余目标。
 *
 * @note 通道顺序（VOFA+ JustFloat，**4 通道**，帧尾 00 00 80 7F）：
 *         CH1 = 目标速度 (cm/s)、CH2 = 实测速度 (cm/s)、CH3 = PID 输出（占空比 %，带符号）、
 *         CH4 = 编码器原始计数（BspEncoder::read_raw()，int32 转 float，不乘任何换算）
 *       开环档（wave = 4）下没有 PID 数据：CH1/CH3 报当前开环占空比，CH2 仍是实测速度。
 *       单帧 20 B，1 kHz 下 20 kB/s。
 *
 * @note 走 **UART8**（921600，同时承载调试打印）：JustFloat 是二进制流，
 *       文本日志（按键/sys 报告）混进来会让 VOFA+ 解析出零星跳变；要完全干净就把打印口分开。
 *
 * @note 本任务不调 update()（PWM 写入权在 1 kHz 的 motor_task），只写目标值/开环占空比和读查询接口。
 */

#include "app_task.hpp"

#include "board_cfg.hpp"             // motor1/2/3 / bsp_uart8（VOFA 波形口）
#include "bsp/bsp_dwt.hpp"           // 测试波形的相位基准
#include "service/system/system.hpp" // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <math.h> // sinf
#include <stddef.h>
#include <stdint.h>
#include <string.h> // memcpy


/** @brief 发送周期 (ms)：1 ms = 1 kHz，与 motor_task 同频，波形上能看到每拍细节 */
constexpr uint32_t MOTOR_MONITOR_PERIOD_MS = 1U;

/** @brief 测试波形幅值 (cm/s)：方波与正弦共用 */
constexpr float MOTOR_TEST_AMP_CM_S =50.0f;
/** @brief 测试波形周期 (s)：方波与正弦共用（方波 = 上半周 +50 / 下半周 −50，与正弦同相） */
constexpr float MOTOR_TEST_PERIOD_S = 2.0f;

/** @brief 开环测试档（wave = 4）的占空比幅值 (%)：同占空比下对比正反向驱动是否对称 */
constexpr float MOTOR_TEST_OPEN_DUTY_PCT = 50.0f;

constexpr float PI_F = 3.14159265f;

/** @brief 监控对象下标：0/1/2 → motor1/2/3；≥ 3 表示停发。运行时读写，故 volatile */
volatile uint8_t motor_monitor_index = 2U;


/** @brief 测试波形：0 = 关，1 = 方波，2 = 正弦，4 = 开环。运行时读写，故 volatile
 *  @note **必须保持 0 才能用遥控器 / 上位机**：非 0 时 remote_task 会完全让出底盘控制权（见那里的保护逻辑）。 */
volatile uint8_t motor_test_wave = 0U;

/** @brief 下标 → 实例；顺序与 motor_monitor_index 一致 */
static DeviceMotor *const motor_monitor_list[] = {&motor1, &motor2, &motor3};

/** @brief 下标 → 该电机的编码器（CH4 用）；顺序与 motor_monitor_index 一致，与 motor1/2/3 的绑定一致 */
static BspEncoder *const motor_encoder_list[] = {&bsp_encoder_tim3, &bsp_encoder_tim1, &bsp_encoder_tim4};

constexpr uint8_t MOTOR_MONITOR_COUNT = static_cast<uint8_t>(sizeof(motor_monitor_list) / sizeof(motor_monitor_list[0]));


/** @brief 当前测试相位 (rad)：用 DWT 绝对时间，先对周期取余，回绕安全 */
static float test_phase_rad(void)
{
  const float t = static_cast<float>(bsp_dwt.get_time_s());

  // 先对周期取余，避免 t 变大后 float 精度下降（t/2 超过 1e7 后才开始丢小数）
  float phase = t - (MOTOR_TEST_PERIOD_S * static_cast<float>(static_cast<int32_t>(t / MOTOR_TEST_PERIOD_S)));
  return (2.0f * PI_F) * phase / MOTOR_TEST_PERIOD_S;
}

/** @brief 按 motor_test_wave 算这一拍的目标速度 (cm/s) */
static float test_target(uint8_t wave)
{
  const float phase = test_phase_rad();

  if (wave == 1U)
  {
    return (sinf(phase) >= 0.0f) ? MOTOR_TEST_AMP_CM_S : -MOTOR_TEST_AMP_CM_S;
  }

  return MOTOR_TEST_AMP_CM_S * sinf(phase);
}

/** @brief 开环测试档：按同一相位给 ±占空比（不经过 PID） */
static float test_open_duty(void)
{
  return (sinf(test_phase_rad()) >= 0.0f) ? MOTOR_TEST_OPEN_DUTY_PCT : -MOTOR_TEST_OPEN_DUTY_PCT;
}


extern "C" void motor_monitor_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  TickType_t   wake_time   = xTaskGetTickCount();
  DeviceMotor *prev_driven = nullptr; // 上一拍被测试波形驱动的电机（换台 / 关测试时要停它）

  for (;;)
  {
    const uint8_t idx  = motor_monitor_index;
    const uint8_t wave = motor_test_wave;

    DeviceMotor *const motor = (idx < MOTOR_MONITOR_COUNT) ? motor_monitor_list[idx] : nullptr;

    if ((motor != nullptr) && (wave != 0U)) // 测试波形：写目标，闭环仍由 motor_task 推进
    {
      if ((prev_driven != nullptr) && (prev_driven != motor))
      {
        prev_driven->stop(); // 换台：先把上一台停住，不留残余目标
      }
      prev_driven = motor;
      if (wave == 4U)
      {
        motor->set_duty_open(test_open_duty()); // 开环：绕过 PID，直接给占空比
      }
      else
      {
        motor->set_speed(test_target(wave));
      }
    }
    else if (prev_driven != nullptr)
    {
      prev_driven->stop(); // 关测试或切到 ≥ 3：把电机停住
      prev_driven = nullptr;
    }

    if (motor != nullptr) // ≥ 3：停发
    {
      // get_pid_frame 先给 3 个 float + 4 B 尾（共 16 B）；把尾往后挪，第 4 槽换成编码器原始计数，
      // 重新补一个 JustFloat 尾 → 4 通道共 20 B 整帧入队，异步由 DMA 发出去。
      uint8_t frame[20];
      const size_t len = motor->get_pid_frame(frame, DeviceMotor::PID_ALL); // 3 通道 16 B

      if (len == 16U)
      {
        const float raw = static_cast<float>(motor_encoder_list[idx]->read_raw());
        memcpy(&frame[12], &raw, sizeof(raw));
        frame[16] = 0x00U;
        frame[17] = 0x00U;
        frame[18] = 0x80U;
        frame[19] = 0x7FU;
        (void)bsp_uart8.send(frame, 20U, 1U);
      }
    }

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(MOTOR_MONITOR_PERIOD_MS));
  }
}
