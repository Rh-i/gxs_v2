/**
 * @file motor_monitor_task.cpp
 * @brief 电机监控任务：把一台（可切换）电机的目标 / 反馈 / PID 输出按 JustFloat 发给 VOFA+
 * @date 2026-11-24
 *
 * @note 观察哪台由 `motor_monitor_index` 决定（0/1/2 → motor1/2/3，≥ 3 停发），运行中可改，
 *       下一拍生效：调试器 `set var motor_monitor_index = 1`，或 Live Watch 直接写。
 *
 * @note 自带测试波形 `motor_test`（运行中可改）：
 *         0 = 关；
 *         1 = 方波 ±50 cm/s（周期 2 s）；2 = 正弦 ±50 cm/s（同相）；
 *         4 = 开环档，绕过 PID 直接给 ±50% 占空比，用来对比正反向驱动 / 被控对象是否对称。
 *       本任务只驱动电机，不碰遥控 / 上位机；测试时自己别同时让它们下发运动指令。
 *       切回 0 或换台时会把上一台 `stop()` 掉，不留残余目标。
 *
 * @note JustFloat 4 通道（帧尾 00 00 80 7F，单帧 20 B，1 kHz ≈ 20 kB/s）：
 *         CH1 目标速度 (cm/s)、CH2 实测速度 (cm/s)、CH3 PID 输出 (% 带符号)、
 *         CH4 编码器原始计数（库内 read_raw()，不乘任何换算）。
 *       开环档（wave = 4）下 CH1/CH3 报当前开环占空比，CH2 仍是实测速度。
 *
 * @note 走 UART8（921600，与调试打印共用）：JustFloat 是二进制流，文本日志混进来会让
 *       VOFA+ 解析出零星跳变，要完全干净得把打印口分开。
 *
 * @note 本任务不调 update()（PWM 写入权在 1 kHz 的 motor_task），只写目标值 / 开环占空比和读查询接口。
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


/** @brief 发送周期 (ms)：1 ms = 1 kHz，与 motor_task 同频，波形上能看到每拍细节 */
constexpr uint32_t MOTOR_MONITOR_PERIOD_MS = 1U;

/** @brief 测试波形幅值 (cm/s)：方波与正弦共用 */
constexpr float MOTOR_TEST_AMP_CM_S = 50.0f;
/** @brief 测试波形周期 (s)：方波上半周 +50 / 下半周 −50，与正弦同相 */
constexpr float MOTOR_TEST_PERIOD_S = 2.0f;
/** @brief 开环测试档（wave = 4）的占空比幅值 (%)：同占空比下对比正反向驱动是否对称 */
constexpr float MOTOR_TEST_OPEN_DUTY_PCT = 50.0f;

constexpr float PI_F = 3.14159265f;

/** @brief 监控对象下标：0/1/2 → motor1/2/3；≥ 3 表示停发。运行时读写，故 volatile */
volatile uint8_t motor_monitor_index = 2U;

/** @brief 测试波形：0 = 关，1 = 方波，2 = 正弦，4 = 开环。运行时读写，故 volatile */
volatile uint8_t motor_test = 0U;

/** @brief 下标 → 实例；顺序与 motor_monitor_index 一致 */
static DeviceMotor *const motor_monitor_list[] = {&motor1, &motor2, &motor3};
constexpr uint8_t MOTOR_MONITOR_COUNT = static_cast<uint8_t>(sizeof(motor_monitor_list) / sizeof(motor_monitor_list[0]));


/** @brief 这一拍测试档要写的值：1 方波 / 2 正弦 → 目标速度 (cm/s)，4 → 开环占空比 (%) */
static float test_value(uint8_t wave)
{
  // 相位用 DWT 绝对时间，先对周期取余，避免 t 变大后 float 精度下降
  const float t     = static_cast<float>(bsp_dwt.get_time_s());
  const float phase = (2.0f * PI_F) * (t - MOTOR_TEST_PERIOD_S * static_cast<float>(static_cast<int32_t>(t / MOTOR_TEST_PERIOD_S))) / MOTOR_TEST_PERIOD_S;
  const float s     = sinf(phase);

  if (wave == 4U)
  {
    return (s >= 0.0f) ? MOTOR_TEST_OPEN_DUTY_PCT : -MOTOR_TEST_OPEN_DUTY_PCT;
  }
  if (wave == 1U)
  {
    return (s >= 0.0f) ? MOTOR_TEST_AMP_CM_S : -MOTOR_TEST_AMP_CM_S;
  }
  return MOTOR_TEST_AMP_CM_S * s;
}


extern "C" void motor_monitor_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  TickType_t   wake_time   = xTaskGetTickCount();
  DeviceMotor *prev_driven = nullptr; // 上一拍被测试波形驱动的电机（换台 / 关测试时要停它）

  for (;;)
  {
    const uint8_t       idx   = motor_monitor_index;
    const uint8_t       wave  = motor_test;
    DeviceMotor *const motor = (idx < MOTOR_MONITOR_COUNT) ? motor_monitor_list[idx] : nullptr;

    if ((motor != nullptr) && (wave != 0U)) // 测试波形：写目标，闭环仍由 motor_task 推进
    {
      if ((prev_driven != nullptr) && (prev_driven != motor))
      {
        prev_driven->stop(); // 换台：先把上一台停住，不留残余目标
      }
      prev_driven = motor;

      const float v = test_value(wave);
      if (wave == 4U)
      {
        motor->set_duty_open(v); // 开环：绕过 PID，直接给占空比
      }
      else
      {
        motor->set_speed(v);
      }
    }
    else if (prev_driven != nullptr)
    {
      prev_driven->stop(); // 关测试或切到 ≥ 3：把电机停住
      prev_driven = nullptr;
    }

    if (motor != nullptr) // ≥ 3：停发
    {
      // 4 通道 JustFloat（目标 / 反馈 / PID 输出 / 编码器原始计数），整帧 20 B，
      // 入队后由 DMA 异步发出（超时 1 ms，丢帧不拖住 1 kHz 节拍）。
      uint8_t frame[20];
      if (motor->get_pid_frame(frame, DeviceMotor::PID_ALL) == sizeof(frame))
      {
        (void)bsp_uart8.send(frame, sizeof(frame), 1U);
      }
    }

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(MOTOR_MONITOR_PERIOD_MS));
  }
}
