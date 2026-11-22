/**
 * @file servo_task.cpp
 * @brief 舵机任务：50 Hz 推进 servo1 / servo2 的 update()
 * @date 2026-11-21
 *
 * @note `DeviceServo::update()` 是唯一的 PWM 写入者，必须周期调用；`set_angle()` 只写目标值，
 *       由 remote_task 根据遥控器旋钮下发（跨任务安全），本任务只负责推进。
 *       上电默认中位在 `board_init()` 里给（135°，对应 1500 µs）。
 *
 * @note 周期 20 ms 与舵机的 50 Hz 刷新一致；以后要开限速（`max_speed_dps` / `max_accel_dps2`），
 *       dt 由本任务内部按 tick 计算，改周期要重新验一下加减速手感。
 *
 * @note 本任务是 servo1 / servo2 唯一的 update() 调用者（DeviceServo 非线程安全）。
 */

#include "app_task.hpp"

#include "board_cfg.hpp"             // servo1 / servo2
#include "service/system/system.hpp" // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


/** @brief 推进周期 (ms)：20 ms = 50 Hz，与舵机 PWM 周期一致 */
constexpr uint32_t SERVO_TASK_PERIOD_MS = 20U;


/* ==================== 任务 ==================== */

extern "C" void servo_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  TickType_t wake_time = xTaskGetTickCount();

  for (;;)
  {
    servo1.update();
    servo2.update();

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(SERVO_TASK_PERIOD_MS));
  }
}

/* ==================== */
