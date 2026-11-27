/**
 * @file motor_task.cpp
 * @brief 电机速度环任务：按周期跑 motor1/2/3 的 update()
 * @date 2026-11-21
 *
 * @note 三台电机由底盘 / 遥控任务写目标（set_speed），本任务只负责推进速度环。
 *       本任务是三台电机**唯一的 update() 调用者**。
 */

#include "app_task.hpp"

#include "board_cfg.hpp"                // motor1/2/3
#include "service/system/system.hpp"    // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


/** @brief 控制周期 (ms)：1 ms = 1 kHz。想更快就改成别的值，update() 里的 dt 是实测的，不用同步改 */
constexpr uint32_t MOTOR_TASK_PERIOD_MS = 1U;


/**
 * @brief 电机速度环任务：按 MOTOR_TASK_PERIOD_MS 周期跑三路电机的 update()
 *
 * @note dt 用 bsp_dwt 实测（秒）而不是名义周期：任务被抢占/抖动时，PID 的 I、D 仍然对得上真实时间。
 * @note 只处理已绑定的电机：没填绑定的电机直接跳过，不会碰到空指针。
 */
extern "C" void motor_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  TickType_t wake_time = xTaskGetTickCount();

  // 时间戳初值无意义：第一次 get_delta_s() 的结果丢掉
  uint32_t mark = 0U;
  bsp_dwt.get_delta_s(&mark);

  for (;;)
  {
    // 关键：dt 的时间戳和编码器增量必须在**同一个不被任务切换打断**的窗口里取。
    // 本任务开时间片轮转，同优先级的其它任务如果正好插在 get_delta_s() 与 read_delta()
    // 之间，delta 覆盖的时间会比 dt 长 → 速度瞬间偏高，PID 输出出现孤立毛刺（编码器本身是干净的）。
    // vTaskSuspendAll() 只关闭任务切换、不关中断，窗口内 ISR 照常响应；窗口长度 ≈ 几十 µs。
    vTaskSuspendAll();
    const float dt_s = static_cast<float>(bsp_dwt.get_delta_s(&mark));

    motor1.update(dt_s);
    motor2.update(dt_s);
    motor3.update(dt_s);
    (void)xTaskResumeAll();

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
  }
}
