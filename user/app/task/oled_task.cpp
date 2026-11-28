/**
 * @file oled_task.cpp
 * @brief OLED 任务：只给节拍 —— 画面内容、闪烁与刷屏判断都在 module/field_page
 * @date 2026-11-27
 *
 * @note 40 ms 一拍，足够跟上按键响应；`field_page.update()` 内部只在内容变化时才刷屏
 *       （一次整帧刷新是 1024 B 阻塞 I²C，约 26 ms），静止时完全不发。
 * @note 本任务最低优先级（osPriorityLow），40 ms 节拍不影响 1 kHz 速度环。
 */

#include "app_task.hpp"

#include "module_cfg.hpp" // field_page

#include "service/system/system.hpp" // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


/** @brief 主循环节拍 (ms)：40 ms 与闪烁（80/60 ms）和「已发送」1 s 的时序匹配 */
constexpr uint32_t OLED_PERIOD_MS = 40U;


extern "C" void oled_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  TickType_t wake_time = xTaskGetTickCount();

  for (;;)
  {
    field_page.update();

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(OLED_PERIOD_MS));
  }
}
