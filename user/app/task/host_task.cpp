/**
 * @file host_task.cpp
 * @brief 上下位机任务壳：等运行态后跑 `Host` 的收帧主循环
 * @date 2026-11-22
 *
 * @note 循环与阻塞等待都写在这里（`receive()`），单次处理在 `Host::process()`。
 */

#include "app_task.hpp"

#include "board_cfg.hpp"             // bsp_uart1（上位机串口）
#include "module_cfg.hpp"            // Host host
#include "service/system/system.hpp" // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"


/* ==================== 任务 ==================== */

extern "C" void host_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  host.send_state(); // 上电同步一次控制源（场地信息由按键长按直接发）

  for (;;)
  {
    uint8_t chunk[64];
    size_t  got = 0U;

    // 唯一的阻塞点：有数据就醒；没数据就睡到下一个计时任务到点（无计时任务则永久睡）
    if (bsp_uart1.receive(chunk, sizeof(chunk), host.wait_ms(), &got) == Status::IO_ERROR)
    {
      vTaskDelay(pdMS_TO_TICKS(10U)); // 串口没起来：退避，别空转
    }

    host.process(chunk, got); // 收帧执行 + 排空 + 仲裁 / 计时 / 运动推进 / 变化补发
  }
}

/* ==================== */
