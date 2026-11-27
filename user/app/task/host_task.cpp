/**
 * @file host_task.cpp
 * @brief 上下位机任务壳：等运行态后把控制权交给 `Host::poll()`
 * @date 2026-11-22
 *
 * @note 具体逻辑全在 `user/module/host.cpp`：
 *       收帧 / 指令执行 / 控制源仲裁 / 场地信息 / CMD_MOVE 状态机。
 *       本文件只负责"库初始化失败就不跑"和"把任务时间让给它"。
 *
 * @note `poll()` 内部是阻塞在 `host_uart.receive()` 上的死循环，不会返回。
 */

#include "app_task.hpp"

#include "module_cfg.hpp"            // Host host
#include "service/system/system.hpp" // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"


/* ==================== 任务 ==================== */

extern "C" void host_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  host.poll(); // 永不返回：串口有数据就处理，超时回来推进运动状态机
}

/* ==================== */
