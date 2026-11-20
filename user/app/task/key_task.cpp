/**
 * @file key_task.cpp
 * @brief 按键任务：200 ms 轮询 key1 / key2，把 PRESS / SHORT / LONG 事件打到调试串口
 * @date 2026-11-21
 *
 * @note 轮询周期与消抖/长按阈值一起决定手感（BspKey：时间 = 计数 × 周期）：
 *       本工程 key1/key2 配的是 debounce_cnt = 1、long_press_cnt = 5，
 *       在 200 ms 轮询下即 **200 ms 消抖 / 1 s 长按**。改周期时记得同步改 board_cfg.hpp。
 *
 * @note 本任务是 key1 / key2 唯一的轮询者（BspKey 非线程安全）。
 *       板载 LED 由 app_default_task 每 500 ms 翻转，本任务不碰它。
 */

#include "app_task.hpp"

#include "board_cfg.hpp"             // key1 / key2 / print_uart(bsp_uart1)
#include "bsp/bsp_key.hpp"           // BspKey::Event
#include "service/system/system.hpp" // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


/** @brief 轮询周期 (ms)：与 board_cfg.hpp 里 debounce_cnt / long_press_cnt 的取值配套 */
constexpr uint32_t KEY_TASK_PERIOD_MS = 200U;


/* ==================== 调试观察量（GDB：print key1_events / key2_events） ==================== */

static volatile uint32_t key1_events = 0U; ///< key1 产生的事件总数
static volatile uint32_t key2_events = 0U; ///< key2 产生的事件总数

/* ==================== */


/* ==================== 内部辅助 ==================== */

/** @brief 把事件打成一行（人按按钮的频率很低，直接 printf 不会淹掉串口） */
static void report(const char *name, BspKey::Event ev)
{
  switch (ev)
  {
  case BspKey::Event::PRESS:
    (void)print_uart("[key] %s press\r\n", name);
    break;
  case BspKey::Event::SHORT:
    (void)print_uart("[key] %s short\r\n", name);
    break;
  case BspKey::Event::LONG:
    (void)print_uart("[key] %s long\r\n", name);
    break;
  default:
    break;
  }
}

/* ==================== */


/* ==================== 任务 ==================== */

extern "C" void key_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  TickType_t wake_time = xTaskGetTickCount();

  for (;;)
  {
    const BspKey::Event e1 = key1.update(); // 本任务是这两个按键唯一的轮询者
    const BspKey::Event e2 = key2.update();

    if (e1 != BspKey::Event::NONE)
    {
      key1_events = key1_events + 1U; // volatile 上不用 ++（C++20 起弃用）
      report("KEY1", e1);
    }

    if (e2 != BspKey::Event::NONE)
    {
      key2_events = key2_events + 1U;
      report("KEY2", e2);
    }

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(KEY_TASK_PERIOD_MS));
  }
}

/* ==================== */
