/**
 * @file field_ui.hpp
 * @author Rh
 * @brief 按键 → 场地页 UI 的共享时间戳（key_task 写，oled_task 读）
 * @date 2026-11-27
 *
 * @details
 *   `BspKey` 只给 PRESS / SHORT / LONG 三个事件，画长按进度条还需要"从什么时候开始按"，
 *   所以由 `key_task`（唯一的按键轮询者）把时刻记在这里，OLED 任务按需读取。
 *
 *   - `press_ms` 非 0 = 有按键按住，值为"按下那一刻"的毫秒数（已扣除 200 ms 消抖延迟，
 *     好让进度条和 `BspKey` 的 LONG 时刻对齐）；
 *   - `sent_ms` 非 0 = 最近一次长按确认的时刻，用来显示 1 s 的 `已发送`。
 */

#ifndef __FIELD_UI_HPP__
#define __FIELD_UI_HPP__

#include <stdint.h>


namespace field_ui
{

/** @brief 长按判定时间 (ms)：与 board_cfg.hpp 的 long_press_cnt × 轮询周期 一致 */
constexpr uint32_t LONG_PRESS_MS = 600U;

/** @brief 消抖延迟 (ms)：debounce_cnt × 轮询周期，PRESS 事件比真实按下晚这么久 */
constexpr uint32_t DEBOUNCE_MS = 200U;

inline volatile uint32_t press_ms = 0U; ///< 0 = 没有按住；否则 = 按下时刻 (ms)
inline volatile uint32_t sent_ms  = 0U; ///< 0 = 还没发过；否则 = 上次确认时刻 (ms)

} // namespace field_ui

#endif // __FIELD_UI_HPP__
