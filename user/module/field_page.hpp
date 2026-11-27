/**
 * @file field_page.hpp
 * @author Rh
 * @brief 场地页 —— 左侧场地示意图（出发区 1~4 + 安全区 红/蓝）+ 右侧文字与「已发送」
 * @version 0.1
 * @date 2026-11-27
 *
 * @note 画面布局与绘制全在本类（见 field_page.cpp），`oled_task` 只按 40 ms 节拍调 `update()`。
 * @note 数据来源：`host`（出生点 / 安全区）+ `notify_sent()`（长按确认事件，内部走长度 1 的队列）。
 * @note 没有变化就不刷屏：一次整帧刷新是 1024 B 阻塞 I²C（约 26 ms），静止时完全不发。
 */

#ifndef __FIELD_PAGE_HPP__
#define __FIELD_PAGE_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep（TickType_t / QueueHandle_t）
#include "queue.h"
#include "task.h"

#include <stdint.h>


class FieldPage
{
public:
  /** @brief 创建内部队列（须在调度器启动前调，见 module_init） */
  void init();

  /** @brief 主循环调用：没接屏时每 5 s 重试一次 init；内容有变化就重绘 + 刷屏 */
  void update();

  /** @brief 通知「已发送」：key_task 长按确认后调用，屏上从收到那拍起显示 1 s */
  void notify_sent();

private:
  /** @brief 一次「亮 → 灭 → 定格」的切换闪烁 */
  struct Blink
  {
    uint8_t  phase = 0U; ///< 0 静止 / 1 亮 / 2 灭 / 3 定格
    uint32_t t0    = 0U;

    /** @brief 检测到变化就重开一次闪烁，否则按时间推进 */
    void step(bool changed, uint32_t now);
  };

  /** @brief 一帧要画什么（与屏上内容逐项比较，全等就不刷） */
  struct View
  {
    uint8_t spawn     = 1U;    ///< 出生点 1~4
    uint8_t zone      = 1U;    ///< 1 红 / 2 蓝
    bool    sent      = false; ///< 显示「已发送」
    bool    lit_spawn = true;  ///< 闪烁时短暂为 false
    bool    lit_zone  = true;  ///< 同上
  };

  /** @brief 贴一个 16×16 字（左上角 x,(page*8)；erase = 在反白块上擦字） */
  void glyph(uint8_t x, uint8_t page, uint8_t gh, bool erase);

  /** @brief 画一个 16×16 格子：方框 +（选中时）整格反白 + 里面的字 */
  void cell(uint8_t x0, uint8_t y0, uint8_t gh, bool lit);

  /** @brief 把一帧画进显存（不调 refresh），末尾整帧上下翻转 */
  void render(const View &v);

  View       _shown;
  bool       _drawn_once = false;
  Blink      _blink_spawn;
  Blink      _blink_zone;
  uint8_t    _prev_spawn = 0xFFU;
  uint8_t    _prev_zone  = 0xFFU;
  TickType_t _last_try   = 0U;

  QueueHandle_t _sent_q  = nullptr; ///< 「已发送」事件队列（长度 1，写方覆盖，永不阻塞）
  uint32_t      _sent_ms = 0U;      ///< 0 = 不在显示期；否则 = 收到事件的时刻 (ms)
};

#endif // __FIELD_PAGE_HPP__
