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
 * @note 只依赖 ICBSP 的 `IicOled`（构造时注入指针），不直接碰 I²C；绘制全部通过
 *       `get_gram()` 写显存（几何按 8 像素页对齐，布局图见 field_page.cpp）。
 */

#ifndef __FIELD_PAGE_HPP__
#define __FIELD_PAGE_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep（TickType_t / QueueHandle_t）
#include "queue.h"
#include "task.h"

#include <stdint.h>


class IicOled; // ICBSP 的 I²C OLED 驱动，本类只用指针


class FieldPage
{
public:
  /** @brief 构造配置（唯一依赖：屏实例） */
  struct Config
  {
    IicOled *oled = nullptr; ///< 屏；init 已在 board_init() 里做过，本类只负责画
  };

  /** @param cfg 屏实例必须非空（init() 里 configASSERT 校验） */
  explicit FieldPage(const Config &cfg) : _config(cfg) {}

  /**
   * @brief 创建内部队列、校验配置
   * @note 须在调度器启动前调用（见 module_init()）：队列先就绪，key_task 才能随时投事件。
   */
  void init();

  /**
   * @brief `oled_task` 循环调用：内容有变化就重绘 + 刷屏
   * @note 屏未就绪（没接屏 / init 失败）时直接返回，不做任何 I²C 传输。
   */
  void update();

  /**
   * @brief 通知「已发送」：key_task 长按确认后调用
   * @note 只投事件、不传时间；显示起点由 update() 收到事件那一拍决定，屏上显示 1 s。
   */
  void notify_sent();

private:
  /** @brief 一次「亮 → 灭 → 定格」的切换闪烁 */
  struct Blink
  {
    uint8_t  phase = 0U; ///< 0 静止 / 1 亮 / 2 灭 / 3 定格
    uint32_t t0    = 0U; ///< 当前阶段的起始时刻 (ms)

    /**
     * @brief 推进闪烁：亮 80 ms → 灭 60 ms → 定格
     * @param changed 本轮数值是否刚变化（变了就重开一次闪烁）
     * @param now     当前时刻 (ms)
     */
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

  /**
   * @brief 贴一个 16×16 字
   * @param x     左上角列（像素 0~127）
   * @param page  左上角页（像素 y = page*8，必须是 8 的倍数）
   * @param gh    字模下标（GH_*）
   * @param erase true = 在反白块上擦字，false = 点亮
   */
  void glyph(uint8_t x, uint8_t page, uint8_t gh, bool erase);

  /**
   * @brief 画一个 16×16 格子：方框 +（选中时）整格反白 + 里面的字
   * @param x0  左上角列（像素）
   * @param y0  左上角行（像素，必须是 8 的倍数）
   * @param gh  格子里的字模下标
   * @param lit 是否选中（选中 = 整格反白、字为黑）
   */
  void cell(uint8_t x0, uint8_t y0, uint8_t gh, bool lit);

  /** @brief 把一帧画进显存（不调 refresh），末尾整帧上下翻转 */
  void render(const View &v);

  const Config _config; ///< 屏实例（构造时注入）

  View    _shown;              ///< 已经画在屏上的内容（逐项比较，没变就不刷）
  bool    _drawn_once = false; ///< 首帧必须画一次（即使恰好等于默认值）
  Blink   _blink_spawn;        ///< 出生点格子的切换闪烁
  Blink   _blink_zone;         ///< 安全区格子的切换闪烁
  uint8_t _prev_spawn = 0xFFU; ///< 上一拍的出生点（0xFF = 上电首次，必然触发闪烁）
  uint8_t _prev_zone  = 0xFFU; ///< 上一拍的安全区

  QueueHandle_t _sent_q  = nullptr; ///< 「已发送」事件队列（长度 1，写方覆盖，永不阻塞）
  uint32_t      _sent_ms = 0U;      ///< 0 = 不在显示期；否则 = 收到事件的时刻 (ms)
};

#endif // __FIELD_PAGE_HPP__
