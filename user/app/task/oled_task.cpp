/**
 * @file oled_task.cpp
 * @brief 场地页：左侧场地示意图（出发区 1~4 + 安全区 红/蓝）+ 右侧文字与长按进度
 * @date 2026-11-27
 *
 * @details
 *
 *   画面上只有一页，128×64 全部用来显示场地信息：
 *
 *     ┌──────── 64×64 场地 ────────┬──── 右侧文字 (x=64) ────┐
 *     │ ┌──┐ ┌──┐ ┌──┐           │ 出生点 2                │
 *     │ │ 1│ │红│ │2 │           │                         │
 *     │ └──┘ └──┘ └──┘           │ 安全区 红                │
 *     │                            │                         │
 *     │                            │ ──────                  │
 *     │ ┌──┐ ┌──┐ ┌──┐           │ （长按进度 / 已发送）  │
 *     │ │ 3│ │蓝│ │4 │           │                         │
 *     │ └──┘ └──┘ └──┘           │                         │
 *     └────────────────────────────┴─────────────────────────┘
 *
 *   - 出发区：四角 16×16 框，里面只有数字；选中的整格反白；
 *   - 安全区：`红` 在 1/2 中间（上沿）、`蓝` 在 3/4 中间（下沿），都是 16×16 框，
 *     选中的那一个反白；
 *   - 右侧两行：`出生点N`、`安全区 红/蓝`；长按时最下面一行跑进度条，确认后 1 s 显示 `已发送`。
 *
 * @note 几何全部按 8 像素页对齐，所以直接写显存字节（`oled.get_gram()`），不做位偏移。
 * @note 这块屏的扫描方向是上下反的（页 0 显示在下面），所以画完在 `render()` 最后统一做一次
 *       整帧上下翻转（`mirror_frame()`）：坐标全部按“正常方向”写，字模不用再单独翻。
 * @note 没有变化就不刷屏：一次整帧刷新是 1024 B 阻塞 I²C（约 26 ms），静止时完全不发。
 * @note 本任务最低优先级，长按进度期间每 100 ms 刷一次也不影响 1 kHz 速度环。
 */

#include "app_task.hpp"

#include "board_cfg.hpp"                 // oled
#include "field_ui.hpp"                  // press_ms / sent_ms
#include "i2c.h"                         // hi2c1（没接屏时重试 init）
#include "module_cfg.hpp"                // host：出生点 / 安全区
#include "oled_font16.hpp"               // 16×16 中文与数字字模
#include "service/system/system.hpp"     // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


namespace
{

using namespace oled_font16;

/* ==================== 节拍 ==================== */

constexpr uint32_t OLED_PERIOD_MS  = 40U;   ///< 主循环节拍
constexpr uint32_t RETRY_PERIOD_MS = 5000U; ///< 没接屏时的 init 重试间隔
constexpr uint32_t BLINK_ON_MS     = 80U;   ///< 切换后第一段高亮
constexpr uint32_t BLINK_OFF_MS    = 60U;   ///< 中间熄灭
constexpr uint32_t SENT_SHOW_MS    = 1000U; ///< `已发送` 显示时长
constexpr uint8_t  OLED_I2C_ADDR   = 0x3CU;

/* ==================== 版面（像素，全部 8 对齐） ==================== */

constexpr uint8_t FIELD_W = 64U; ///< 场地示意图宽高：0~63

constexpr uint8_t CELL_X[4] = {0U, 48U, 0U, 48U}; ///< 出发区 1~4 左上角
constexpr uint8_t CELL_Y[4] = {0U, 0U, 48U, 48U};

constexpr uint8_t SAFE_RED_X  = 24U;  ///< `红`：1、2 中间（上沿）
constexpr uint8_t SAFE_RED_Y  = 0U;
constexpr uint8_t SAFE_BLUE_X = 24U;  ///< `蓝`：3、4 中间（下沿）
constexpr uint8_t SAFE_BLUE_Y = 48U;

constexpr uint8_t TEXT_X       = 64U;            ///< 右侧文字起点
constexpr uint8_t ROW_SPAWN_Y  = 8U;             ///< `出生点N`
constexpr uint8_t ROW_SAFE_Y   = 32U;            ///< `安全区 红/蓝`
constexpr uint8_t BAR_Y        = 56U;            ///< 长按进度条（4 px 高）
constexpr uint8_t BAR_BITS     = 0x0FU;          ///< 页内第 0~3 行
constexpr uint8_t SENT_Y       = 48U;            ///< `已发送`（反白）

/** @brief 一行文字用到的字模下标 */
constexpr uint8_t TXT_SPAWN[3] = {GH_CHU, GH_SHENG, GH_DIAN}; ///< 出生点
constexpr uint8_t TXT_SAFE[3]  = {GH_AN, GH_QUAN, GH_QU};     ///< 安全区
constexpr uint8_t TXT_SENT[3]  = {GH_YI, GH_FA, GH_SONG};     ///< 已发送

/* ==================== 显存操作 ==================== */

uint8_t (*g_gram)[8] = nullptr; ///< oled.get_gram() 缓存

/** @brief 8 位位序翻转（字模按 V 翻转时要逐字节反转） */
uint8_t rev8(uint8_t v)
{
  uint8_t r = 0U;
  for (uint8_t i = 0U; i < 8U; ++i)
  {
    r = static_cast<uint8_t>((r << 1) | (v & 1U));
    v = static_cast<uint8_t>(v >> 1);
  }
  return r;
}

/** @brief 画 16×16 方框（左上角 x0,y0；y0 必须是 8 的倍数） */
void frame16(uint8_t x0, uint8_t y0)
{
  const uint8_t p = static_cast<uint8_t>(y0 / 8U);
  for (uint8_t x = x0; x < (x0 + 16U); ++x)
  {
    g_gram[x][p]        = static_cast<uint8_t>(g_gram[x][p] | 0x01U);
    g_gram[x][p + 1U]   = static_cast<uint8_t>(g_gram[x][p + 1U] | 0x80U);
  }
  g_gram[x0][p]               = 0xFFU;
  g_gram[x0][p + 1U]          = 0xFFU;
  g_gram[x0 + 15U][p]         = 0xFFU;
  g_gram[x0 + 15U][p + 1U]    = 0xFFU;
}

/** @brief 16×16 全部点亮 */
void fill16(uint8_t x0, uint8_t y0)
{
  const uint8_t p = static_cast<uint8_t>(y0 / 8U);
  for (uint8_t x = x0; x < (x0 + 16U); ++x)
  {
    g_gram[x][p]      = 0xFFU;
    g_gram[x][p + 1U] = 0xFFU;
  }
}

/** @brief 矩形填充（左上 x0,y0；右下 x1,y1；两端都是 8 对齐的整页） */
void fill_box(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  for (uint8_t p = static_cast<uint8_t>(y0 / 8U); p <= static_cast<uint8_t>(y1 / 8U); ++p)
  {
    for (uint8_t x = x0; x <= x1; ++x)
    {
      g_gram[x][p] = 0xFFU;
    }
  }
}

/** @brief 点亮 y 那一行（y 必须是 8 的倍数）的一份位掩码 */
void hline(uint8_t x0, uint8_t x1, uint8_t y, uint8_t bits)
{
  const uint8_t p = static_cast<uint8_t>(y / 8U);
  for (uint8_t x = x0; x <= x1; ++x)
  {
    g_gram[x][p] = static_cast<uint8_t>(g_gram[x][p] | bits);
  }
}

/**
 * @brief 贴一个 16×16 字（左上角 x,(page*8)）
 * @param erase true = 擦黑（反白块上用），false = 点亮
 */
void blit(uint8_t x, uint8_t page, uint8_t gh, bool erase)
{
  const uint8_t(*gl)[2] = GLYPH[gh];
  for (uint8_t c = 0U; c < 16U; ++c)
  {
    const uint8_t lo = gl[c][0];
    const uint8_t hi = gl[c][1];

    if (erase)
    {
      g_gram[x + c][page]        = static_cast<uint8_t>(g_gram[x + c][page] & static_cast<uint8_t>(~lo));
      g_gram[x + c][page + 1U]   = static_cast<uint8_t>(g_gram[x + c][page + 1U] & static_cast<uint8_t>(~hi));
    }
    else
    {
      g_gram[x + c][page]        = static_cast<uint8_t>(g_gram[x + c][page] | lo);
      g_gram[x + c][page + 1U]   = static_cast<uint8_t>(g_gram[x + c][page + 1U] | hi);
    }
  }
}

/** @brief 一行 16×16 文字 */
void text(uint8_t x, uint8_t page, const uint8_t *gh, uint8_t n, bool erase = false)
{
  for (uint8_t i = 0U; i < n; ++i)
  {
    blit(static_cast<uint8_t>(x + (16U * i)), page, gh[i], erase);
  }
}

/**
 * @brief 整帧上下翻转（页序 0↔7，每字节位序反转）
 * @note 这块屏把页 0 显示在最下面，画完翻一次，坐标就能按“正常方向”写；
 *       字模也随之不用单独翻转了（之前逐字 V 翻转只能修正字形，修不了位置）。
 */
void mirror_frame()
{
  for (uint8_t x = 0U; x < 128U; ++x)
  {
    uint8_t tmp[8];
    for (uint8_t p = 0U; p < 8U; ++p)
    {
      tmp[p] = rev8(g_gram[x][7U - p]);
    }
    for (uint8_t p = 0U; p < 8U; ++p)
    {
      g_gram[x][p] = tmp[p];
    }
  }
}

/* ==================== 画面内容 ==================== */

/** @brief 一帧要画什么 */
struct View
{
  uint8_t spawn     = 1U;    ///< 1~4
  uint8_t zone      = 1U;    ///< 1 红 / 2 蓝
  uint8_t hold      = 0U;    ///< 0~100（10 的倍数）
  bool    sent      = false; ///< 显示 `已发送`
  bool    lit_spawn = true;  ///< 出发区高亮（闪烁时短暂为 false）
  bool    lit_zone  = true;  ///< 安全区高亮
};

bool operator==(const View &a, const View &b)
{
  return (a.spawn == b.spawn) && (a.zone == b.zone) && (a.hold == b.hold) && (a.sent == b.sent) &&
         (a.lit_spawn == b.lit_spawn) && (a.lit_zone == b.lit_zone);
}

bool operator!=(const View &a, const View &b)
{
  return !(a == b);
}

/** @brief 把一帧画进显存（不调用 refresh） */
void render(const View &v)
{
  oled.clear();
  g_gram = oled.get_gram();

  /* --- 场地外墙 3000×3000 --- */
  hline(0U, static_cast<uint8_t>(FIELD_W - 1U), 0U, 0x01U);                    // 上
  hline(0U, static_cast<uint8_t>(FIELD_W - 1U), static_cast<uint8_t>(FIELD_W - 1U), 0x80U); // 下
  for (uint8_t p = 0U; p < 8U; ++p)
  {
    g_gram[0][p]                      = 0xFFU;                                 // 左
    g_gram[FIELD_W - 1U][p]           = 0xFFU;                                 // 右
  }

  /* --- 出发区 1~4 --- */
  for (uint8_t i = 0U; i < 4U; ++i)
  {
    const bool lit = (i == static_cast<uint8_t>(v.spawn - 1U)) && v.lit_spawn;
    frame16(CELL_X[i], CELL_Y[i]);
    if (lit)
    {
      fill16(CELL_X[i], CELL_Y[i]);
    }
    blit(CELL_X[i], static_cast<uint8_t>(CELL_Y[i] / 8U), static_cast<uint8_t>(GH_N1 + i), lit);
  }

  /* --- 安全区：红（1/2 之间）、蓝（3/4 之间） --- */
  const bool red_on  = (v.zone == 1U) && v.lit_zone;
  const bool blue_on = (v.zone == 2U) && v.lit_zone;

  frame16(SAFE_RED_X, SAFE_RED_Y);
  if (red_on)
  {
    fill16(SAFE_RED_X, SAFE_RED_Y);
  }
  blit(SAFE_RED_X, static_cast<uint8_t>(SAFE_RED_Y / 8U), GH_HONG, red_on);

  frame16(SAFE_BLUE_X, SAFE_BLUE_Y);
  if (blue_on)
  {
    fill16(SAFE_BLUE_X, SAFE_BLUE_Y);
  }
  blit(SAFE_BLUE_X, static_cast<uint8_t>(SAFE_BLUE_Y / 8U), GH_LAN, blue_on);

  /* --- 右侧文字：出生点N / 安全区 红蓝 --- */
  text(TEXT_X, static_cast<uint8_t>(ROW_SPAWN_Y / 8U), TXT_SPAWN, 3U);
  blit(static_cast<uint8_t>(TEXT_X + 48U), static_cast<uint8_t>(ROW_SPAWN_Y / 8U),
       static_cast<uint8_t>(GH_N1 + (v.spawn - 1U)), false);

  text(TEXT_X, static_cast<uint8_t>(ROW_SAFE_Y / 8U), TXT_SAFE, 3U);
  blit(static_cast<uint8_t>(TEXT_X + 48U), static_cast<uint8_t>(ROW_SAFE_Y / 8U),
       (v.zone == 1U) ? GH_HONG : GH_LAN, false);

  /* --- 已发送 / 长按进度条 --- */
  if (v.sent)
  {
    fill_box(TEXT_X, SENT_Y, static_cast<uint8_t>(TEXT_X + 47U), static_cast<uint8_t>(SENT_Y + 15U));
    text(TEXT_X, static_cast<uint8_t>(SENT_Y / 8U), TXT_SENT, 3U, true);
  }
  else if (v.hold > 0U)
  {
    const uint8_t n = static_cast<uint8_t>((v.hold * 64U) / 100U);
    hline(TEXT_X, static_cast<uint8_t>(TEXT_X + n - 1U), BAR_Y, BAR_BITS);
  }

  mirror_frame(); // 面板上下反：提交前统一翻一次
}

/* ==================== 闪烁状态机 ==================== */

/** @brief 一次"亮 → 灭 → 定格"的闪烁 */
struct Blink
{
  uint8_t  phase = 0U; ///< 0 静止 / 1 亮 / 2 灭 / 3 定格
  uint32_t t0    = 0U;
};

void blink_step(Blink &b, bool changed, uint32_t now)
{
  if (changed)
  {
    b.phase = 1U;
    b.t0    = now;
    return;
  }

  const uint32_t dt = now - b.t0;
  if ((b.phase == 1U) && (dt >= BLINK_ON_MS))
  {
    b.phase = 2U;
    b.t0    = now;
  }
  else if ((b.phase == 2U) && (dt >= BLINK_OFF_MS))
  {
    b.phase = 3U;
  }
}

bool blink_lit(const Blink &b)
{
  return b.phase != 2U; // 静止 / 亮 / 定格 都点亮
}

/** @brief 当前毫秒数（tick → ms） */
uint32_t now_ms()
{
  return static_cast<uint32_t>(xTaskGetTickCount() * 1000U / configTICK_RATE_HZ);
}

/** @brief 长按进度：0~100，10 的倍数（没按住时 0） */
uint8_t hold_percent(uint32_t now)
{
  const uint32_t t = field_ui::press_ms;
  if (t == 0U)
  {
    return 0U;
  }

  uint32_t el = now - t;
  if (el >= field_ui::LONG_PRESS_MS)
  {
    return 100U;
  }
  return static_cast<uint8_t>(((el * 100U) / field_ui::LONG_PRESS_MS) / 10U * 10U);
}

bool sent_visible(uint32_t now)
{
  const uint32_t t = field_ui::sent_ms;
  return (t != 0U) && ((now - t) < SENT_SHOW_MS);
}

} // namespace


extern "C" void oled_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  TickType_t wake_time = xTaskGetTickCount();
  TickType_t last_try  = wake_time;
  View       shown;          // 已经画在屏上的内容
  bool       drawn_once = false; // 首帧无论内容是否等于默认值都要画一次
  Blink      blink_spawn;
  Blink      blink_zone;
  uint8_t    prev_spawn = 0xFFU;
  uint8_t    prev_zone  = 0xFFU;

  for (;;)
  {
    if (!oled.is_ready() && ((xTaskGetTickCount() - last_try) >= pdMS_TO_TICKS(RETRY_PERIOD_MS)))
    {
      last_try = xTaskGetTickCount();
      (void)oled.init(&hi2c1, OLED_I2C_ADDR);
    }

    if (oled.is_ready())
    {
      const uint32_t now = now_ms();

      const uint8_t spawn = host.spawn_id();
      const uint8_t zone  = host.safe_zone();

      blink_step(blink_spawn, (spawn != prev_spawn), now);
      blink_step(blink_zone, (zone != prev_zone), now);

      View v;
      v.spawn     = spawn;
      v.zone      = zone;
      v.hold      = hold_percent(now);
      v.sent      = sent_visible(now);
      v.lit_spawn = blink_lit(blink_spawn);
      v.lit_zone  = blink_lit(blink_zone);

      prev_spawn = spawn;
      prev_zone  = zone;

      if (!drawn_once || (v != shown))
      {
        render(v);
        (void)oled.refresh();
        shown      = v;
        drawn_once = true;
      }
    }

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(OLED_PERIOD_MS));
  }
}
