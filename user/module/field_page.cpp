/**
 * @file field_page.cpp
 * @author Rh
 * @brief 场地页绘制
 * @version 0.1
 * @date 2026-11-27
 *
 * @details
 *
 *   画面只有一页，128×64 全部用来显示场地信息：
 *
 *     ┌──────── 64×64 场地 ────────┬──── 右侧文字 (x=64) ────┐
 *     │ ┌──┐ ┌──┐ ┌──┐           │ 出生点 2                │
 *     │ │ 1│ │红│ │2 │           │                         │
 *     │ └──┘ └──┘ └──┘           │ 安全区 红                │
 *     │                            │                         │
 *     │ ┌──┐ ┌──┐ ┌──┐           │ （已发送时反白）        │
 *     │ │ 3│ │蓝│ │4 │           │                         │
 *     │ └──┘ └──┘ └──┘           │                         │
 *     └────────────────────────────┴─────────────────────────┘
 *
 *   - 出发区：四角 16×16 框（数字 1~4），选中的整格反白；
 *   - 安全区：红在 1/2 之间上沿、蓝在 3/4 之间下沿，选中的反白；
 *   - 右侧：`出生点N`、`安全区 红|蓝`，长按确认后 1 s 显示反白的 `已发送`。
 *
 * @note 几何全部按 8 像素页对齐，直接写显存字节（`oled.get_gram()`），不做位偏移。
 * @note 这块屏扫描方向上下反（页 0 显示在最下面），所以 render() 最后整帧翻一次，
 *       坐标全部按“正常方向”写，字模不用单独翻。
 */

#include "field_page.hpp"

#include "board_cfg.hpp"   // oled
#include "i2c.h"           // hi2c1（没接屏时重试 init）
#include "module_cfg.hpp"  // host：出生点 / 安全区
#include "oled_font16.hpp" // 16×16 中文与数字字模

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


namespace
{

using namespace oled_font16;

constexpr uint32_t RETRY_PERIOD_MS = 5000U; ///< 没接屏时的 init 重试间隔
constexpr uint32_t BLINK_ON_MS     = 80U;   ///< 切换后第一段高亮
constexpr uint32_t BLINK_OFF_MS    = 60U;   ///< 中间熄灭
constexpr uint32_t SENT_SHOW_MS    = 1000U; ///< 「已发送」显示时长
constexpr uint8_t  OLED_I2C_ADDR   = 0x3CU;

constexpr uint8_t FIELD_W = 64U; ///< 场地示意图宽高：0~63

constexpr uint8_t CELL_X[4] = {0U, 48U, 0U, 48U}; ///< 出发区 1~4 左上角
constexpr uint8_t CELL_Y[4] = {0U, 0U, 48U, 48U};

constexpr uint8_t SAFE_RED_X  = 24U; ///< 红：1、2 中间（上沿）
constexpr uint8_t SAFE_RED_Y  = 0U;
constexpr uint8_t SAFE_BLUE_X = 24U; ///< 蓝：3、4 中间（下沿）
constexpr uint8_t SAFE_BLUE_Y = 48U;

constexpr uint8_t TEXT_X      = 64U; ///< 右侧文字起点
constexpr uint8_t ROW_SPAWN_P = 1U;  ///< `出生点N` 所在页（y = 8）
constexpr uint8_t ROW_SAFE_P  = 4U;  ///< `安全区` 所在页（y = 32）
constexpr uint8_t SENT_P      = 6U;  ///< `已发送` 所在页（y = 48）

} // namespace


/* ==================== 绘制 ==================== */

void FieldPage::glyph(uint8_t x, uint8_t page, uint8_t gh, bool erase)
{
  uint8_t (*const g)[8] = oled.get_gram();
  const uint8_t(*gl)[2] = GLYPH[gh];

  for (uint8_t c = 0U; c < 16U; ++c)
  {
    if (erase)
    {
      g[x + c][page]      = static_cast<uint8_t>(g[x + c][page] & static_cast<uint8_t>(~gl[c][0]));
      g[x + c][page + 1U] = static_cast<uint8_t>(g[x + c][page + 1U] & static_cast<uint8_t>(~gl[c][1]));
    }
    else
    {
      g[x + c][page]      = static_cast<uint8_t>(g[x + c][page] | gl[c][0]);
      g[x + c][page + 1U] = static_cast<uint8_t>(g[x + c][page + 1U] | gl[c][1]);
    }
  }
}

void FieldPage::cell(uint8_t x0, uint8_t y0, uint8_t gh, bool lit)
{
  uint8_t (*const g)[8] = oled.get_gram();
  const uint8_t p = static_cast<uint8_t>(y0 / 8U);

  for (uint8_t x = x0; x < (x0 + 16U); ++x) // 上下两条边（选中时整格反白）
  {
    g[x][p]      = lit ? 0xFFU : static_cast<uint8_t>(g[x][p] | 0x01U);
    g[x][p + 1U] = lit ? 0xFFU : static_cast<uint8_t>(g[x][p + 1U] | 0x80U);
  }
  for (uint8_t q = p; q < (p + 2U); ++q) // 左右两条边
  {
    g[x0][q]       = 0xFFU;
    g[x0 + 15U][q] = 0xFFU;
  }

  glyph(x0, p, gh, lit); // 反白块上擦字
}

void FieldPage::render(const View &v)
{
  oled.clear();
  uint8_t (*const g)[8] = oled.get_gram();

  // 场地外墙 64×64：上下各一行，左右各一列
  for (uint8_t x = 0U; x < FIELD_W; ++x)
  {
    g[x][0]                 = static_cast<uint8_t>(g[x][0] | 0x01U);
    g[x][FIELD_W / 8U - 1U] = static_cast<uint8_t>(g[x][FIELD_W / 8U - 1U] | 0x80U);
  }
  for (uint8_t p = 0U; p < 8U; ++p)
  {
    g[0][p]            = 0xFFU;
    g[FIELD_W - 1U][p] = 0xFFU;
  }

  // 出发区 1~4：选中的（且闪烁未熄灭）整格反白
  for (uint8_t i = 0U; i < 4U; ++i)
  {
    cell(CELL_X[i], CELL_Y[i], static_cast<uint8_t>(GH_N1 + i),
         (i == static_cast<uint8_t>(v.spawn - 1U)) && v.lit_spawn);
  }

  // 安全区：红在 1/2 之间（上沿），蓝在 3/4 之间（下沿）
  cell(SAFE_RED_X, SAFE_RED_Y, GH_HONG, (v.zone == 1U) && v.lit_zone);
  cell(SAFE_BLUE_X, SAFE_BLUE_Y, GH_LAN, (v.zone == 2U) && v.lit_zone);

  // 右侧两行：出生点N / 安全区 红|蓝
  const uint8_t row_spawn[4] = {GH_CHU, GH_SHENG, GH_DIAN, static_cast<uint8_t>(GH_N1 + (v.spawn - 1U))};
  const uint8_t row_safe[4]  = {GH_AN, GH_QUAN, GH_QU, static_cast<uint8_t>((v.zone == 1U) ? GH_HONG : GH_LAN)};
  for (uint8_t i = 0U; i < 4U; ++i)
  {
    glyph(static_cast<uint8_t>(TEXT_X + (16U * i)), ROW_SPAWN_P, row_spawn[i], false);
    glyph(static_cast<uint8_t>(TEXT_X + (16U * i)), ROW_SAFE_P, row_safe[i], false);
  }

  // 已发送：48×16 反白块上擦字
  if (v.sent)
  {
    for (uint8_t q = SENT_P; q < (SENT_P + 2U); ++q)
    {
      for (uint8_t x = TEXT_X; x < (TEXT_X + 48U); ++x)
      {
        g[x][q] = 0xFFU;
      }
    }
    const uint8_t row_sent[3] = {GH_YI, GH_FA, GH_SONG};
    for (uint8_t i = 0U; i < 3U; ++i)
    {
      glyph(static_cast<uint8_t>(TEXT_X + (16U * i)), SENT_P, row_sent[i], true);
    }
  }

  // 面板上下反（页 0 显示在最下面）：整帧翻一次，坐标就能按正常方向写
  for (uint8_t x = 0U; x < 128U; ++x)
  {
    uint8_t tmp[8];
    for (uint8_t p = 0U; p < 8U; ++p)
    {
      uint8_t s = g[x][7U - p];
      uint8_t r = 0U;
      for (uint8_t i = 0U; i < 8U; ++i) // 逐位翻转
      {
        r = static_cast<uint8_t>((r << 1) | (s & 1U));
        s = static_cast<uint8_t>(s >> 1);
      }
      tmp[p] = r;
    }
    for (uint8_t p = 0U; p < 8U; ++p)
    {
      g[x][p] = tmp[p];
    }
  }
}


/* ==================== */

/* ==================== 闪烁 ==================== */

void FieldPage::Blink::step(bool changed, uint32_t now)
{
  if (changed)
  {
    phase = 1U;
    t0    = now;
    return;
  }

  const uint32_t dt = now - t0;
  if ((phase == 1U) && (dt >= BLINK_ON_MS))
  {
    phase = 2U;
    t0    = now;
  }
  else if ((phase == 2U) && (dt >= BLINK_OFF_MS))
  {
    phase = 3U;
  }
}


/* ==================== */

/* ==================== 调度 ==================== */

void FieldPage::init()
{
  _sent_q = xQueueCreate(1U, sizeof(uint8_t));
  configASSERT(_sent_q != nullptr);
}

void FieldPage::notify_sent()
{
  const uint8_t evt = 1U;
  if (_sent_q != nullptr)
  {
    (void)xQueueOverwrite(_sent_q, &evt); // 长度 1：覆盖，不阻塞也不积压
  }
}

void FieldPage::update()
{
  const TickType_t now_tick = xTaskGetTickCount();

  // 没接屏（或接线错）：每 5 s 重试一次 init，成功前不画
  if (!oled.is_ready())
  {
    if ((now_tick - _last_try) >= pdMS_TO_TICKS(RETRY_PERIOD_MS))
    {
      _last_try   = now_tick;
      _drawn_once = false;
      (void)oled.init(&hi2c1, OLED_I2C_ADDR);
    }
    if (!oled.is_ready())
    {
      return;
    }
  }

  const uint32_t now   = static_cast<uint32_t>(now_tick * 1000U / configTICK_RATE_HZ);
  const uint8_t  spawn = host.spawn_id();
  const uint8_t  zone  = host.safe_zone();

  // 「已发送」以收到事件的那一拍为起点（key_task 只发事件，不碰时钟和显示状态）
  uint8_t evt;
  if ((_sent_q != nullptr) && (xQueueReceive(_sent_q, &evt, 0U) == pdTRUE))
  {
    _sent_ms = now;
  }

  // 出生点 / 安全区变化时闪一下（亮 → 灭 → 定格）
  _blink_spawn.step(spawn != _prev_spawn, now);
  _blink_zone.step(zone != _prev_zone, now);
  _prev_spawn = spawn;
  _prev_zone  = zone;

  View v;
  v.spawn     = spawn;
  v.zone      = zone;
  v.sent      = (_sent_ms != 0U) && ((now - _sent_ms) < SENT_SHOW_MS);
  v.lit_spawn = _blink_spawn.phase != 2U; // 只有“灭”那一段不亮
  v.lit_zone  = _blink_zone.phase != 2U;

  // 内容没变就不刷：一次整帧刷新是 1024 B 阻塞 I²C
  if (_drawn_once && (v.spawn == _shown.spawn) && (v.zone == _shown.zone) && (v.sent == _shown.sent) &&
      (v.lit_spawn == _shown.lit_spawn) && (v.lit_zone == _shown.lit_zone))
  {
    return;
  }

  render(v);
  (void)oled.refresh();
  _shown      = v;
  _drawn_once = true;
}

/* ==================== */
