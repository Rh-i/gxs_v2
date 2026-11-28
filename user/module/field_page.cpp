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
 *     ┌──────── 64×64 场地 ──────┬──── 右侧文字 (x=64) ──────┐
 *     │ ┌──┐ ┌──┐ ┌──┐           │ 出生点 2                 │
 *     │ │ 1│ │红│ |2 │           │                         │
 *     │ └──┘ └──┘ └──┘           │ 安全区 红                │
 *     │                          │                         │
 *     │ ┌──┐ ┌──┐ ┌──┐           │ （已发送时反白）          │
 *     │ │ 3│ │蓝│ │4 │           │                         │
 *     │ └──┘ └──┘ └──┘           │                         │
 *     └──────────────────────────┴─────────────────────────┘
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

#include "device/screen/iic_oled.hpp" // IicOled

#include "module_cfg.hpp" // host：出生点 / 安全区

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


namespace
{

/* 16×16 点阵（11 个汉字 + 4 个数字）
 * 由 Noto Sans CJK 栅格化生成（阈值 90），存法与 ICBSP 的 font_8x16 相同：
 * 每列 2 字节，低字节 = 上半页（bit0 = 最上行）、高字节 = 下半页。
 * 本表**不做逐字翻转**：render() 末尾整帧上下翻一次，点阵按“屏幕正常方向”存。
 * 数字字墨 12×14、`红`/`蓝` 字墨 14×14，都已居中留边，直接贴到 16×16 格上即可。 */

enum Gh : uint8_t
{
  GH_CHU = 0, // 出
  GH_SHENG,   // 生
  GH_DIAN,    // 点
  GH_AN,      // 安
  GH_QUAN,    // 全
  GH_QU,      // 区
  GH_HONG,    // 红
  GH_LAN,     // 蓝
  GH_YI,      // 已
  GH_FA,      // 发
  GH_SONG,    // 送
  GH_N1,      // 1
  GH_N2,      // 2
  GH_N3,      // 3
  GH_N4,      // 4
  GH_COUNT
};

/** @brief 16 列 × 2 字节，共 GH_COUNT 个字（定义见文件末尾） */
extern const uint8_t GLYPH[GH_COUNT][16][2];


constexpr uint32_t BLINK_ON_MS  = 80U;   ///< 切换后第一段高亮
constexpr uint32_t BLINK_OFF_MS = 60U;   ///< 中间熄灭
constexpr uint32_t SENT_SHOW_MS = 1000U; ///< 「已发送」显示时长

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

/** @brief 贴一个 16×16 字：逐列与显存按位或（点亮）或与非（在反白块上擦字） */
void FieldPage::glyph(uint8_t x, uint8_t page, uint8_t gh, bool erase)
{
  uint8_t (*const g)[8]  = _config.oled->get_gram();
  const uint8_t (*gl)[2] = GLYPH[gh];

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

/** @brief 画一个 16×16 格子：上下两条横线 + 左右两条竖线，选中时整格反白、字为黑 */
void FieldPage::cell(uint8_t x0, uint8_t y0, uint8_t gh, bool lit)
{
  uint8_t (*const g)[8] = _config.oled->get_gram();
  const uint8_t p       = static_cast<uint8_t>(y0 / 8U);

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

/** @brief 把一帧画进显存：外墙 → 出发区 1~4 → 安全区红/蓝 → 右侧文字 → 已发送，最后整帧翻转 */
void FieldPage::render(const View &v)
{
  _config.oled->clear();
  uint8_t (*const g)[8] = _config.oled->get_gram();

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
    cell(CELL_X[i], CELL_Y[i], static_cast<uint8_t>(GH_N1 + i), (i == static_cast<uint8_t>(v.spawn - 1U)) && v.lit_spawn);
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

/** @brief 推进闪烁状态机：变化时从“亮”重新开始，亮 80 ms → 灭 60 ms → 定格 */
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

/** @brief 创建内部队列（调度器启动前调用，见 module_init） */
void FieldPage::init()
{
  configASSERT(_config.oled != nullptr);

  _sent_q = xQueueCreate(1U, sizeof(uint8_t));
  configASSERT(_sent_q != nullptr);
}

/** @brief 投一个「已发送」事件（长度 1 的队列，覆盖写，永不阻塞） */
void FieldPage::notify_sent()
{
  const uint8_t evt = 1U;
  if (_sent_q != nullptr)
  {
    (void)xQueueOverwrite(_sent_q, &evt); // 长度 1：覆盖，不阻塞也不积压
  }
}

/** @brief 一拍：取 host 快照 → 推进闪烁 → 与屏上内容逐项比较，变了才重绘 + 刷屏 */
void FieldPage::update()
{
  if (!_config.oled->is_ready())
  {
    return; // 屏没接 / 初始化失败
  }

  const TickType_t now_tick = xTaskGetTickCount();

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
  if (_drawn_once && (v.spawn == _shown.spawn) && (v.zone == _shown.zone) && (v.sent == _shown.sent) && (v.lit_spawn == _shown.lit_spawn) && (v.lit_zone == _shown.lit_zone))
  {
    return;
  }

  render(v);
  (void)_config.oled->refresh();
  _shown      = v;
  _drawn_once = true;
}

/* ==================== */


namespace
{

/* 16×16 点阵（11 个汉字 + 4 个数字）
 * 由 Noto Sans CJK 栅格化生成（阈值 90），存法与 ICBSP 的 font_8x16 相同：
 * 每列 2 字节，低字节 = 上半页（bit0 = 最上行）、高字节 = 下半页。
 * 本表**不做逐字翻转**：render() 末尾整帧上下翻一次，点阵按“屏幕正常方向”存。
 * 数字字墨 12×14、`红`/`蓝` 字墨 14×14，都已居中留边，直接贴到 16×16 格上即可。 */
const uint8_t GLYPH[GH_COUNT][16][2] = {
  {{0x00, 0x00}, {0x00, 0x7E}, {0xF8, 0x7E}, {0xF8, 0x40}, {0x80, 0x40}, {0x80, 0x40}, {0x80, 0x40}, {0xFE, 0x7F}, {0x80, 0x40}, {0x80, 0x40}, {0x80, 0x40}, {0xF8, 0x40}, {0xF8, 0xFE}, {0x00, 0xFE}, {0x00, 0x00}, {0x00, 0x00}}, // 出
  {{0x00, 0x00}, {0xC0, 0x40}, {0x70, 0x42}, {0x1C, 0x42}, {0x14, 0x42}, {0x10, 0x42}, {0x10, 0x42}, {0xFE, 0x7F}, {0x10, 0x42}, {0x10, 0x42}, {0x10, 0x42}, {0x10, 0x42}, {0x10, 0x42}, {0x10, 0x40}, {0x00, 0x00}, {0x00, 0x00}}, // 生
  {{0x00, 0x00}, {0x00, 0xC0}, {0xC0, 0x77}, {0xC0, 0x17}, {0x40, 0x04}, {0x40, 0xF4}, {0x40, 0x44}, {0x7E, 0x04}, {0x48, 0x34}, {0x48, 0xE4}, {0x48, 0x04}, {0xC8, 0x17}, {0xC8, 0x77}, {0x08, 0xC0}, {0x00, 0x00}, {0x00, 0x00}}, // 点
  {{0x00, 0x00}, {0xB8, 0x00}, {0xB8, 0xC0}, {0x88, 0x46}, {0x88, 0x6F}, {0xC8, 0x6D}, {0xEA, 0x38}, {0x8E, 0x18}, {0x88, 0x1C}, {0x88, 0x37}, {0x88, 0x23}, {0x88, 0x60}, {0xB8, 0xC0}, {0xB8, 0x40}, {0x00, 0x00}, {0x00, 0x00}}, // 安
  {{0x00, 0x00}, {0xC0, 0x40}, {0x40, 0x40}, {0xE0, 0x48}, {0xB0, 0x48}, {0x98, 0x48}, {0x8C, 0x48}, {0x86, 0x7F}, {0x8C, 0x48}, {0x98, 0x48}, {0xB0, 0x48}, {0xE0, 0x48}, {0xC0, 0x40}, {0xC0, 0x40}, {0x00, 0x00}, {0x00, 0x00}}, // 全
  {{0xFE, 0x7F}, {0xFE, 0x7F}, {0x02, 0x40}, {0x12, 0x58}, {0x32, 0x4C}, {0x62, 0x46}, {0xC2, 0x43}, {0xC2, 0x43}, {0xC2, 0x43}, {0x72, 0x47}, {0x3A, 0x4E}, {0x0A, 0x58}, {0x02, 0x40}, {0x02, 0x40}, {0x00, 0x00}, {0x00, 0x00}}, // 区
  {{0x00, 0x00}, {0x80, 0x20}, {0xE0, 0x26}, {0xF8, 0x27}, {0xDC, 0x35}, {0x60, 0x34}, {0x10, 0x20}, {0x18, 0x20}, {0x18, 0x20}, {0xF8, 0x3F}, {0xF8, 0x3F}, {0x18, 0x20}, {0x18, 0x20}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}}, // 红
  {{0x00, 0x00}, {0x00, 0x40}, {0x88, 0x43}, {0xC8, 0x7B}, {0x0C, 0x48}, {0xFC, 0x5F}, {0x08, 0x78}, {0x08, 0x4B}, {0xE8, 0x79}, {0xDC, 0x58}, {0xCC, 0x4B}, {0xC8, 0x7A}, {0x88, 0x40}, {0x00, 0x40}, {0x00, 0x00}, {0x00, 0x00}}, // 蓝
  {{0x02, 0x00}, {0xF2, 0x3F}, {0xF2, 0x7F}, {0x82, 0x41}, {0x82, 0x41}, {0x82, 0x41}, {0x82, 0x41}, {0x82, 0x41}, {0x82, 0x41}, {0x82, 0x41}, {0x86, 0x41}, {0xFE, 0x41}, {0x00, 0x60}, {0x00, 0x38}, {0x00, 0x00}, {0x00, 0x00}}, // 已
  {{0x00, 0x00}, {0x00, 0x60}, {0x30, 0x30}, {0x3C, 0x18}, {0x20, 0xCE}, {0xA0, 0xC3}, {0xFC, 0x47}, {0x3E, 0x79}, {0x20, 0x31}, {0x20, 0x39}, {0x24, 0x6D}, {0x2C, 0x47}, {0x28, 0xC1}, {0x20, 0xC0}, {0x00, 0x00}, {0x00, 0x00}}, // 发
  {{0x00, 0x00}, {0x00, 0x40}, {0xC6, 0x60}, {0xCC, 0x3F}, {0x88, 0x3F}, {0x00, 0x60}, {0x90, 0x71}, {0x9E, 0x59}, {0x9C, 0x4D}, {0xF0, 0x47}, {0x98, 0x47}, {0x9C, 0x4D}, {0x96, 0x59}, {0x90, 0x41}, {0x00, 0x00}, {0x00, 0x00}}, // 送
  {{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x08, 0x40}, {0x08, 0x40}, {0x0C, 0x40}, {0xFC, 0x7F}, {0xFC, 0x7F}, {0xFC, 0x7F}, {0x00, 0x40}, {0x00, 0x40}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}}, // 1
  {{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x08, 0x60}, {0x0C, 0x78}, {0x04, 0x7C}, {0x04, 0x7E}, {0x8C, 0x4F}, {0xFC, 0x47}, {0xF8, 0x41}, {0xF0, 0x40}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}}, // 2
  {{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x08, 0x20}, {0x0C, 0x60}, {0x84, 0x40}, {0x84, 0x40}, {0xCC, 0x61}, {0xFC, 0x77}, {0x78, 0x3F}, {0x30, 0x1E}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}}, // 3
  {{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x0F}, {0xC0, 0x0F}, {0xE0, 0x0D}, {0x78, 0x0C}, {0x7C, 0x6D}, {0xFC, 0x7F}, {0xFC, 0x7F}, {0x00, 0x0C}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}}, // 4
};

} // namespace
