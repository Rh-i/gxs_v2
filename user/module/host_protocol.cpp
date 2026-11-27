#include "host_protocol.hpp"

#include <string.h> // memmove / memcpy


namespace host_proto
{

/* ==================== 编解码 ==================== */

/**
 * @brief 组帧：AA 55 | TYPE | SEQ | LEN | payload | SUM | 0D 0A
 * @return 实际字节数；载荷超长返回 0
 */
size_t pack(uint8_t *out, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len)
{
  if (out == nullptr)
    return 0U;
  if (len > MAX_PAYLOAD)
    return 0U;
  if ((len > 0U) && (payload == nullptr))
    return 0U;

  out[0] = SOF1;
  out[1] = SOF2;
  out[2] = type;
  out[3] = seq;
  out[4] = len;

  if (len > 0U)
  {
    memcpy(&out[5], payload, len);
  }

  // 累加和：从 TYPE 起到载荷末尾（不含两个帧头字节）
  uint8_t sum = 0U;
  for (size_t i = 2U; i < (5U + len); ++i)
  {
    sum = static_cast<uint8_t>(sum + out[i]);
  }
  out[5U + len] = sum;
  out[6U + len] = EOF1;
  out[7U + len] = EOF2;

  return 8U + len;
}

void put_i16(uint8_t *p, int16_t v)
{
  p[0] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFFU);
  p[1] = static_cast<uint8_t>(static_cast<uint16_t>(v) >> 8);
}

void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFFU);
  p[1] = static_cast<uint8_t>(v >> 8);
}

void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFFU);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFFU);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFFU);
}

int16_t get_i16(const uint8_t *p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

uint16_t get_u16(const uint8_t *p)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t get_u32(const uint8_t *p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

/* ==================== */
/* ==================== 解析器 ==================== */

void Parser::reset()
{
  _n = 0U;
}

void Parser::_drop(size_t k)
{
  if (k >= _n)
  {
    _n = 0U;
    return;
  }

  memmove(_buf, &_buf[k], _n - k);
  _n -= k;
}

/**
 * @brief 喂字节 → 尝试切出一帧
 *
 * @note 解析是「找帧头 → 等长度 → 校验」的循环：任何一步失败都只丢 1 字节重来，
 *       这样误入的随机数据不会连带吃掉后面的好帧。
 */
bool Parser::push(uint8_t byte, Frame *out)
{
  if (_n >= MAX_FRAME)
  {
    _drop(1U); // 极端情况：这个字节挤进来之前先把最老的扔掉
  }
  _buf[_n] = byte;
  ++_n;

  while (true)
  {
    // 1. 找帧头：不匹配就叫丢前 1 字节
    if (_n >= 1U && _buf[0] != SOF1)
    {
      _drop(1U);
      continue;
    }
    if (_n >= 2U && _buf[1] != SOF2)
    {
      _drop(1U);
      continue;
    }

    // 2. 帧头 + 头部字段还没凑齐
    if (_n < 5U)
      return false;

    // 3. 长度检查
    const uint8_t len = _buf[4];
    if (len > MAX_PAYLOAD)
    {
      _drop(1U);
      continue;
    }

    const size_t total = 8U + len; // 头部 5 + 载荷 + 累加和 1 + 帧尾 2
    if (_n < total)
      return false;

    // 4. 累加和：从 TYPE 起到载荷末尾
    uint8_t sum = 0U;
    for (size_t i = 2U; i < (5U + len); ++i)
    {
      sum = static_cast<uint8_t>(sum + _buf[i]);
    }

    if ((sum != _buf[5U + len]) || (_buf[6U + len] != EOF1) || (_buf[7U + len] != EOF2))
    {
      _drop(1U);
      continue;
    }

    // 5. 成功：填帧并移走这一帧
    if (out != nullptr)
    {
      out->type = _buf[2];
      out->seq  = _buf[3];
      out->len  = len;
      if (len > 0U)
      {
        memcpy(out->payload, &_buf[5], len);
      }
    }
    _drop(total);
    return true;
  }
}

/* ==================== */

} // namespace host_proto
