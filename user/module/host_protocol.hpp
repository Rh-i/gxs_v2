/**
 * @file host_protocol.hpp
 * @author Rh
 * @brief 上下位机串口协议（USART10 / 921600）—— 帧格式与编解码
 * @version 0.1
 * @date 2026-11-22
 *
 * @details
 *
 * 【帧格式】（上下行通用）
 *
 *     0xAA 0x55 | VER | TYPE | SEQ | LEN | PAYLOAD ... | SUM
 *       帧头     版本   类型   序号  长度     载荷        累加和(低 8 位)
 *
 *   - `SUM` = 从 `VER` 起、到 `PAYLOAD` 最后一个字节的**无符号 8 位累加和**（不含两个帧头字节）
 *   - `LEN <= MAX_PAYLOAD`，越界帧直接丢弃并重新找帧头
 *   - 多字节数值一律**小端**；角度单位 0.1°、速度 cm/s、角速度 mrad/s
 *
 * 【本文件只做编解码】不碰串口、不碰任务；收发与调度在 `host_task.cpp`，
 * 协议说明给上位机的那份在 `user/module/上位机协议.md`。
 */

#ifndef __HOST_PROTOCOL_HPP__
#define __HOST_PROTOCOL_HPP__

#include <stddef.h>
#include <stdint.h>


namespace host_proto
{

/* ==================== 帧常量 ==================== */

constexpr uint8_t SOF1 = 0xAAU; ///< 帧头 1
constexpr uint8_t SOF2 = 0x55U; ///< 帧头 2
constexpr uint8_t VER  = 0x01U; ///< 协议版本

constexpr size_t MAX_PAYLOAD = 48U;                         ///< 载荷上限（超过视为错误帧）
constexpr size_t OVERHEAD    = 7U;                          ///< AA 55 VER TYPE SEQ LEN SUM
constexpr size_t MAX_FRAME   = OVERHEAD + MAX_PAYLOAD;      ///< 一整帧的字节上限

/* ==================== */


/* ==================== 消息类型 ==================== */

enum Type : uint8_t
{
  /* ---- 上位机 → 车 ---- */
  CMD_VEL      = 0x01U, ///< i16 vx, i16 vy, i16 wz（车体系，cm/s、cm/s、mrad/s）
  CMD_VEL_TIME = 0x02U, ///< 同 CMD_VEL + u16 dur_ms：跑够时长自动停
  CMD_MOVE     = 0x03U, ///< i16 x_cm, i16 y_cm, i16 yaw_0p1deg, u8 ref（0 车体 / 1 场地）
  CMD_SERVO    = 0x04U, ///< u8 id(1/2), u16 angle_0p1deg（0~2700）
  PING         = 0x05U, ///< 可选 u32 时间戳（回 PONG 时原样带回）
  CMD_TASK     = 0x06U, ///< u8 task_id, u8 arg（任务 1/2/3 预留）
  ESTOP        = 0xFFU, ///< 急停，可省理由字节

  /* ---- 车 → 上位机 ---- */
  ACK          = 0x00U, ///< u8 acked_type, u8 acked_seq, u8 result
  FIELD_STATUS = 0x01U, ///< u8 spawn_id(1~4), u8 safe_zone(1/2)
  STATE        = 0x02U, ///< u8 source, u16 fault, u32 uptime_ms（**只在控制源变化 / 上电时发**）
  IMU          = 0x03U, ///< **预留**（本固件不发）i16 yaw_0p1deg, i16 wz_mrad_s
  MOTORS       = 0x04U, ///< **预留**（本固件不发）3 × (i16 target, i16 actual, i16 out_0p1pct)
  /* 0x05 原 SERVO 反馈：舵机无位置反馈，不传输（上位机自己记目标） */
  EVENT        = 0x06U, ///< u8 code, u8 data
  HELLO        = 0x07U, ///< u8 ver, u8 fw_major, u8 fw_minor
  MOVE_DONE    = 0x08U, ///< u8 cmd_type, u8 cmd_seq, u8 result
  PONG         = 0x0FU, ///< u32 uptime_ms（回 PING）
};

/** @brief ACK / MOVE_DONE 的结果码 */
enum Result : uint8_t
{
  RES_OK          = 0U, ///< 已接受
  RES_BAD_FRAME   = 1U, ///< 长度 / 累加和不对（一般不会走到，解析器已过滤）
  RES_UNSUPPORTED = 2U, ///< 不认识的消息类型
  RES_REJECTED    = 3U, ///< 当前模式不允许（例如没切到上位机模式就发速度指令）
};

/** @brief EVENT 的 code（用到新事件时再加） */
enum EventCode : uint8_t
{
  EV_ESTOP = 2U, ///< 收到急停
};

/* ==================== */


/* ==================== 编解码 ==================== */

/** @brief 组一帧，返回实际字节数（载荷超长返回 0） */
size_t pack(uint8_t *out, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len);

/** @brief 小端写入 */
void put_i16(uint8_t *p, int16_t v);
void put_u16(uint8_t *p, uint16_t v);
void put_u32(uint8_t *p, uint32_t v);

/** @brief 小端读取 */
int16_t  get_i16(const uint8_t *p);
uint16_t get_u16(const uint8_t *p);
uint32_t get_u32(const uint8_t *p);

/** @brief 解析出来的一帧 */
struct Frame
{
  uint8_t type = 0U;
  uint8_t seq  = 0U;
  uint8_t len  = 0U;
  uint8_t payload[MAX_PAYLOAD] = {};
};

/**
 * @brief 逐字节解析器：喂一个字节，凑齐一帧且校验通过时返回 true
 *
 * @note 失败就丢弃 1 个字节重新同步（不会整段丢掉后面的好帧）；
 *       `LEN` 超过 `MAX_PAYLOAD` 或版本不符同样只丢 1 字节。
 */
class Parser
{
public:
  /** @brief 清空内部缓冲（串口重建后调用） */
  void reset();

  /** @brief 喂一个字节；out 非空且返回 true 时 out 填入完整一帧 */
  bool push(uint8_t byte, Frame *out);

private:
  uint8_t _buf[MAX_FRAME] = {}; ///< 待解析字节
  size_t  _n              = 0U; ///< 已缓存字节数

  /** @brief 丢掉前 k 个字节，剩下的前移 */
  void _drop(size_t k);
};

/* ==================== */

} // namespace host_proto

#endif // __HOST_PROTOCOL_HPP__
