#include "host.hpp"

#include "board_cfg.hpp"  // bsp_uart10 / servo1 / servo2 / wt102
#include "module_cfg.hpp" // chassis

#include "FreeRTOS.h" // IWYU pragma: keep（xTaskGetTickCount / configTICK_RATE_HZ）
#include "task.h"

#include <math.h> // atan2f / hypotf / fabsf


namespace
{

/* ==================== 常量 ==================== */

/**
 * @brief 阻塞等数据的超时 (ms)
 * @note 串口一来数据就立即处理；超时只是兜底，用来回来推进 CMD_MOVE 状态机
 */
constexpr uint32_t HOST_WAIT_MS = 10U;

constexpr uint8_t FW_MAJOR = 0U; ///< HELLO 帧里的固件版本（上位机判兼容）
constexpr uint8_t FW_MINOR = 1U;

constexpr float YAW_RAW_TO_RAD = 3.14159265f / 32768.0f;              ///< WT102 原始 yaw → 弧度
constexpr float DEG0P1_TO_RAD  = 0.1f * 3.14159265f / 180.0f;         ///< 0.1° → 弧度

/* CMD_MOVE 的执行参数（近似执行） */
constexpr float    MOVE_SPEED_CM_S = 40.0f;  ///< 直行速度
constexpr float    TURN_W_MAX      = 3.0f;   ///< 原地转向的最大角速度 (rad/s)
constexpr float    TURN_KP         = 2.0f;   ///< 转向 P 增益 (1/s)
constexpr float    TURN_TOL_RAD    = 0.035f; ///< 到位阈值 ≈ 2°
constexpr uint32_t TURN_TIMEOUT_MS = 3000U;  ///< 单段转向超时，防止原地磨蹭

/** @brief 当前毫秒数（tick → ms） */
uint32_t now_ms()
{
  return static_cast<uint32_t>(xTaskGetTickCount() * 1000U / configTICK_RATE_HZ);
}

/** @brief 角度归一化到 (-π, π] */
float wrap_pi(float a)
{
  while (a > 3.14159265f)
  {
    a -= 6.28318531f;
  }
  while (a < -3.14159265f)
  {
    a += 6.28318531f;
  }
  return a;
}

/**
 * @brief 当前航向 (rad)
 *
 * @note 直接读 wt102 的最新帧：只取 **单个 int16 字段**，M7 上 16 位对齐读写是原子的，
 *       所以并发 update() 时读到的要么是上一帧、要么是新一帧，不会撕裂。
 *       ⚠ 不要在这里改成读多个字段 / 整个 ZData —— 那才会真的出现“新 yaw + 旧 gyro”。
 */
float yaw_now()
{
  return static_cast<float>(wt102.get_z_newest().yaw) * YAW_RAW_TO_RAD;
}

} // namespace

/* ==================== */


/* ==================== 对外接口 ==================== */

void Host::set_control_source(ControlSource src)
{
  _source = src;
}

ControlSource Host::control_source() const
{
  return _source;
}

void Host::next_spawn()
{
  _spawn = static_cast<uint8_t>((_spawn >= 4U) ? 1U : (_spawn + 1U));
}

void Host::toggle_zone()
{
  _zone = (_zone == 1U) ? 2U : 1U;
}

void Host::push_field()
{
  _field_dirty = true;
}

uint8_t Host::spawn_id() const
{
  return _spawn;
}

uint8_t Host::safe_zone() const
{
  return _zone;
}

/* ==================== */


/* ==================== 主循环 ==================== */

/**
 * @brief 阻塞收帧的主循环
 *
 * @note 顺序固定：收 → 仲裁 → 计时 → 运动推进 → 变化补发。
 *       没有 `vTaskDelayUntil`：串口有数据就立刻处理，`_drain_rx()` 的超时只用于回到这里推进状态机。
 */
void Host::poll()
{
  _send_hello(); // 上电握手：版本
  _send_state(); // 上电同步一次控制源（场地信息不主动发，按键改了才发）

  _last_src = _source;

  for (;;)
  {
    _drain_rx();
    _arbitrate();
    _expire_timed();
    _motion_step();
    _publish_changes();
  }
}

/* ==================== */


/* ==================== 收 ==================== */

/** @brief 阻塞等数据，把积压的字节全部喂解析器；凑出一帧就立刻执行 */
void Host::_drain_rx()
{
  uint8_t  chunk[64];
  size_t   got     = 0U;
  uint32_t wait_ms = HOST_WAIT_MS;

  while (bsp_uart10.receive(chunk, sizeof(chunk), wait_ms, &got) == Status::OK && got > 0U)
  {
    wait_ms = 0U; // 后续只排空，不再等待

    for (size_t i = 0U; i < got; ++i)
    {
      host_proto::Frame f;
      if (_parser.push(chunk[i], &f))
      {
        _handle(f);
      }
    }
  }
}

/** @brief 按 TYPE 分派一帧（会动的指令都在各自的 _on_xxx 里先做权限判断） */
void Host::_handle(const host_proto::Frame &f)
{
  switch (f.type)
  {
  case host_proto::CMD_VEL:
    _on_cmd_vel(f);
    break;
  case host_proto::CMD_VEL_TIME:
    _on_cmd_vel_time(f);
    break;
  case host_proto::CMD_MOVE:
    _on_cmd_move(f);
    break;
  case host_proto::CMD_SERVO:
    _on_cmd_servo(f);
    break;
  case host_proto::PING:
    _on_ping();
    break;
  case host_proto::CMD_TASK:
    // 任务 1/2/3 预留：只回不支持
    _send_ack(f.type, f.seq, host_proto::RES_UNSUPPORTED);
    break;
  case host_proto::ESTOP:
    _on_estop(f);
    break;
  default:
    _send_ack(f.type, f.seq, host_proto::RES_UNSUPPORTED);
    break;
  }
}

/* ==================== */


/* ==================== 指令 ==================== */

/** @brief 0x01 持续速度（车体系，不处理计时） */
void Host::_on_cmd_vel(const host_proto::Frame &f)
{
  if (f.len < 6U)
  {
    _send_ack(f.type, f.seq, host_proto::RES_BAD_FRAME);
    return;
  }

  if (_source != ControlSource::HOST)
  {
    // 100 Hz 持续流：被拒时最多 5 Hz 回一次 ACK，否则拒绝应答自己就吃掉 1 kB/s
    static uint32_t last_reject_ms = 0U;
    const uint32_t  t              = now_ms();

    if ((t - last_reject_ms) >= 200U)
    {
      last_reject_ms = t;
      _send_ack(f.type, f.seq, host_proto::RES_REJECTED);
    }
    return;
  }

  const float vx = static_cast<float>(host_proto::get_i16(&f.payload[0]));
  const float vy = static_cast<float>(host_proto::get_i16(&f.payload[2]));
  const float wz = static_cast<float>(host_proto::get_i16(&f.payload[4])) / 1000.0f; // mrad/s → rad/s

  if (_job.phase != MovePhase::IDLE)
  {
    _finish_job(host_proto::RES_REJECTED); // 新指令抢占：结束旧任务
  }
  _timed.active = false;

  chassis.drive(vx, vy, wz);
}

/** @brief 0x02 持续速度 + 时长（到点自动停并回 MOVE_DONE） */
void Host::_on_cmd_vel_time(const host_proto::Frame &f)
{
  if (f.len < 8U)
  {
    _send_ack(f.type, f.seq, host_proto::RES_BAD_FRAME);
    return;
  }
  if (_source != ControlSource::HOST)
  {
    _send_ack(f.type, f.seq, host_proto::RES_REJECTED);
    return;
  }

  const float    vx  = static_cast<float>(host_proto::get_i16(&f.payload[0]));
  const float    vy  = static_cast<float>(host_proto::get_i16(&f.payload[2]));
  const float    wz  = static_cast<float>(host_proto::get_i16(&f.payload[4])) / 1000.0f;
  const uint16_t dur = host_proto::get_u16(&f.payload[6]);

  if (_job.phase != MovePhase::IDLE)
  {
    _finish_job(host_proto::RES_REJECTED);
  }

  chassis.drive(vx, vy, wz);

  _timed.active  = true;
  _timed.stop_ms = now_ms() + dur;
  _timed.cmd_seq = f.seq;

  _send_ack(f.type, f.seq, host_proto::RES_OK);
}

/** @brief 0x03 近似位置动作（转向 → 直行 → 转向） */
void Host::_on_cmd_move(const host_proto::Frame &f)
{
  if (_source != ControlSource::HOST)
  {
    _send_ack(f.type, f.seq, host_proto::RES_REJECTED);
    return;
  }

  _timed.active = false;
  if (_job.phase != MovePhase::IDLE)
  {
    _finish_job(host_proto::RES_REJECTED);
  }

  _start_move_job(f);
}

/** @brief 0x04 舵机目标角（只写目标，PWM 由 servo_task 推进） */
void Host::_on_cmd_servo(const host_proto::Frame &f)
{
  if (f.len < 3U)
  {
    _send_ack(f.type, f.seq, host_proto::RES_BAD_FRAME);
    return;
  }
  if (_source != ControlSource::HOST)
  {
    _send_ack(f.type, f.seq, host_proto::RES_REJECTED);
    return;
  }

  const uint8_t id    = f.payload[0];
  const float   angle = static_cast<float>(host_proto::get_u16(&f.payload[1])) * 0.1f; // 0~2700 → 0~270°

  if (id == 1U)
  {
    servo1.set_angle(angle);
    _send_ack(f.type, f.seq, host_proto::RES_OK);
  }
  else if (id == 2U)
  {
    servo2.set_angle(angle);
    _send_ack(f.type, f.seq, host_proto::RES_OK);
  }
  else
  {
    _send_ack(f.type, f.seq, host_proto::RES_UNSUPPORTED);
  }
}

/** @brief 0x05 PING：回 PONG（开机时长）+ 当前控制源 + 场地信息 */
void Host::_on_ping()
{
  _send_pong();
  _send_state();
  _send_field();
}

/** @brief 0xFF 急停：任何模式都生效 */
void Host::_on_estop(const host_proto::Frame &f)
{
  _timed.active = false;
  _job.phase    = MovePhase::IDLE;
  chassis.stop();

  _send_event(host_proto::EV_ESTOP, 0U);
  _send_ack(f.type, f.seq, host_proto::RES_OK);
}

/* ==================== */


/* ==================== 发 ==================== */

/** @brief 所有下行帧的唯一出口：组帧 + 交给串口 DMA（超时 0 = 忙就丢，不阻塞） */
void Host::_send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
  uint8_t buf[host_proto::MAX_FRAME];

  const size_t n = host_proto::pack(buf, type, ++_tx_seq, payload, len);
  if (n > 0U)
  {
    (void)bsp_uart10.send(buf, n, 0U);
  }
}

void Host::_send_ack(uint8_t acked_type, uint8_t acked_seq, uint8_t result)
{
  uint8_t pl[3] = {acked_type, acked_seq, result};
  _send_frame(host_proto::ACK, pl, sizeof(pl));
}

void Host::_send_hello()
{
  uint8_t pl[3] = {host_proto::VER, FW_MAJOR, FW_MINOR};
  _send_frame(host_proto::HELLO, pl, sizeof(pl));
}

void Host::_send_pong()
{
  uint8_t pl[4];
  host_proto::put_u32(pl, now_ms());
  _send_frame(host_proto::PONG, pl, sizeof(pl));
}

void Host::_send_state()
{
  uint8_t pl[7];
  pl[0] = static_cast<uint8_t>(_source);
  host_proto::put_u16(&pl[1], 0U); // fault_bits：暂未使用
  host_proto::put_u32(&pl[3], now_ms());
  _send_frame(host_proto::STATE, pl, sizeof(pl));
}

void Host::_send_field()
{
  uint8_t pl[2] = {_spawn, _zone};
  _send_frame(host_proto::FIELD_STATUS, pl, sizeof(pl));
}

void Host::_send_event(uint8_t code, uint8_t data)
{
  uint8_t pl[2] = {code, data};
  _send_frame(host_proto::EVENT, pl, sizeof(pl));
}

void Host::_finish_job(uint8_t result)
{
  if (_job.phase != MovePhase::IDLE)
  {
    uint8_t pl[3] = {_job.cmd_type, _job.cmd_seq, result};
    _send_frame(host_proto::MOVE_DONE, pl, sizeof(pl));
  }

  _job.phase = MovePhase::IDLE;
  chassis.stop();
}

/* ==================== */


/* ==================== 运动 ==================== */

/**
 * @brief 解析 CMD_MOVE 并建立运动任务
 *
 * 载荷：`i16 x_cm` `i16 y_cm` `i16 yaw_0p1deg` `u8 ref`
 *   - `ref = 0` 车体系：方向相对**当前车头**，yaw 也是相对当前车头的转角
 *   - `ref = 1` 场地系：方向与 yaw 都是绝对量（WT102 yaw 的原点为 0°）
 */
void Host::_start_move_job(const host_proto::Frame &f)
{
  if (f.len < 7U)
  {
    _send_ack(f.type, f.seq, host_proto::RES_BAD_FRAME);
    return;
  }

  const float   x_cm    = static_cast<float>(host_proto::get_i16(&f.payload[0]));
  const float   y_cm    = static_cast<float>(host_proto::get_i16(&f.payload[2]));
  const float   cmd_0p1 = static_cast<float>(host_proto::get_i16(&f.payload[4]));
  const uint8_t ref     = f.payload[6];

  const float dist = hypotf(x_cm, y_cm);
  // 底盘约定：+x 右、+y 前、ω 俯视逆时针为正 ⇒ 逆时针角 θ 的方向向量是 (-sinθ, cosθ)
  const float dir = atan2f(-x_cm, y_cm);

  const uint32_t now       = now_ms();
  const float    yaw_start = yaw_now();
  const float    base      = (ref == 0U) ? yaw_start : 0.0f;

  _job.phase            = MovePhase::TURN1;
  _job.heading1         = wrap_pi(base + dir);
  _job.heading2         = (ref == 0U) ? wrap_pi(yaw_start + (cmd_0p1 * DEG0P1_TO_RAD))
                                      : (cmd_0p1 * DEG0P1_TO_RAD);
  _job.vx_body          = 0.0f;
  _job.vy_body          = MOVE_SPEED_CM_S;
  _job.straight_dur_ms  = static_cast<uint32_t>((dist / MOVE_SPEED_CM_S) * 1000.0f);
  _job.straight_stop_ms = now;
  _job.turn_start_ms    = now;
  _job.cmd_type         = f.type;
  _job.cmd_seq          = f.seq;

  _send_ack(f.type, f.seq, host_proto::RES_OK);

  if (dist <= 0.5f)
  {
    // 不移动，只转到目标朝向
    _job.heading1        = _job.heading2;
    _job.straight_dur_ms = 0U;
  }
}

/** @brief 原地转向一拍：P 控制 + 限幅 + 超时兜底；返回 true 表示可以进入下一段 */
bool Host::_turn_step(float target_rad, uint32_t t_ms)
{
  const float err = wrap_pi(target_rad - yaw_now());

  if (fabsf(err) <= TURN_TOL_RAD)
  {
    return true;
  }

  float w = TURN_KP * err;
  if (w > TURN_W_MAX)
  {
    w = TURN_W_MAX;
  }
  else if (w < -TURN_W_MAX)
  {
    w = -TURN_W_MAX;
  }

  chassis.drive(0.0f, 0.0f, w);

  return ((t_ms - _job.turn_start_ms) > TURN_TIMEOUT_MS); // 超时也算到位，避免原地磨蹭
}

/** @brief 运动状态机：每轮推进一步（TURN1 → STRAIGHT → TURN2 → 完成） */
void Host::_motion_step()
{
  if (_job.phase == MovePhase::IDLE)
  {
    return;
  }

  const uint32_t t_ms = now_ms();

  switch (_job.phase)
  {
  case MovePhase::TURN1:
    if (_turn_step(_job.heading1, t_ms))
    {
      // 车头已朝向运动方向：直行期间锁存车体速度 (0, +v)
      _job.vx_body          = 0.0f;
      _job.vy_body          = MOVE_SPEED_CM_S;
      _job.straight_stop_ms = t_ms + _job.straight_dur_ms;
      _job.turn_start_ms    = t_ms;

      _job.phase = (_job.straight_dur_ms == 0U) ? MovePhase::TURN2 : MovePhase::STRAIGHT;
    }
    break;

  case MovePhase::STRAIGHT:
    if (t_ms >= _job.straight_stop_ms)
    {
      _job.phase         = MovePhase::TURN2;
      _job.turn_start_ms = t_ms;
    }
    else
    {
      chassis.drive(_job.vx_body, _job.vy_body, 0.0f);
    }
    break;

  case MovePhase::TURN2:
    if (_turn_step(_job.heading2, t_ms))
    {
      _finish_job(host_proto::RES_OK);
    }
    break;

  default:
    _job.phase = MovePhase::IDLE;
    break;
  }
}

/* ==================== */


/* ==================== 周期事务 ==================== */

/** @brief 控制源不是上位机（遥控器抢回 / 强制停车）：中止运动任务，别让车继续跑 */
void Host::_arbitrate()
{
  if (_source == ControlSource::HOST)
  {
    return;
  }

  if (_job.phase != MovePhase::IDLE)
  {
    _finish_job(host_proto::RES_REJECTED);
  }
  if (_timed.active)
  {
    _timed.active = false;
    chassis.stop();
  }
}

/** @brief CMD_VEL_TIME 到点：停车并回 MOVE_DONE */
void Host::_expire_timed()
{
  if (!_timed.active || (now_ms() < _timed.stop_ms))
  {
    return;
  }

  _timed.active = false;
  chassis.stop();

  uint8_t pl[3] = {host_proto::CMD_VEL_TIME, _timed.cmd_seq, host_proto::RES_OK};
  _send_frame(host_proto::MOVE_DONE, pl, sizeof(pl));
}

/**
 * @brief 只在值变化时补发
 *
 * @note 场地信息只在 `push_field()`（按键长按）时置脏、由本轮补发；
 *       其余情况不主动发，上位机要就发 `PING`（`_on_ping()` 会回 FIELD_STATUS）。
 */
void Host::_publish_changes()
{
  if (_source != _last_src)
  {
    _last_src = _source;
    _send_state();
  }

  if (_field_dirty)
  {
    _field_dirty = false;
    _send_field();
  }
}

/* ==================== */
