#include "host.hpp"

#include "board_cfg.hpp"  // 上位机串口 / servo1 / servo2
#include "module_cfg.hpp" // chassis

#include "FreeRTOS.h" // IWYU pragma: keep（xTaskGetTickCount / configTICK_RATE_HZ）
#include "task.h"

#include <math.h> // hypotf


namespace
{

/* ==================== 常量 ==================== */

/**
 * @brief 发送超时 (ms)
 * @note 不能传 0。`bsp_uart` 的 TX 流缓冲区要先拿锁，`sys_task` 每 10 ms 会跑一次
 *       `tx_recover()`，抢不到锁时超时 0 会把整帧丢掉。一帧最多 56 B，921600 下不到
 *       0.6 ms，给 5 ms 足够。
 */
constexpr uint32_t HOST_TX_TIMEOUT_MS = 5U;

/** @brief 上位机所在的串口（USART1，PB6/PB7，921600）；换口只改这一行 */
auto &host_uart = bsp_uart1;

/* CMD_MOVE 的执行参数（开环近似：按“距离/速度”定时长，位置精度由上位机闭环兜底） */
constexpr float MOVE_SPEED_MIN_CM_S = 5.0f;   ///< 速度下限（再小就走不动）
constexpr float MOVE_SPEED_MAX_CM_S = 100.0f; ///< 速度上限（再快打滑严重、开环误差大）

/* FIELD_STATUS 重发：长按确认后立刻发 1 帧，之后每 100 ms 一帧，共 10 帧；收到 0xAC 就停 */
constexpr uint8_t  FIELD_PUSH_COPIES    = 10U;
constexpr uint32_t FIELD_PUSH_PERIOD_MS = 100U;

/** @brief 当前毫秒数（tick → ms） */
uint32_t now_ms()
{
  return static_cast<uint32_t>(xTaskGetTickCount() * 1000U / configTICK_RATE_HZ);
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

/**
 * @brief 把当前出生点/安全区发给上位机（按键任务长按时调用）
 *
 * @note 立即发 1 帧，随后每 100 ms 重发，最多 10 帧。上位机回一帧 `0xAC`
 *       （`acked_type = FIELD_STATUS`）就提前停止。`send()` 支持多写者，主循环不需要
 *       为它额外唤醒；重发节奏由软件定时器推进。
 */
void Host::send_field_now()
{
  if (_field_timer == nullptr)
  {
    _send_field_once(); // 定时器没建好（极端情况）：至少发一帧
    return;
  }

  // 连续长按时从第一帧重新计数：重置周期即可，早到的旧回调由 _field_next_ms 挡掉
  _send_field_once();
  _field_left    = static_cast<uint8_t>(FIELD_PUSH_COPIES - 1U);
  _field_next_ms = now_ms() + FIELD_PUSH_PERIOD_MS;
  (void)xTimerReset(_field_timer, 0U);
}

/** @brief 立即发一帧 FIELD_STATUS */
void Host::_send_field_once()
{
  const uint8_t pl[2] = {_spawn, _zone};
  _send_frame(host_proto::FIELD_STATUS, pl, sizeof(pl));
}

/** @brief 定时器回调：发下一帧，发满就停表 */
void Host::_field_tick()
{
  if (_field_left == 0U)
  {
    (void)xTimerStop(_field_timer, 0U);
    return;
  }

  // 停表前已经排进队列的回调会早到，这里丢掉，保证 100 ms 的间隔不被打乱
  const uint32_t now = now_ms();
  if (static_cast<int32_t>(now - _field_next_ms) < 0)
  {
    return;
  }

  _send_field_once();
  _field_left    = static_cast<uint8_t>(_field_left - 1U);
  _field_next_ms = now + FIELD_PUSH_PERIOD_MS;

  if (_field_left == 0U)
  {
    (void)xTimerStop(_field_timer, 0U);
  }
}

void Host::_field_timer_cb(TimerHandle_t t)
{
  static_cast<Host *>(pvTimerGetTimerID(t))->_field_tick();
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
 * @brief 阻塞收帧的主循环（事件驱动）
 *
 * @note 只有一个阻塞点：`receive()`。串口来 1 个字节就醒，不来就睡到下一个计时任务
 *       到点（`_wait_ms()`）；没有任何计时任务时睡 `portMAX_DELAY`，**不轮询也不空转**。
 * @note 顺序固定：收 → 仲裁 → 计时 → 运动推进 → 变化补发。
 */
void Host::poll()
{
  // 场地信息重发用的软件定时器（100 ms 自动重载，由 send_field_now() 启停）
  if (_field_timer == nullptr)
  {
    _field_timer = xTimerCreate("field", pdMS_TO_TICKS(FIELD_PUSH_PERIOD_MS), pdTRUE, this,
                                _field_timer_cb);
  }

  _send_state(); // 上电同步一次控制源（场地信息由按键长按直接发）

  _last_src = _source;

  for (;;)
  {
    uint8_t chunk[64];
    size_t  got = 0U;

    // 唯一的等待点：有数据 / 到点 / 永久睡
    if (host_uart.receive(chunk, sizeof(chunk), _wait_ms(), &got) == Status::IO_ERROR)
    {
      vTaskDelay(pdMS_TO_TICKS(10U)); // 串口没起来：别空转，退避一下
    }
    if (got > 0U)
    {
      _feed(chunk, got);
    }

    _drain_rx(); // 把这一轮积压的排空（非阻塞）

    _arbitrate();
    _expire_timed();
    _motion_step();
    _publish_changes();
  }
}

/* ==================== */


/* ==================== 收 ==================== */

/** @brief 把字节喂给解析器；凑出一帧就立即执行 */
void Host::_feed(const uint8_t *data, size_t n)
{
  for (size_t i = 0U; i < n; ++i)
  {
    host_proto::Frame f;
    if (_parser.push(data[i], &f))
    {
      _handle(f);
    }
  }
}

/** @brief 非阻塞排空（timeout = 0）：一次最多 64 字节，直到收不到 */
void Host::_drain_rx()
{
  uint8_t chunk[64];
  size_t  got = 0U;

  while (host_uart.receive(chunk, sizeof(chunk), 0U, &got) == Status::OK && got > 0U)
  {
    _feed(chunk, got);
  }
}

/** @brief 到下一个计时任务还有多久；没有就永久睡 */
uint32_t Host::_wait_ms() const
{
  const uint32_t now  = now_ms();
  uint32_t       wait = portMAX_DELAY;

  if (_timed.active)
  {
    wait = (static_cast<int32_t>(_timed.stop_ms - now) > 0) ? (_timed.stop_ms - now) : 1U;
  }

  if (_job.phase == MovePhase::DRIVE)
  {
    const uint32_t left =
        (static_cast<int32_t>(_job.stop_ms - now) > 0) ? (_job.stop_ms - now) : 1U;
    if ((wait == portMAX_DELAY) || (left < wait))
    {
      wait = left;
    }
  }

  return wait;
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
  case host_proto::HOST_ACK:
    _on_host_ack(f);
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
  _abort_timed(host_proto::RES_REJECTED);

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
  _abort_timed(host_proto::RES_REJECTED);

  chassis.drive(vx, vy, wz);

  _timed.active  = true;
  _timed.stop_ms = now_ms() + dur;
  _timed.cmd_seq = f.seq;

  _send_ack(f.type, f.seq, host_proto::RES_OK);
}

/** @brief 0x03 近似位置动作（按 (x,y) 直线开一段，开环） */
void Host::_on_cmd_move(const host_proto::Frame &f)
{
  if (_source != ControlSource::HOST)
  {
    _send_ack(f.type, f.seq, host_proto::RES_REJECTED);
    return;
  }

  _abort_timed(host_proto::RES_REJECTED);
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

/** @brief 0x05 PING：只回一帧 PONG（在线 + 开机时长），不附带其它信息 */
void Host::_on_ping()
{
  _send_pong();
}

/**
 * @brief 0xAC 上位机确认：只用于"收到了 FIELD_STATUS"，不回任何帧
 *
 * @note 不校验 `acked_seq`，因为上位机可能回的是第一次收到的那帧的序号。
 */
void Host::_on_host_ack(const host_proto::Frame &f)
{
  if ((f.len < 1U) || (f.payload[0] != host_proto::FIELD_STATUS))
  {
    return;
  }

  _field_left = 0U;
  if (_field_timer != nullptr)
  {
    (void)xTimerStop(_field_timer, 0U);
  }
}

/** @brief 急停：任何模式都生效，正在跑的任务都回 MOVE_DONE(REJECTED) */
void Host::_on_estop(const host_proto::Frame &f)
{
  _finish_job(host_proto::RES_REJECTED); // 有运动任务就回 MOVE_DONE + 停车
  _abort_timed(host_proto::RES_REJECTED);

  _send_event(host_proto::EV_ESTOP, 0U);
  _send_ack(f.type, f.seq, host_proto::RES_OK);
}

/* ==================== */


/* ==================== 发 ==================== */

/** @brief 所有下行帧的唯一出口：组帧 + 交给串口发送（超时 5 ms，见 HOST_TX_TIMEOUT_MS） */
void Host::_send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
  uint8_t buf[host_proto::MAX_FRAME];

  const size_t n = host_proto::pack(buf, type, _next_seq(), payload, len);
  if (n > 0U)
  {
    (void)host_uart.send(buf, n, HOST_TX_TIMEOUT_MS);
  }
}

/** @brief 下行帧序号：主循环与按键任务都会发帧，用临界区保护 */
uint8_t Host::_next_seq()
{
  taskENTER_CRITICAL();
  _tx_seq = static_cast<uint8_t>(_tx_seq + 1U);
  const uint8_t s = _tx_seq;
  taskEXIT_CRITICAL();
  return s;
}

void Host::_send_ack(uint8_t acked_type, uint8_t acked_seq, uint8_t result)
{
  uint8_t pl[3] = {acked_type, acked_seq, result};
  _send_frame(host_proto::ACK, pl, sizeof(pl));
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

/** @brief 中止正在跑的 CMD_VEL_TIME：有的话回一帧 MOVE_DONE(result) */
void Host::_abort_timed(uint8_t result)
{
  if (!_timed.active)
  {
    return;
  }

  _timed.active = false;
  chassis.stop();

  uint8_t pl[3] = {host_proto::CMD_VEL_TIME, _timed.cmd_seq, result};
  _send_frame(host_proto::MOVE_DONE, pl, sizeof(pl));
}

/* ==================== */


/* ==================== 运动 ==================== */

/**
 * @brief 解析 CMD_MOVE 并建立运动任务
 *
 * @note 载荷：`i16 x_cm` `i16 y_cm` `u16 speed_cm_s`（车体系，+x 右 / +y 前）。
 *       全向底盘不需要先转向：按 (x,y) 的方向合成车体速度，速度取指令给的
 *       `speed_cm_s`（夹到 5~100 cm/s），再按「距离 / 速度」定时长。
 *       这是**开环近似**（没有位置环/陀螺仪），位置精度由上位机闭环兜底。
 */
void Host::_start_move_job(const host_proto::Frame &f)
{
  if (f.len < 6U)
  {
    _send_ack(f.type, f.seq, host_proto::RES_BAD_FRAME);
    return;
  }

  const float x_cm = static_cast<float>(host_proto::get_i16(&f.payload[0]));
  const float y_cm = static_cast<float>(host_proto::get_i16(&f.payload[2]));

  float speed = static_cast<float>(host_proto::get_u16(&f.payload[4]));
  if (speed < MOVE_SPEED_MIN_CM_S)
  {
    speed = MOVE_SPEED_MIN_CM_S;
  }
  else if (speed > MOVE_SPEED_MAX_CM_S)
  {
    speed = MOVE_SPEED_MAX_CM_S;
  }

  const float dist = hypotf(x_cm, y_cm);

  _send_ack(f.type, f.seq, host_proto::RES_OK);

  if (dist <= 0.5f)
  {
    // 距离可忽略：不建任务，直接回完成
    uint8_t pl[3] = {f.type, f.seq, host_proto::RES_OK};
    _send_frame(host_proto::MOVE_DONE, pl, sizeof(pl));
    return;
  }

  _job.phase    = MovePhase::DRIVE;
  _job.vx_body  = x_cm / dist * speed;
  _job.vy_body  = y_cm / dist * speed;
  _job.stop_ms  = now_ms() + static_cast<uint32_t>((dist / speed) * 1000.0f);
  _job.cmd_type = f.type;
  _job.cmd_seq  = f.seq;
}

/** @brief 运动状态机：按锁定的车体速度直线开，到时停车并回 MOVE_DONE */
void Host::_motion_step()
{
  if (_job.phase == MovePhase::IDLE)
  {
    return;
  }

  // 用有符号差值比较，tick 回绕（约 49.7 天）时也能正确判定
  if (static_cast<int32_t>(now_ms() - _job.stop_ms) >= 0)
  {
    _finish_job(host_proto::RES_OK);
    return;
  }

  chassis.drive(_job.vx_body, _job.vy_body, 0.0f);
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
  _abort_timed(host_proto::RES_REJECTED);
}

/** @brief CMD_VEL_TIME 到点：停车并回 MOVE_DONE */
void Host::_expire_timed()
{
  if (!_timed.active || (static_cast<int32_t>(now_ms() - _timed.stop_ms) < 0))
  {
    return;
  }

  _timed.active = false;
  chassis.stop();

  uint8_t pl[3] = {host_proto::CMD_VEL_TIME, _timed.cmd_seq, host_proto::RES_OK};
  _send_frame(host_proto::MOVE_DONE, pl, sizeof(pl));
}

/**
 * @brief 控制源变化时补发 STATE（不是心跳；场地信息由按键直接发）
 */
void Host::_publish_changes()
{
  if (_source != _last_src)
  {
    _last_src = _source;
    _send_state();
  }
}

/* ==================== */
