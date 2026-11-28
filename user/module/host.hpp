/**
 * @file host.hpp
 * @author Rh
 * @brief 上位机协议执行器 —— 把"收帧 / 执行指令 / 仲裁 / 上报"全部收进一个类
 * @version 0.1
 * @date 2026-11-22
 *
 * @details
 *
 * 【分层】
 *   `host_protocol.*`  纯编解码（帧格式、累加和、解析器）
 *   `host.hpp/.cpp`    本类：协议执行器 —— 独占上位机串口（USART1），把帧变成动作和应答
 *   `user/app/task/host_task.cpp`  只是个壳：阻塞等待与循环在那里，`host.process()` 做收帧与状态推进
 *
 * 【本类管的三件事】
 *   1. 指令：CMD_VEL / CMD_VEL_TIME / CMD_MOVE / CMD_SERVO / ESTOP
 *   2. 仲裁：`ControlSource`（遥控器 CH5 写、本类读）
 *   3. 场地信息：出生点 1~4 / 安全区 1=红 2=蓝（按键长按确认时直接推送）
 *
 * 【生效规则】
 *   - 只有 `ControlSource::HOST` 时速度类指令才执行，否则回 `RES_REJECTED`
 *   - 场地信息：**按键长按时直接发一帧**（`send_field_now()`，`send()` 支持多写者），
 *     `host_task` 不需要为它额外唤醒
 */

#ifndef __HOST_HPP__
#define __HOST_HPP__

#include "host_protocol.hpp" // 帧格式与消息类型

#include "FreeRTOS.h" // IWYU pragma: keep（TimerHandle_t）
#include "timers.h"

#include <stdint.h>


/**
 * @brief 底盘听谁的
 *
 * @note 优先级：**遥控器最大**——CH5 三挡：`HOST` 上位机 / `STOP` 强制停车 / `RC` 遥控；
 *       遥控器离线时由 `remote_task` 切到 `HOST`（上位机接管）。
 */
enum class ControlSource : uint8_t
{
  RC   = 0U, ///< 摇杆直接控底盘
  HOST = 1U, ///< 上位机指令控底盘
  STOP = 2U, ///< 强制停车（两边都不接受指令）
};


class Host
{
public:
  Host() = default;

  /* ==================== 对外接口（其他任务用） ==================== */

  /** @brief 设置控制源（remote_task 按 CH5 / 离线状态每拍写） */
  void set_control_source(ControlSource src);

  /** @brief 读当前控制源 */
  ControlSource control_source() const;

  /** @brief 出生点 1 → 2 → 3 → 4 → 1（按键短按调用）；只改本地值，不发送 */
  void next_spawn();

  /** @brief 安全区 1 ↔ 2（按键短按调用）；只改本地值，不发送 */
  void toggle_zone();

  /** @brief 把当前出生点/安全区直接发给上位机（按键任务长按时调用，不经过收帧循环） */
  void send_field_now();

  /** @brief 当前出生点 1~4 */
  uint8_t spawn_id() const;

  /** @brief 当前安全区 1 / 2 */
  uint8_t safe_zone() const;

  /**
   * @brief 建场地信息重发定时器（100 ms 自动重载，由 send_field_now() 启停）
   * @note 只创不定时器，不发帧：在调度器启动前由 module_init() 调用是安全的。
   */
  void init();

  /** @brief 发一帧 STATE（控制源 + 运行时长），并把 _last_src 对齐到当前值 */
  void send_state();

  /**
   * @brief 下一次等待的上限 (ms)
   * @return 到下一个计时任务（CMD_MOVE / CMD_VEL_TIME）的剩余毫秒数；
   *         没有计时任务时返回 `portMAX_DELAY`（一直睡，不空转）
   * @note `host_task` 拿它当 `receive()` 的超时：有数据立即醒，没数据就睡到点。
   */
  uint32_t wait_ms() const;

  /**
   * @brief 一次处理：收帧执行 → 排空 → 仲裁 → 计时 → 运动推进 → 变化补发
   * @param data 刚收到的字节（可空）
   * @param n    字节数（可为 0，此时只做周期事务）
   * @note 不阻塞：阻塞等待在 `host_task` 的 `receive()` 里。
   */
  void process(const uint8_t *data, size_t n);

  /* ==================== */

private:
  /* ==================== 运动任务（CMD_MOVE 直线开环） ==================== */

  /** @brief 运动任务（CMD_MOVE）的阶段 */
  enum class MovePhase : uint8_t
  {
    IDLE = 0, ///< 没有运动任务
    DRIVE,    ///< 按锁定的车体速度直线开，到时停车
  };

  /** @brief 当前运动任务的全部参数 */
  struct MotionJob
  {
    MovePhase phase    = MovePhase::IDLE;
    float     vx_body  = 0.0f; ///< 车体 x 速度 (cm/s)
    float     vy_body  = 0.0f; ///< 车体 y 速度 (cm/s)
    uint32_t  stop_ms  = 0U;   ///< 结束时刻 (ms)
    uint8_t   cmd_type = 0U;   ///< 原始指令类型（回 MOVE_DONE 用）
    uint8_t   cmd_seq  = 0U;   ///< 原始指令序号
  };

  /** @brief 定时速度指令（CMD_VEL_TIME） */
  struct TimedDrive
  {
    bool     active  = false;
    uint32_t stop_ms = 0U;
    uint8_t  cmd_seq = 0U;
  };

  /* ==================== */


  /* ==================== 收 ==================== */

  /** @brief 把收到的字节喂给解析器（凑出一帧就立即执行） */
  void _feed(const uint8_t *data, size_t n);

  /** @brief 非阻塞排空积压（timeout = 0，不等待） */
  void _drain_rx();

  /** @brief 按 TYPE 分派一帧 */
  void _handle(const host_proto::Frame &f);

  void _on_cmd_vel(const host_proto::Frame &f);
  void _on_cmd_vel_time(const host_proto::Frame &f);
  void _on_cmd_move(const host_proto::Frame &f);
  void _on_cmd_servo(const host_proto::Frame &f);
  void _on_ping();
  void _on_host_ack(const host_proto::Frame &f);
  void _on_estop(const host_proto::Frame &f);

  /* ==================== */


  /* ==================== 场地信息重发 ==================== */

  /** @brief 立即发一帧 FIELD_STATUS */
  void _send_field_once();

  /** @brief 定时器回调：发下一帧，发满就停表 */
  void _field_tick();

  /** @brief 软件定时器的静态入口 */
  static void _field_timer_cb(TimerHandle_t t);

  /* ==================== */


  /* ==================== 发 ==================== */

  void _send_frame(uint8_t type, const uint8_t *payload, uint8_t len);

  /** @brief 取下一个下行帧序号（host_task 与按键任务都会发帧，用临界区保护） */
  uint8_t _next_seq();

  void _send_ack(uint8_t acked_type, uint8_t acked_seq, uint8_t result);

  void _send_pong();
  void _send_event(uint8_t code, uint8_t data);

  /** @brief 结束当前运动任务：回 MOVE_DONE（如有任务）+ 停车 */
  void _finish_job(uint8_t result);

  /** @brief 中止正在跑的 CMD_VEL_TIME：有的话回一帧 MOVE_DONE(result)（含停车） */
  void _abort_timed(uint8_t result);

  /* ==================== */


  /* ==================== 运动 ==================== */

  void _start_move_job(const host_proto::Frame &f);
  void _motion_step();

  /* ==================== */


  /* ==================== 周期事务 ==================== */

  /** @brief 控制源不是上位机 → 中止运动任务并停车 */
  void _arbitrate();

  /** @brief CMD_VEL_TIME 到点 → 停车 + 回 MOVE_DONE */
  void _expire_timed();

  /** @brief 控制源 / 比赛信息变化时补发对应帧（不是心跳） */
  void _publish_changes();

  /* ==================== */


  /* ==================== 状态 ==================== */

  host_proto::Parser _parser;        ///< 收帧解析器（跨轮保留半帧）
  volatile uint8_t   _tx_seq = 0U;   ///< 下行帧序号（两个任务共用）
  MotionJob          _job;           ///< 当前运动任务
  TimedDrive         _timed;         ///< 当前定时速度任务

  volatile ControlSource _source = ControlSource::RC; ///< 遥控器 CH5 写、本类读
  volatile uint8_t       _spawn  = 1U;                ///< 出生点（按键写）
  volatile uint8_t       _zone   = 1U;                ///< 安全区（按键写）

  TimerHandle_t _field_timer = nullptr; ///< FIELD_STATUS 重发定时器（100 ms，自动重载）
  uint8_t       _field_left  = 0U;      ///< 还要发几帧（发满 10 帧或收到确认就归零）
  uint32_t      _field_next_ms = 0U;    ///< 下一帧的允许发送时刻（丢弃早到的回调）

  ControlSource _last_src = ControlSource::RC; ///< 用于检测控制源变化

  /* ==================== */
};

#endif // __HOST_HPP__
