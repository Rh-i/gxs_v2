/**
 * @file remote_task.cpp
 * @brief 遥控任务：R9DS 通道 → 底盘速度，含模式切换、小陀螺模式与失联保护
 * @date 2026-11-21
 *
 * 通道表（Rc 语义值 -100~+100，开关给挡位）：
 *   CH1 右摇杆上下 → 自旋（仅普通模式；小陀螺模式下忽略），正 = 左旋（俯视逆时针）
 *   CH3 左摇杆上下 → 前进/后退，正 = 前进
 *   CH4 左摇杆左右 → 左右平移，正 = 左移
 *   CH5 C 三挡开关 → **控制源仲裁**：+100 上位机 / 0 强制停车 / -100 遥控（遥控器优先）
 *   — 遥控器**离线**（关掉/拔线）时控制权自动交给上位机，不再刹车锁死。
 *   CH6 左旋钮   → 舵机1 角度：0 = 中位 135°，正 = 增大，负 = 减小
 *   CH7 背面左推杆 → 小陀螺模式的自转速度：0 = 不转，正 = 顺时针，负 = 逆时针
 *   CH8 右旋钮   → 舵机2 角度（映射同 CH6）
 *   CH9/CH10 B/A：A（CH10）+100 = 小陀螺模式，-100 = 普通模式；B 预留
 *
 * 小陀螺模式（A = +100）：车体持续自转，同时 CH3/CH4 平移可用。平移方向以**场地系**为准：
 *   进入该模式的瞬间把当时的 yaw 记为 0°，之后用 (yaw - yaw0) 把摇杆向量反旋回车体系，
 *   所以车体怎么转，“推前进”始终走同一条直线（经典小陀螺）。
 *   - 用 cos/sin 直接算，yaw 的 ±180° 回绕不影响结果，不需要把角度展开成连续值。
 *   - IMU 离线时自动退化成车体系（不反旋），不会因为没数据而不动。
 *   - 轮速预算：平移太大会和自转叠加顶到单轮限幅（那样方向会失真），所以先用
 *     chassis.translation_budget() 算剩余预算，超出就等比缩平移，保自转。
 *
 * 安全约束：链路不可用或 CH5 不在遥控挡时一律刹车，不复用上一拍的摇杆值。
 * 本任务独占 `r9ds.update()`，不能再有第二个消费者。
 */

#include "app_task.hpp"

#include "board_cfg.hpp"             // r9ds、wt102
#include "module_cfg.hpp"            // chassis / host / ControlSource
#include "service/system/system.hpp" // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <math.h> // sinf / cosf / sqrtf
#include <stdint.h>


constexpr uint32_t REMOTE_PERIOD_MS  = 10U;    // 100 Hz：摇杆变化远慢于此，再快只是重复写目标
constexpr float    REMOTE_V_MAX_CM_S = 120.0f; // 摇杆打满 = 120 cm/s
// 自转满杆取「轮速与平移满杆一致」：ω_max = V_MAX / R = 120 / 10，R 见 module_cfg.cpp
constexpr float   REMOTE_W_MAX_RAD_S = 12.0f;
constexpr int16_t STICK_DEADBAND     = 5; // 遥控器语义值 ±100 的 5%，滤掉中位零漂

/** @brief 小陀螺模式的自转速度上限 (rad/s)：左推杆推满就是这个值（约 1.3 圈/s） */
constexpr float SPIN_W_MAX_RAD_S = 8.0f;

/** @brief 舵机映射：旋钮 0 = 中位，±100 打满 270° 行程 */
constexpr float SERVO_MID_DEG        = 135.0f; ///< 270° 舵机中位
constexpr float SERVO_HALF_RANGE_DEG = 135.0f; ///< 半行程：±100 → ±135°
constexpr int16_t KNOB_DEADBAND      = 3;      ///< 旋钮回中的零漂死区

/** @brief yaw 原始值 → 弧度：int16 满量程 32768 对应 180° */
constexpr float YAW_RAW_TO_RAD = 3.1415926f / 32768.0f;

/**
 * @brief yaw 正方向与「俯视逆时针」的符号关系
 *
 * @note 现场校验：A 抬起进入小陀螺（CH6 给个中速），推 CH3 前进看轨迹：
 *       - 走直线 → 保持当前值
 *       - 走成圆弧/圆（公转；补偿方向反了，误差翻倍）→ 改成相反值
 * @note 2026-11-21 本车实测：原值 -1 时推前进会**公转**（车体一边转一边绕圈），
 *       说明 WT102 的 yaw 随车体逆时针转动而**增大**，补偿方向要取正 → 改成 +1。
 */
constexpr float SPIN_YAW_SIGN = 1.0f;


/** @brief 摇杆 → -1.0~+1.0，中位死区内直接给 0 */
static float stick(int16_t value)
{
  if ((value <= STICK_DEADBAND) && (value >= -STICK_DEADBAND))
    return 0.0f;

  return static_cast<float>(value) / 100.0f;
}

/** @brief 左旋钮 / 右旋钮（-100~+100）→ 舵机角度：0 = 中位，正 = 增大，负 = 减小 */
static float knob_to_servo_deg(int16_t knob)
{
  if ((knob <= KNOB_DEADBAND) && (knob >= -KNOB_DEADBAND))
    return SERVO_MID_DEG;

  float k = static_cast<float>(knob) / 100.0f;
  if (k > 1.0f)
    k = 1.0f;
  else if (k < -1.0f)
    k = -1.0f;

  return SERVO_MID_DEG + (k * SERVO_HALF_RANGE_DEG);
}

/**
 * @brief 背面左推杆（-100~+100）→ 自转角速度：0 = 不转，正 = 顺时针，负 = 逆时针
 *
 * @note 底盘约定 ω 俯视逆时针为正，所以「顺时针」要取负号。
 */
static float spin_rate(int16_t slider)
{
  if ((slider <= STICK_DEADBAND) && (slider >= -STICK_DEADBAND))
    return 0.0f;

  float s = static_cast<float>(slider);
  if (s > 100.0f)
    s = 100.0f;
  else if (s < -100.0f)
    s = -100.0f;

  return -s * (SPIN_W_MAX_RAD_S / 100.0f);
}


extern "C" void remote_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  TickType_t wake_time = xTaskGetTickCount();

  bool    spin_mode = false; ///< 上一拍的 A 开关状态（只用来抓「刚进入」那一拍）
  int16_t yaw_entry = 0;     ///< 进入小陀螺那一刻的 yaw，场地系的 0°

  /** 上一拍的控制源：只在「切换」时刹一次车，避免 100 Hz 重复 stop() */
  ControlSource prev_src = ControlSource::RC;

  for (;;)
  {
    r9ds.update(); // 本任务是该串口唯一的接收消费者

    // 电机测试进行中（motor_test_wave ≠ 0 且确实有电机在测）：本任务不碰底盘，否则这里的
    // chassis.stop() 会以 100 Hz 把 motor_monitor_task 写进去的测试目标 / 开环占空比冲掉。
    // 注意 motor_monitor_index ≥ 3 时监控任务不驱动任何电机，此时不应屏蔽遥控。
    if ((motor_test_wave != 0U) && (motor_monitor_index < 3U))
    {
      spin_mode = false;
      vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(REMOTE_PERIOD_MS));
      continue;
    }

    // 遥控器关掉 / 拔线 / 连续丢帧：**控制权交给上位机**（上位机不发指令时车就保持停着）
    if (!r9ds.is_online())
    {
      if (prev_src != ControlSource::HOST)
      {
        chassis.stop(); // 丢掉遥控器最后给的速度，不会带着残余速度冲出去
        prev_src = ControlSource::HOST;
      }
      host.set_control_source(ControlSource::HOST);
      spin_mode = false;
      vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(REMOTE_PERIOD_MS));
      continue;
    }

    const auto  &rc   = r9ds.get_rc();
    const int8_t mode = static_cast<int8_t>(rc.switch_c); // -1 遥控 / 0 强制停车 / +1 上位机

    // 控制源仲裁：遥控器永远优先——CH5 直接指定底盘听谁（状态存于 host）
    const ControlSource src = (mode > 0) ? ControlSource::HOST
                                         : ((mode < 0) ? ControlSource::RC : ControlSource::STOP);
    host.set_control_source(src);

    if (src != prev_src)
    {
      chassis.stop(); // 切换控制源：先停一次，不把上一源的速度带进新源
      prev_src = src;
    }

    if (src != ControlSource::RC)
    {
      // 上位机挡 / 停车挡：舵机也不由旋钮控（上位机挡下归 host_task）
      spin_mode = false;
      vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(REMOTE_PERIOD_MS));
      continue;
    }

    // 遥控挡：两个旋钮控舵机（PWM 由 servo_task 推进）
    servo1.set_angle(knob_to_servo_deg(rc.knob_left));
    servo2.set_angle(knob_to_servo_deg(rc.knob_right));

    // 遥控挡：摇杆 → 场地系平移（底盘约定 v_x 右正、v_y 前正）
    const float fwd  = stick(rc.left_vertical);   // +1 前进
    const float left = stick(rc.left_horizontal); // +1 左移

    float vx = -left * REMOTE_V_MAX_CM_S;
    float vy = fwd * REMOTE_V_MAX_CM_S;

    if (static_cast<int8_t>(rc.switch_a) > 0)
    {
      /* ---------- 小陀螺模式：自转 + 场地系平移 ---------- */
      if (!spin_mode)
      {
        yaw_entry = wt102.get_z_newest().yaw; // 上升沿：此刻车头方向 = 场地系 0°
        spin_mode = true;
      }

      const float w = spin_rate(rc.slider_left); // 背面左推杆：0 不转 / 正顺时针 / 负逆时针

      // 把「场地系」摇杆向量反旋回车体系：v_body = R(-Δyaw) · v_field
      if (wt102.is_online())
      {
        const int16_t yaw_now = wt102.get_z_newest().yaw; // 只取单个 int16，读上一帧也够用
        const int16_t dyaw    = static_cast<int16_t>(yaw_now - yaw_entry);
        const float   a       = static_cast<float>(dyaw) * YAW_RAW_TO_RAD * SPIN_YAW_SIGN;
        const float   c    = cosf(a);
        const float   s    = sinf(a);

        const float vx_body = (vx * c) + (vy * s);
        const float vy_body = (-vx * s) + (vy * c);
        vx                  = vx_body;
        vy                  = vy_body;
      }
      // IMU 离线：不反旋，退化成车体系

      // 轮速预算：平移太大会和自转叠加顶到单轮限幅、把方向拧歪 → 超了就等比缩平移
      const float budget = chassis.translation_budget(w);
      const float mag    = sqrtf((vx * vx) + (vy * vy));
      if ((mag > budget) && (mag > 0.001f))
      {
        const float k = budget / mag;
        vx *= k;
        vy *= k;
      }

      chassis.drive(vx, vy, w);
    }
    else
    {
      /* ---------- 普通模式：车体系平移 + CH1 手动自旋 ---------- */
      spin_mode = false;

      const float ccw = stick(rc.right_vertical); // +1 左旋
      chassis.drive(vx, vy, ccw * REMOTE_W_MAX_RAD_S);
    }

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(REMOTE_PERIOD_MS));
  }
}
