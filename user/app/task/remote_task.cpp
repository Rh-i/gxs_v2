/**
 * @file remote_task.cpp
 * @brief 遥控任务：R9DS 通道 → 底盘速度，含控制源仲裁与失联保护
 * @date 2026-11-21
 *
 * 通道表（Rc 语义值 -100~+100，开关给挡位）：
 *   CH1 右摇杆上下 → 自旋，正 = 左旋（俯视逆时针）
 *   CH3 左摇杆上下 → 前进/后退，正 = 前进
 *   CH4 左摇杆左右 → 左右平移，正 = 左移
 *   CH5 C 三挡开关 → **控制源仲裁**：+100 上位机 / 0 强制停车 / -100 遥控（遥控器优先）
 *   — 遥控器**离线**（关掉/拔线）时控制权自动交给上位机，不再刹车锁死。
 *   CH6 左旋钮   → 舵机1 角度：0 = 中位 135°，正 = 增大，负 = 减小
 *   CH8 右旋钮   → 舵机2 角度（映射同 CH6）
 *
 * @note 原来的「小陀螺模式」（CH7/CH9/CH10 + 陀螺仪 yaw 反旋）已随 WT102 一起删除，
 *       现在只有车体系平移 + CH1 手动自旋。
 *
 * 安全约束：链路不可用或 CH5 不在遥控挡时一律刹车，不复用上一拍的摇杆值。
 * 本任务独占 `r9ds.update()`，不能再有第二个消费者。
 */

#include "app_task.hpp"

#include "board_cfg.hpp"             // r9ds / servo1 / servo2
#include "module_cfg.hpp"            // chassis / host / ControlSource
#include "service/system/system.hpp" // sys_state

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


constexpr uint32_t REMOTE_PERIOD_MS  = 10U;    // 100 Hz：摇杆变化远慢于此，再快只是重复写目标
constexpr float    REMOTE_V_MAX_CM_S = 120.0f; // 摇杆打满 = 120 cm/s
// 自转满杆取「轮速与平移满杆一致」：ω_max = V_MAX / R = 120 / 10，R 见 module_cfg.cpp
constexpr float   REMOTE_W_MAX_RAD_S = 12.0f;
constexpr int16_t STICK_DEADBAND     = 5; // 遥控器语义值 ±100 的 5%，滤掉中位零漂

/** @brief 舵机映射：旋钮 0 = 中位，±100 打满 270° 行程 */
constexpr float SERVO_MID_DEG        = 135.0f; ///< 270° 舵机中位
constexpr float SERVO_HALF_RANGE_DEG = 135.0f; ///< 半行程：±100 → ±135°
constexpr int16_t KNOB_DEADBAND      = 3;      ///< 旋钮回中的零漂死区


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


extern "C" void remote_task(void *argument)
{
  (void)argument;
  sys_state.wait_running(); // 初始化失败时任务自行退出，不跑在半初始化的系统上

  TickType_t wake_time = xTaskGetTickCount();

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
      vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(REMOTE_PERIOD_MS));
      continue;
    }

    // 遥控挡：两个旋钮控舵机（PWM 由 servo_task 推进）
    servo1.set_angle(knob_to_servo_deg(rc.knob_left));
    servo2.set_angle(knob_to_servo_deg(rc.knob_right));

    // 遥控挡：摇杆 → 车体系平移（底盘约定 v_x 右正、v_y 前正）+ CH1 手动自旋
    const float fwd  = stick(rc.left_vertical);   // +1 前进
    const float left = stick(rc.left_horizontal); // +1 左移

    const float vx  = -left * REMOTE_V_MAX_CM_S;
    const float vy  = fwd * REMOTE_V_MAX_CM_S;
    const float ccw = stick(rc.right_vertical); // +1 左旋

    chassis.drive(vx, vy, ccw * REMOTE_W_MAX_RAD_S);

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(REMOTE_PERIOD_MS));
  }
}
