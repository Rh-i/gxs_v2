/**
 * @file app_task.hpp
 * @brief 应用层任务声明 + 任务间共享数据
 *
 * @details key / motor / remote / imu 由 app_init() 统一创建。
 */

#ifndef __APP_TASK_HPP__
#define __APP_TASK_HPP__

#include <stdint.h> // 运行时开关用 uint8_t


/* ==================== 任务入口 ==================== */

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief 按键任务：50 ms 轮询 key1 / key2，把短按/长按事件打到调试串口
   * @param argument 任务参数（未使用，NULL）
   * @note 轮询周期与 board_cfg.hpp 里 debounce_cnt / long_press_cnt 配套（100 ms 消抖 / 500 ms 长按）。
   *       本任务独占 key1 / key2 的 update()。
   */
  void key_task(void *argument);

  /**
   * @brief 上下位机通讯任务：USART1（921600）收指令 + 周期上报（独占上位机串口）
   * @param argument 任务参数（未使用，NULL）
   * @note 只在遥控器 CH5 拨到上位机挡（`ControlSource::HOST`）时才把速度下给底盘。
   *       协议说明见 `user/module/上位机协议.md`。
   */
  void host_task(void *argument);

  /**
   * @brief 舵机任务：50 Hz 推进 servo1 / servo2 的 update()
   * @param argument 任务参数（未使用，NULL）
   * @note `DeviceServo::update()` 是唯一 PWM 写入者；默认中位由 board_init() 里的 set_angle() 给出。
   *       本任务独占两个舵机的 update()。
   */
  void servo_task(void *argument);

  /**
   * @brief 电机速度环任务：默认 1 ms（1 kHz）周期，跑 motor1/2/3 的 update()
   * @param argument 任务参数（未使用，NULL）
   * @note 控制周期改 `motor_task.cpp` 里的 MOTOR_TASK_PERIOD_MS；dt 由 bsp_dwt 实测，
   *       改频率后需要重新标定 PID（ki / kd 是每拍量纲）。
   */
  void motor_task(void *argument);

  /**
   * @brief 遥控任务：R9DS 通道 → 底盘速度（100 Hz）
   * @param argument 任务参数（未使用，NULL）
   * @note CH3 前后、CH4 平移、CH1 自旋；CH5 三挡：+100 自动（接口暂留）/ 0 停车 / -100 遥控。
   *       失联或非遥控挡一律制动。本任务独占 r9ds.update()。
   */
  void remote_task(void *argument);

  /**
   * @brief OLED 场地页任务：显示出生点与安全区颜色，按键后刷新，空闲不刷屏
   * @param argument 任务参数（未使用，NULL）
   * @note 一整帧 I²C 刷新是阻塞的（约 24~26 ms @400 kHz），所以本任务用最低优先级。
   *       OLED 实例定义在 `board_cfg.cpp`；没接屏时每周期重试 init，不影响其它任务。
   */
  void oled_task(void *argument);

  /**
   * @brief 电机监控任务：1 kHz 把一台（可切换）电机的目标 / 反馈 / PID 输出按 JustFloat 发给 VOFA+
   * @param argument 任务参数（未使用，NULL）
   * @note 观察对象是全局变量 `motor_monitor_index`（0/1/2 → motor1/2/3，≥ 3 = 停发），
   *       调试器或 EmberProbe 写它即可切换，无需重新烧录。
   *       测试波形同受 `motor_test_wave` 控制（0 = 关，1 = 方波，2 = 正弦，±50 cm/s、2 s，
   *       4 = 开环固定 ±50% 占空比）；**非 0 时遥控/上位机被屏蔽，调完改回 0**。
   *       波形走 UART8（921600，与调试打印同口）。
   */
  void motor_monitor_task(void *argument);

#ifdef __cplusplus
}
#endif

/* ==================== */
/* 运行时开关（调试器 / EmberProbe 直接写，无需重新烧录） */

/** @brief VOFA 观察（也是测试波形驱动）的电机：0/1/2 → motor1/2/3，≥ 3 = 停发 */
extern volatile uint8_t motor_monitor_index;

/** @brief 电机测试波形：0 = 关（底盘听遥控/上位机），1 = 方波 ±50 cm/s，2 = 正弦 50 cm/s / 2 s */
extern volatile uint8_t motor_test_wave;

/* ==================== */

#endif // __APP_TASK_HPP__
