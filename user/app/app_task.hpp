/**
 * @file app_task.hpp
 * @brief 应用层任务声明 + 任务间共享数据
 *
 * @details key / motor / remote / imu 由 app_init() 统一创建。
 */

#ifndef __APP_TASK_HPP__
#define __APP_TASK_HPP__

#include <stdint.h>


/* ==================== 任务间共享数据 ==================== */

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief WT102 最新一帧的 yaw 原始值（int16）
   *
   * @note 由 imu_task 每 2 ms 刷新；别的任务只读。单独用 int16 传是为了避免
   *       “一个任务写 wt102、另一个任务读 wt102”这种并发访问（库要求读取期间不得并发 update()）。
   *       M7 上对 volatile int16_t 的单次读写是原子的，不需要临界区。
   */
  extern volatile int16_t g_imu_yaw_raw;

#ifdef __cplusplus
}
#endif

/* ==================== */


/* ==================== 任务入口 ==================== */

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief 按键任务：200 ms 轮询 key1 / key2，把短按/长按事件打到调试串口
   * @param argument 任务参数（未使用，NULL）
   * @note 轮询周期与 board_cfg.hpp 里 debounce_cnt / long_press_cnt 配套（200 ms 消抖 / 1 s 长按）。
   *       本任务独占 key1 / key2 的 update()。
   */
  void key_task(void *argument);

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
   * @brief 陀螺仪任务：500 Hz 读 WT102（USART2），刷新 Z 轴三个量的最近两帧
   * @param argument 任务参数（未使用，NULL）
   * @note 数据直接从 `wt102.get_z_newest()` / `get_z_previous()` 取，
   *       单位换算（°/s、°、g）由用的人做。本任务独占 `wt102.update()`。
   */
  void imu_task(void *argument);

#ifdef __cplusplus
}
#endif

/* ==================== */

#endif // __APP_TASK_HPP__
