/**
 * @file module_cfg.hpp
 * @author Rh
 * @brief Module 层统一管理 —— module_init() 与全局实例声明
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 * @note 全局实例在此声明、在 module_cfg.cpp 中实例化；本层的初始化动作收进 module_init()，
 *       在 board_init() 之后调用（依赖三个已绑定的 DeviceMotor）。
 */

#ifndef __MODULE_CFG_HPP__
#define __MODULE_CFG_HPP__

#include "host.hpp"
#include "omni3_chassis.hpp"


/* ==================== 初始化 ==================== */

/**
 * @brief Module 层统一初始化
 *
 * @note 内部用 configASSERT 校验配置，失败就地停机，因此不需要返回值。
 *       须在 board_init() 之后调用（依赖 motor1/2/3 已 init()）。
 */
void module_init(void);

/* ==================== */


/* ==================== 全局声明 ==================== */

extern Omni3Chassis chassis; ///< 三全向轮底盘（M1 右前 / M2 左前 / M3 正后）
extern Host         host;    ///< 上位机协议执行器（定义见 module_cfg.cpp）

/**
 * @brief 场地页的按键共享状态（key_task 写，oled_task 读）
 * @note 长按确认后屏上显示 1 s 的 `已发送`。
 */
namespace field_ui
{
extern volatile uint32_t sent_ms; ///< 0 = 还没发过；否则 = 上次确认时刻 (ms)
}

/* ==================== */
#endif // __MODULE_CFG_HPP__
