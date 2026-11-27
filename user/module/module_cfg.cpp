#include "module_cfg.hpp"

#include "board_cfg.hpp" // motor1/2/3


// 轮心到车体中心的距离 (cm)：当前按 10 cm 填，实测后改这里。
// 只影响自转：给 2 rad/s 时实际转多快，误差比例就等于 R 的误差比例；平移不受它影响。
constexpr float CHASSIS_R_CM = 10.0f;

// 底盘限幅：要盖住遥控侧的上限（平移 120 cm/s、自转 12 rad/s），
// 否则会在这里被静默截断，表现为「摇杆推满也不见得快」。
constexpr float CHASSIS_V_MAX_CM_S  = 130.0f;
constexpr float CHASSIS_W_MAX_RAD_S = 13.0f;


/* ==================== 全局实例 ==================== */

/**
 * @note 电机顺序按底盘定义 M1 右前 / M2 左前 / M3 正后；
 *       实际装配顺序如果不同，在这里换指针或改 omni3_chassis.cpp 的系数，别改 DeviceMotor::invert。
 */
Omni3Chassis chassis({.motor           = {&motor1, &motor2, &motor3},
                      .wheel_radius_cm = CHASSIS_R_CM,
                      .v_max_cm_s      = CHASSIS_V_MAX_CM_S,
                      .w_max_rad_s     = CHASSIS_W_MAX_RAD_S});

/** @brief 上位机协议执行器（逻辑全在 host.cpp） */
Host host;

/** @brief 场地页（逻辑全在 field_page.cpp） */
FieldPage field_page;


/* ==================== */


/* ==================== 函数定义 ==================== */


/** @brief Module 层统一初始化：失败就地停机 */
void module_init(void)
{
  chassis.init();
  field_page.init();
}

/* ==================== */
