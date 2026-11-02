/**
 * @file board_instances.hpp
 * @brief 板级实例清单（gxs_v2）：库用 X-macro 按这里的名字展开
 *
 * @details 库没有运行时登记表：HAL 回调、`sys_task` 巡检、`can_rx` / `can_tx` 都靠这组宏
 *          在编译期知道「板子上有哪几个实例」。
 *
 *          【只写名字，类型由 decltype 推】改 `BspCanT` / `BspUartT` 的模板参数不用动本文件。
 *
 *          【加实例的完整步骤】
 *            1. board_cfg.hpp 里定义 `inline` 实例
 *            2. 本文件对应宏里加一项（续行用反斜杠，最后一项后面不加逗号）
 *            3. board_cfg.cpp 的 board_init() 里加 configASSERT
 *            4. CubeMX 里把对应中断打开
 *
 *          【忘了同步不会报错】：那个实例的回调不会被分发、sys_task 也不巡检它，
 *          表现是「数据收不到」而不是「编译失败」。
 */

#ifndef __BOARD_INSTANCES_HPP__
#define __BOARD_INSTANCES_HPP__

#include "board_cfg.hpp"

/**
 * @brief CAN 实例列表
 *
 * 本工程 `.ioc` 里没有启用 FDCAN，所以是空的（宏体写成注释，展开后什么都不产生）。
 * 打开 FDCAN 后改成：`#define BSP_CAN_INSTANCES(X) X(bsp_can1)`
 */
#define BSP_CAN_INSTANCES(X) /* 无 */

/** @brief UART 实例列表（sys_task 的串口巡检按此展开） */
#define BSP_UART_INSTANCES(X) \
  X(bsp_uart1)

/**
 * @brief SPI 实例列表
 *
 * 本工程 `.ioc` 里没有启用 SPI，所以是空的。
 */
#define BSP_SPI_INSTANCES(X) /* 无 */

#endif // __BOARD_INSTANCES_HPP__
