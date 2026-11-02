/**
 * @file board_cfg.cpp
 * @brief board_init()：把 board_cfg.hpp 里的实例逐个 init()（gxs_v2）
 *
 * @details 【本文件只放一件事】`board_init()`。实例本身都在 board_cfg.hpp 里（`inline`）。
 *
 *          【为什么每行都 configASSERT】
 *          `init()` 返回非 OK 说明句柄没初始化、DMA 没配或参数不对。就地停机后
 *          `fault_record` 会记下代码行号，`print_uart` 里能看出是哪一路；
 *          继续跑下去只会在半初始化的系统上产生更难查的现象。
 *
 *          【顺序】DWT 第一个（sys_task 一开始就用它算耗时）；其余外设之间没有依赖，
 *          挂在 CAN 上的设备要放在 CAN init 之后、system_start() 之前。
 *
 *          【不要在这里做】不调 HAL_Init() / SystemClock_Config()（CubeMX 的 main.c 负责）、
 *          不建任务（system_init() / system_start() 负责）、不写业务逻辑（放 user/app）。
 */

#include "board_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)


void board_init(void)
{
  // 计时基准：sys_task 用它算 10 ms 周期内的耗时，必须最先起来
  configASSERT(bsp_dwt.init() == Status::OK);

  // 调试串口
  configASSERT(bsp_uart1.init() == Status::OK);

  // 之后加的外设、设备（CAN 电机等）在这里继续 init()
}
