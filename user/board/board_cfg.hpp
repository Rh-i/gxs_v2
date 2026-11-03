/**
 * @file board_cfg.hpp
 * @brief 板级实例配置（gxs_v2）
 *
 * @details 本工程目前是**接入骨架**：只开了一路调试串口 + DWT，用来验证
 *          「CubeMX 工程 → submodule → integrate.py → 编译出 elf」这条链路。
 *
 *          【要加外设时改这三步】
 *            1. 这里加实例（`inline` 变量），句柄来自 CubeMX 生成的 `usart.h` / `tim.h` / `fdcan.h`
 *            2. `board_instances.hpp` 加实例名（中断分发与 sys_task 巡检按清单展开）
 *            3. `board_cfg.cpp` 的 `board_init()` 里加 `configASSERT(xxx.init() == Status::OK)`
 *
 *          【本工程的外设情况】（`.ioc` 实际启用）
 *            USART1/2/3、UART4/5/7、TIM1/2/3/4/8/12、I2C1
 *            **没有 FDCAN、没有 SPI**，串口也**没有配 DMA**，所以只能用 `uart_mode::it`。
 *
 *          【硬规矩】
 *            - 串口 / SPI 用 DMA 时实例必须放 `.dma_buffer`（DMA 访问不到 ITCM/DTCM）
 *            - 改了 `BspUartT` 的 Transport 或缓冲大小，要在 `code/bsp/bsp_uart.cpp` 补显式实例化
 *              （库里现有：`<dma,128,128>`、`<it,128,128>`、`<it,128,32>`，别用其它组合）
 *            - 这里每加一个实例，`board_instances.hpp` 必须同步，否则中断不会分发
 */

#ifndef __BOARD_CFG_HPP__
#define __BOARD_CFG_HPP__

// CubeMX 生成的句柄头文件（按实际外设增删）
#include "main.h"
#include "usart.h"

#include "bsp_dwt.hpp"
#include "bsp_uart.hpp"


/** @brief 板级初始化：各实例 init()，实现在 board_cfg.cpp */
void board_init(void);


/* ==================== 外设实例 ==================== */

/**
 * USART1：中断收发，115200
 *   - 本工程没给串口配 DMA，所以用 `uart_mode::it`；配了 DMA 再换成 `uart_mode::dma`
 *   - 两个模板参数是 RX / TX 环形缓冲字节数
 *   - 虽然暂未用 DMA，仍放在 `.dma_buffer`：段已在链接脚本里，位置固定、上电早期拷贝，
 *     以后换 DMA 不用再改链接脚本
 */
inline __attribute__((section(".dma_buffer"))) BspUartT<uart_mode::it, 128, 128> bsp_uart1(
    {.huart = &huart1, .mode = UartDir::TxRx, .baudrate = 115200U});

/**
 * 调试打印：包含本头文件的文件都能用
 *
 *     print_uart("tick %u\r\n", count);
 *
 * 打印浮点需要宿主 CMake 里的 `-u _printf_float`（integrate.py 已加）。
 */
#define print_uart(...) bsp_uart1.printf(__VA_ARGS__)

/**
 * DWT 计时：由库内部初始化（实例与 `init()` 都在 `system.cpp` 的 `system_init()` 里）。
 * 头文件包含后直接调用 bsp_dwt.get_delta_s() / get_time_s() 即可。
 */

#endif // __BOARD_CFG_HPP__
