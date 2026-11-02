# user/module —— 业务模块

这里放**多个设备/算法组合出来的业务逻辑**，例如：

- 底盘运动学（麦克纳姆 / 全向轮 / 舵轮）
- 云台控制
- 自定义通信协议（`corexy_protocol` 之类，只有本工程用）

不属于这里的：

| 想放的东西 | 应该放 |
| --- | --- |
| 单个设备驱动（电机、舵机、IMU、遥控） | 框架 `code/device/` |
| 外设初始化 / 收发 | 框架 `code/bsp/` |
| 任务创建、系统状态 | 框架 `code/service/` |
| 业务任务本身 | `user/app/task/` |

## 加模块的步骤

1. 在本目录建 `xxx.hpp` / `xxx.cpp`
2. 头文件里只暴露「构造 + `init()` + 每周期调用」三类接口，不要暴露内部状态
3. 实例定义在 `user/app/app_main.cpp`（或专门的 `module_cfg`），并在 `app_init()` 里 `init()`
4. **不需要改 CMake**：根 `CMakeLists.txt` 已经 `GLOB_RECURSE user/**`，新增文件下次构建自动带上
   （`CONFIGURE_DEPENDS` 保证增删文件后不用手动重新配置）

## 当前状态

GXS_V2 是接入骨架，还没有业务模块。加第一个模块时，建议先照 `user/app/task/demo_task.cpp`
的写法把任务骨架搭起来，再往里填逻辑。
