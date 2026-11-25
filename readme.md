# Dm_02_Bsp_V2

基于 STM32H723 的嵌入式 BSP 工程，使用 STM32CubeMX 生成外设初始化代码，并以 C/C++ 实现电机、传感器和常用控制组件。工程通过 CMake 和 Ninja 构建。

## 仓库定位与提交范围

本仓库是项目的**代码基线**，用于维护可复用的基础能力。仅应向本仓库提交公共支持库、电机支持库和传感器支持库，以及这些公共组件所必需的配置与维护内容。具体机器人、赛季或任务专用的业务逻辑和应用代码应在各自的项目中维护；现有应用代码用于示例与调试。
## 功能概览

- 电机：达妙电机及大疆 M3508 驱动，基于 FDCAN 收发。
- 传感器与输入：BMI088 IMU、FlySky SBUS 接收器。
- 其他硬件：W25Q Flash、WS2812 LED、PWM 蜂鸣器，以及 UART、FDCAN 基础接口。
- 控制与调试：PID、滑模控制组件、VOFA 数据发送，以及 Fusion 姿态融合库。

具体接口和总线配置见 [硬件接口说明](Hardware/README.md)，分层设计见 [设计思路](设计思路.md)。各模块的实际启用情况请以应用代码和 CubeMX 配置为准。

## 板上外设与配置状态

以下为当前 CubeMX 配置和仓库代码所体现的状态，不代表所有外设均已在当前应用中启用或完成实机验证：

| 外设或接口 | 当前配置与支持情况 |
| --- | --- |
| 主控 | STM32H723VGT6，Cortex-M7；CubeMX 配置系统时钟为 550 MHz。 |
| FDCAN1 / FDCAN3 | 配置为经典 CAN，1 Mbps；应用示例启动两条总线。 |
| FDCAN2 | 配置为 CAN FD + BRS，仲裁段 1 Mbps、数据段 5 Mbps；应用示例使用该总线控制达妙电机。 |
| BMI088 | 提供驱动；SPI2 配置为主机并使用收发 DMA，配置了加速度计和陀螺仪片选及中断引脚。 |
| W25Q Flash | 提供驱动；OCTOSPI2 配置为四线模式。 |
| 遥控器 | 提供 FlySky SBUS 接收支持；UART5 配置为 100000 baud、偶校验、2 位停止位，并配置接收 DMA。 |
| 串口调试 | UART9 配置为 921600 baud，收发均配置 DMA；当前示例用于 VOFA。UART7、USART1、USART3、USART10 也已配置。 |
| LED 与蜂鸣器 | 提供 WS2812 和 PWM 蜂鸣器组件；SPI6 配置为仅发送主机，TIM3 CH4 与 TIM12 CH2 配置为 PWM 输出。 |
| USB | CubeMX 配置了 USB Device CDC 和全速设备接口。 |

具体引脚、参数及启动方式以 [CubeMX 配置](Dm_02_Bsp_V2.ioc)、生成代码和各模块文档为准；电机总线细节见 [硬件接口说明](Hardware/README.md)。
## 环境准备

- Git（包含子模块支持）
- CMake 3.22 或更新版本
- Ninja
- Arm GNU Toolchain（`arm-none-eabi-gcc`、`arm-none-eabi-g++` 等命令需在 PATH 中）
- 修改外设配置时需要 STM32CubeMX；下载固件还需与硬件匹配的调试器和烧录工具

工程使用 C11 和 C++20；请使用支持 C++20 的 Arm 工具链。

## 获取与构建

克隆时初始化 Fusion 子模块：

```bash
git clone --recurse-submodules <仓库地址>
cd Dm_02_Bsp_V2
```

已有工作区可执行：

```bash
git submodule update --init --recursive
```

在项目根目录使用预设构建：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

发布构建可将上述两处 `Debug` 替换为 `Release`。生成的 ELF 和 map 文件位于对应的 `build/Debug/` 或 `build/Release/` 目录，可使用调试器加载 ELF 或按烧录工具的要求转换格式。构建预设及工具链配置分别位于 `CMakePresets.json` 和 `cmake/gcc-arm-none-eabi.cmake`。

## 工程结构

```text
Application/     应用初始化、主循环与定时回调
Hardware/        电机、传感器和其他硬件接口
Service/         服务组件（如 VOFA）
Tools/           控制算法及第三方 Fusion 库
Core/            CubeMX 生成的核心代码
Drivers/         STM32 HAL 等驱动
Middlewares/     中间件
USB_DEVICE/      USB 设备代码
cmake/           工具链与 CubeMX 构建配置
Dm_02_Bsp_V2.ioc CubeMX 工程配置
```

## 使用与调试

应用入口位于 [Application.cpp](Application/Application.cpp)。当前示例初始化 FDCAN1、FDCAN2、FDCAN3，启用一台达妙电机，并初始化一台 M3508；TIM7 回调中以 1 kHz 更新控制目标、发送电机命令和 VOFA 数据。实际接线、电机 ID、总线参数和控制逻辑应按设备调整。

**上电前请检查应用中的电机使能和控制代码、接线及供电。** 电机接口、帧格式、初始化顺序和实机验收要点见 [硬件接口说明](Hardware/README.md)。

外设配置可通过根目录的 `.ioc` 文件在 STM32CubeMX 中查看或修改。重新生成代码后，请检查生成结果和用户代码区，并重新构建验证。

## 许可证

本项目许可证见 [LICENSE](LICENSE)。第三方组件遵循各自的许可证，例如 [Fusion 许可证](Tools/ThirdParty/Fusion/LICENSE.md)。
