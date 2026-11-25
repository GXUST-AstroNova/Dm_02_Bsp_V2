# WS2812 单灯珠（SPI6_MOSI / PA7）

板载 1 颗 WS2812B-3528：DIN 接 **SPI6_MOSI（PA7）**，VDD=5 V，DOUT→TP1。
SPI6 已由 CubeMX 配成 Master、TX-only、8 bit、MSB first、HSE 24 MHz / 4 = **6 MHz**。

## 文件

| 文件 | 内容 |
| --- | --- |
| `ws2812_codec.{h,cpp}` | 纯逻辑：按 SPI 时钟推导 WS2812 时序并编码位流（无 HAL，可主机测试） |
| `ws2812.{h,cpp}` | 单灯驱动：颜色/亮度、**非阻塞**中断发送、`Busy()`、限速 |
| `README.md` | 本文件 |

## 非阻塞发送原理

WS2812 用单线归零码，每个 bit 由高电平时间区分 0/1。驱动把每个 WS2812 bit 展开成 8 个
SPI bit（6 MHz 下）：'0' = `11000000`（T0H=333 ns），'1' = `11111000`（T1H=833 ns），
bit 周期 1333 ns，均在 WS2812B 规格内。

发送使用 **SPI6 中断**（`HAL_SPI_Transmit_IT`）：

- `Show()` 编码 24 字节后启动中断发送，**立即返回**（不阻塞主循环）；
- 发送约 **32 µs**，期间 `Busy()==true`，结束后 HAL 状态回到 READY；
- 驱动在 `ws2812.cpp` 中提供 `SPI6_IRQHandler`（工程原本没有该中断），优先级 **7**，
  低于底盘控制 TIM13（6）与 CAN（5）；
- 不使用 DMA/BDMA、不使用全局 HAL SPI 回调，因此与 SPI2（BMI088）互不影响；
- 帧尾 MOSI 保持低电平，`min_interval_ms`（默认 1 ms）保证 WS2812 锁存时间；
- SPI 若进入错误/中止状态，`Show()` 会自动调用 `Recover()`（无传输在途时 DeInit/Init
  软复位，不等待传输）并在下一次 `Show()` 重试；也可手动调用 `Recover()`。

## 用法

```cpp
#include "ws2812.h"

static Ws2812 led(&hspi6, Ws2812::Config{});   // GRB、亮度 255、6 MHz

void Board_Status_Init(void) {
    (void)led.Init();                           // 异步发送一帧全黑
}

void App_Loop(void) {
    led.Set_Color({255, 0, 0});                 // 只改目标颜色
    if (!led.Busy()) {
        (void)led.Show(HAL_GetTick());          // 非阻塞刷新；太近/忙时返回 false
    }
}
```

可配置项（`Ws2812::Config`）：

| 字段 | 默认 | 说明 |
| --- | --- | --- |
| `order` | `Grb` | 颜色顺序；WS2812B 为 GRB，若实际为 RGB 在此改 |
| `brightness` | 255 | 软件亮度 0..255，缩放公式 `(v*b+127)/255` |
| `min_interval_ms` | 1 | 两次 `Show` 最小间隔（锁存 + 限速）；0 按 1 ms 处理（WS2812 锁存需 >50 µs） |
| `spi_clock_hz` | 6000000 | 必须与 SPI6 实际内核时钟一致（HSE/4）；编码器会校验时序，不合法则 `Init()=-1` |

## 时序（6 MHz 自动推导）

| 参数 | 值 | WS2812B 规格 |
| --- | --- | --- |
| 每符号 SPI bit 数 | 8 | — |
| T0H / T0L | 333 ns / 1000 ns | 250–550 / 700–1000 ns |
| T1H / T1L | 833 ns / 500 ns | 650–950 / 300–600 ns |
| bit 周期 | 1333 ns | 650–1850 ns |
| 单帧 | 24 字节 / 约 32 µs | — |

## 注意

- **电平**：WS2812B 在 VDD=5 V 时 DIN 的 VIH 约 0.7×VDD=3.5 V，PA7 为 3.3 V 逻辑。
  若实测颜色偶发错乱，通常是这个原因，需要硬件电平转换（软件无法修正）。
- 若以后在 CubeMX 中启用 SPI6 全局中断，请删除 `ws2812.cpp` 末尾的 `SPI6_IRQHandler`，
  由生成的 `stm32h7xx_it.c` 接管（否则重复定义）。
- 若以后改 SPI6 分频/时钟，同步修改 `Config::spi_clock_hz`；`Init()` 会拒绝不合规时钟。
- 只支持单灯（本板硬件）；`ws2812_codec` 本身支持多像素编码，可用于以后扩展灯链。
