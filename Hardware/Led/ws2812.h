//
// 单灯珠 WS2812 非阻塞驱动（SPI6_MOSI / PA7）
//
// 板载：1 颗 WS2812B-3528，DIN 接 SPI6_MOSI(PA7)，VDD=5 V，DOUT→TP1（测试点）。
//
// 非阻塞发送：
//   - 使用 SPI6 中断发送（HAL_SPI_Transmit_IT），Show() 启动后立即返回；
//   - SPI6_IRQn 由本驱动 Init() 使能，SPI6_IRQHandler 定义在本文件（工程原本没有，
//     stm32h7xx_it.c 若以后由 CubeMX 生成同一中断处理，请删除本文件中的定义）；
//   - 完成状态用 HAL_SPI_GetState() 观察（Busy()），不使用全局 HAL 回调，
//     不会与 SPI2（BMI088）等其它 SPI 设备互相影响；
//   - 无 DMA、无动态内存、无忙等。
//
// 时序：SPI6 = HSE 24 MHz / 4 = 6 MHz，编码器自动展开为每 WS2812 bit 8 个 SPI bit
//   （T0H=333 ns、T1H=833 ns、Tbit=1333 ns，见 ws2812_codec.h）。一帧单灯 24 字节，
//   发送约 32 µs。帧尾 MOSI 保持低电平，两次 Show 间隔由 min_interval_ms 保证锁存。
//
// 用法：
//   static Ws2812 led(&hspi6, Ws2812::Config{});
//   led.Init();                                  // 异步发送一帧全黑
//   led.Set_Color({255, 0, 0});
//   if (!led.Busy()) { (void)led.Show(HAL_GetTick()); }   // 主循环调用，立即返回
//

#ifndef DM_02_BSP_WS2812_H
#define DM_02_BSP_WS2812_H

#include <cstdint>

#include "spi.h"
#include "ws2812_codec.h"

class Ws2812 {
  public:
    struct Config {
        LedColorOrder order = LedColorOrder::Grb; ///< 颜色顺序（WS2812B 默认 GRB）
        uint8_t brightness = 255U;                ///< 软件亮度 0..255
        uint32_t min_interval_ms = 1U;            ///< 两次 Show 的最小间隔（锁存/限速）
        uint32_t spi_clock_hz = 6000000U;         ///< 必须与 SPI6 实际内核时钟一致（24/4）
    };

    Ws2812(SPI_HandleTypeDef *spi, const Config &config) noexcept;

    /**
     * 校验 SPI 状态/参数/时序，使能 SPI6 中断，并异步发送一帧全黑（清掉上电随机色）。
     * @return 0 = 成功；-1 = 失败（参数/SPI 状态/时序非法）
     * @note 非阻塞：只启动中断发送后立即返回。
     */
    int8_t Init() noexcept;

    /** 只更新目标颜色（影子），不发送。 */
    bool Set_Color(LedColor color) noexcept;

    LedColor Get_Color() const noexcept { return color_; }

    /**
     * 非阻塞刷新：把当前颜色编码后启动 SPI6 中断发送，立即返回。
     * @return true = 已启动发送；false = 未初始化 / 上一帧还在发送 / 距上次过近 /
     *         SPI 状态异常 / HAL 启动失败（详见 Get_Error_Count）
     */
    bool Show(uint32_t now_ms) noexcept;

    /** 上一帧是否仍在发送（正常单灯约 32 µs 后变 false）。 */
    bool Busy() const noexcept;

    /**
     * SPI 进入错误/中止状态后的恢复：无传输在途时执行 DeInit/Init 软复位，
     * 不等待传输完成（不阻塞）。Show 检测到错误状态时也会自动尝试一次。
     * @return true = 已恢复，可继续 Show；false = 当前忙/恢复失败
     */
    bool Recover() noexcept;

    bool Is_Ready() const noexcept { return ready_; }

    uint32_t Get_Show_Count() const noexcept { return show_count_; }
    uint32_t Get_Error_Count() const noexcept { return error_count_; }

  private:
    bool Start_Transfer_() noexcept;

    SPI_HandleTypeDef *spi_ = nullptr;
    Config config_{};
    Ws2812Codec codec_;
    LedColor color_{};
    uint8_t tx_buf_[3U * Ws2812Codec::kMaxBitsPerSymbol]{}; ///< 6 MHz 时实际用 24 B
    uint32_t last_start_ms_ = 0U;
    uint32_t show_count_ = 0U;
    uint32_t error_count_ = 0U;
    bool ready_ = false;
    bool has_shown_ = false;
};

#endif // DM_02_BSP_WS2812_H
