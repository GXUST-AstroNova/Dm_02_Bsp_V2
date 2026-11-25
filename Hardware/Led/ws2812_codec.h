//
// WS2812 位编码器（纯逻辑，无 HAL 依赖，可主机测试）
//
// 原理：用一个 SPI 字节串模拟 WS2812 的单线归零码。WS2812 每个 bit 用固定
// 高电平时间区分 0/1，因此把每个 WS2812 bit 展开成 N 个 SPI bit（前 h 位为 1、
// 其余为 0），SPI 时钟 6 MHz 时 N=8、h0=2、h1=5，时序为：
//   T0H=333 ns、T0L=1000 ns、T1H=833 ns、T1L=500 ns、Tbit=1333 ns
// 全部落在 WS2812B 数据手册容差内。
//
// 编码器在构造时按实际 SPI 时钟自动推导 N/h0/h1，并做 WS2812B 容差校验；
// 若 SPI 时钟被改成不满足时序的值（例如 3 MHz），Ok() 为 false，
// 驱动 Init() 会直接失败，避免“能显示但颜色/位置不可信”的隐性错误。
//

#ifndef DM_02_BSP_WS2812_CODEC_H
#define DM_02_BSP_WS2812_CODEC_H

#include <cstdint>

/** 一个 RGB 像素（0..255，不做颜色空间转换）。 */
struct LedColor {
    uint8_t r = 0U;
    uint8_t g = 0U;
    uint8_t b = 0U;
};

/** WS2812 数据线上三通道的发送顺序；WS2812B 默认 GRB。 */
enum class LedColorOrder : uint8_t {
    Grb = 0, ///< G, R, B（WS2812B 默认）
    Rgb,     ///< R, G, B
    Bgr,     ///< B, G, R
    Brg,     ///< B, R, G
    Gbr,     ///< G, B, R
    Rbg,     ///< R, B, G
};

class Ws2812Codec {
  public:
    /** 单个 WS2812 bit 最多允许展开的 SPI bit 数（限制缓冲区大小）。 */
    static constexpr uint8_t kMaxBitsPerSymbol = 16U;

    /** 编码时序参数（由 spi_clock_hz 推导并校验）。 */
    struct Timing {
        uint8_t bits_per_symbol = 0U; ///< 每个 WS2812 bit 展开的 SPI bit 数 N
        uint8_t t0h_bits = 0U;        ///< '0' 的高电平 SPI bit 数 h0
        uint8_t t1h_bits = 0U;        ///< '1' 的高电平 SPI bit 数 h1
        uint32_t bit_period_ns = 0U;  ///< 推导出的 WS2812 bit 周期（ns）
    };

    /**
     * @param spi_clock_hz SPI SCK 频率（工程为 HSE 24 MHz / 4 = 6 MHz）
     */
    explicit Ws2812Codec(uint32_t spi_clock_hz) noexcept;

    /** 时序是否合法（SPI 时钟可生成合规的 WS2812 波形）。 */
    bool Ok() const noexcept { return ok_; }

    const Timing &Get_Timing() const noexcept { return timing_; }

    /** 编码每个像素占用的字节数（= 3 * N；6 MHz 时为 24）。 */
    uint16_t Bytes_Per_Led() const noexcept { return bytes_per_led_; }

    /**
     * 把 pixels[0..count) 编码成 SPI 字节流（WS2812 高位先出，颜色顺序按 order）。
     * @param brightness 亮度 0..255，软件缩放公式 (v*brightness+127)/255
     * @param out        输出缓冲区
     * @param out_size   缓冲区容量（字节）
     * @return 写入字节数；0 = 参数非法/容量不足/编码器未就绪
     */
    uint16_t Encode(const LedColor *pixels, uint16_t count, LedColorOrder order,
                    uint8_t brightness, uint8_t *out, uint16_t out_size) const noexcept;

  private:
    static uint8_t Scale_(uint8_t value, uint8_t brightness) noexcept;

    bool ok_ = false;
    Timing timing_{};
    uint16_t bytes_per_led_ = 0U;
};

#endif // DM_02_BSP_WS2812_CODEC_H
