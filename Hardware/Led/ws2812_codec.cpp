#include "ws2812_codec.h"

#include <cmath>

namespace {

/** WS2812B 数据手册时序容差（ns，取 datasheet 标称 ±150 ns 的保守区间）。 */
constexpr uint32_t kT0hMin = 250U, kT0hMax = 550U;
constexpr uint32_t kT0lMin = 700U, kT0lMax = 1000U;
constexpr uint32_t kT1hMin = 650U, kT1hMax = 950U;
constexpr uint32_t kT1lMin = 300U, kT1lMax = 600U;
constexpr uint32_t kTbitMin = 650U, kTbitMax = 1850U;

bool In_Range(uint32_t value, uint32_t lo, uint32_t hi) noexcept {
    return value >= lo && value <= hi;
}

} // namespace

Ws2812Codec::Ws2812Codec(uint32_t spi_clock_hz) noexcept {
    if (spi_clock_hz == 0U) {
        return;
    }

    const double t_ns = 1.0e9 / static_cast<double>(spi_clock_hz);
    const auto bits_for = [t_ns](double target_ns) {
        return static_cast<int>(std::lround(target_ns / t_ns));
    };

    const int n = bits_for(1250.0); // 目标 WS2812 bit 周期 ≈ 1.25 us
    if (n < 2 || n > static_cast<int>(kMaxBitsPerSymbol)) {
        return;
    }

    int h0 = bits_for(400.0); // T0H ≈ 0.4 us
    int h1 = bits_for(800.0); // T1H ≈ 0.8 us
    if (h0 < 1) {
        h0 = 1;
    }
    if (h1 <= h0) {
        h1 = h0 + 1;
    }
    if (h1 >= n) {
        return; // 没有低电平尾部，无法保证锁存
    }

    const uint32_t t0h = static_cast<uint32_t>(std::lround(h0 * t_ns));
    const uint32_t t0l = static_cast<uint32_t>(std::lround((n - h0) * t_ns));
    const uint32_t t1h = static_cast<uint32_t>(std::lround(h1 * t_ns));
    const uint32_t t1l = static_cast<uint32_t>(std::lround((n - h1) * t_ns));
    const uint32_t tbit = static_cast<uint32_t>(std::lround(n * t_ns));

    if (!In_Range(t0h, kT0hMin, kT0hMax) || !In_Range(t0l, kT0lMin, kT0lMax) ||
        !In_Range(t1h, kT1hMin, kT1hMax) || !In_Range(t1l, kT1lMin, kT1lMax) ||
        !In_Range(tbit, kTbitMin, kTbitMax)) {
        return;
    }

    timing_.bits_per_symbol = static_cast<uint8_t>(n);
    timing_.t0h_bits = static_cast<uint8_t>(h0);
    timing_.t1h_bits = static_cast<uint8_t>(h1);
    timing_.bit_period_ns = tbit;
    bytes_per_led_ = static_cast<uint16_t>(3U * static_cast<uint16_t>(n)); // 24 bit * n / 8
    ok_ = true;
}

uint8_t Ws2812Codec::Scale_(uint8_t value, uint8_t brightness) noexcept {
    return static_cast<uint8_t>((static_cast<uint16_t>(value) * brightness + 127U) / 255U);
}

uint16_t Ws2812Codec::Encode(const LedColor *pixels, uint16_t count, LedColorOrder order,
                             uint8_t brightness, uint8_t *out, uint16_t out_size) const noexcept {
    if (!ok_ || pixels == nullptr || out == nullptr || count == 0U) {
        return 0U;
    }

    const uint32_t total = static_cast<uint32_t>(count) * bytes_per_led_;
    if (total == 0U || total > out_size) {
        return 0U;
    }

    uint8_t channel[3] = {0U, 1U, 2U};
    switch (order) {
        case LedColorOrder::Grb: channel[0] = 1U; channel[1] = 0U; channel[2] = 2U; break;
        case LedColorOrder::Rgb: channel[0] = 0U; channel[1] = 1U; channel[2] = 2U; break;
        case LedColorOrder::Bgr: channel[0] = 2U; channel[1] = 1U; channel[2] = 0U; break;
        case LedColorOrder::Brg: channel[0] = 2U; channel[1] = 0U; channel[2] = 1U; break;
        case LedColorOrder::Gbr: channel[0] = 1U; channel[1] = 2U; channel[2] = 0U; break;
        case LedColorOrder::Rbg: channel[0] = 0U; channel[1] = 2U; channel[2] = 1U; break;
        default: return 0U;
    }

    uint32_t pos = 0U;
    uint8_t byte = 0U;
    uint8_t filled = 0U;
    const uint8_t n = timing_.bits_per_symbol;

    for (uint16_t i = 0U; i < count; ++i) {
        const LedColor &c = pixels[i];
        const uint8_t rgb[3] = {Scale_(c.r, brightness), Scale_(c.g, brightness),
                                Scale_(c.b, brightness)};
        for (uint8_t k = 0U; k < 3U; ++k) {
            const uint8_t value = rgb[channel[k]];
            for (int bit = 7; bit >= 0; --bit) {
                const bool one = ((value >> static_cast<uint8_t>(bit)) & 0x01U) != 0U;
                const uint8_t ones = one ? timing_.t1h_bits : timing_.t0h_bits;
                for (uint8_t b = 0U; b < n; ++b) {
                    byte = static_cast<uint8_t>((static_cast<uint16_t>(byte) << 1U) |
                                                ((b < ones) ? 1U : 0U));
                    if (++filled == 8U) {
                        out[pos++] = byte;
                        byte = 0U;
                        filled = 0U;
                    }
                }
            }
        }
    }

    // 24 bit * N 一定是 8 的整数倍（24 * N / 8 = 3N），出现半字节说明逻辑错误
    if (filled != 0U || pos != total) {
        return 0U;
    }
    return static_cast<uint16_t>(pos);
}
