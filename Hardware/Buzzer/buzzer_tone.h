//
// 蜂鸣器音调换算（纯逻辑，无 HAL 依赖，可主机测试）
//
// 定时器计数频率 tick_hz（= 定时器时钟 / (PSC+1)，工程默认 1 MHz）下，
// 目标频率 freq_hz 的自动重装值：ARR = round(tick_hz / freq_hz) - 1。
// 50% 占空比：CCR = (ARR + 1) / 2（NPN 低边、高电平驱动，无源蜂鸣器需要方波）。
//

#ifndef DM_02_BSP_BUZZER_TONE_H
#define DM_02_BSP_BUZZER_TONE_H

#include <cstdint>

namespace buzzer_tone {

/** ARR 下限：过小的 ARR 无法产生可听频率，统一限幅到 2。 */
inline constexpr uint32_t kMinArr = 2U;
/** 16 位定时器 ARR 上限。 */
inline constexpr uint32_t kMaxArr = 65535U;

/**
 * 频率 → ARR。
 * @param freq_hz 目标频率（Hz）；0 = 休止/静音
 * @param tick_hz 定时器计数频率（Hz）
 * @return ARR；0 = 休止（调用方应把 CCR 设为 0）；频率过高/过低时被限幅
 */
inline uint32_t Arr_For(uint16_t freq_hz, uint32_t tick_hz) noexcept {
    if (freq_hz == 0U || tick_hz == 0U) {
        return 0U;
    }
    uint32_t periods = (tick_hz + freq_hz / 2U) / freq_hz; // round(tick/f)
    if (periods == 0U) {
        return kMinArr; // 频率高于定时器能力：限幅到最高可输出
    }
    uint32_t arr = periods - 1U;
    if (arr < kMinArr) {
        arr = kMinArr;
    }
    if (arr > kMaxArr) {
        arr = kMaxArr;
    }
    return arr;
}

/** 50% 占空比对应的 CCR。 */
inline uint32_t Compare_For(uint32_t arr) noexcept {
    return (arr + 1U) / 2U;
}

} // namespace buzzer_tone

#endif // DM_02_BSP_BUZZER_TONE_H
