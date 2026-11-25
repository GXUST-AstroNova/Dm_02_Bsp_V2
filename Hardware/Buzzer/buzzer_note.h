//
// 蜂鸣器乐谱数据结构与常用音符频率（十二平均律，A4=440 Hz）
//
// 一段音乐 = BuzzerNote 数组；freq_hz=0 表示休止符，duration_ms 为音长。
// 播放时把数组或数组的任意子区间（指针 + 个数）交给 Buzzer::Play() 即可。
//
// 例：
//   static constexpr BuzzerNote kTwinkle[] = {
//       {buzzer_note::C5, 400}, {buzzer_note::C5, 400},
//       {buzzer_note::G5, 400}, {buzzer_note::G5, 400},
//       {buzzer_note::A5, 400}, {buzzer_note::A5, 400},
//       {buzzer_note::G5, 600}, {buzzer_note::kRest, 120},
//   };
//

#ifndef DM_02_BSP_BUZZER_NOTE_H
#define DM_02_BSP_BUZZER_NOTE_H

#include <cstdint>

/** 一个音符：频率（Hz，0=休止）与持续时间（ms）。 */
struct BuzzerNote {
    uint16_t freq_hz = 0U;
    uint16_t duration_ms = 0U;
};

namespace buzzer_note {

/** 休止符。 */
inline constexpr uint16_t kRest = 0U;

// 十二平均律（四舍五入到整数 Hz），C4..B6 共 3 个八度。
inline constexpr uint16_t C4 = 262U;
inline constexpr uint16_t D4 = 294U;
inline constexpr uint16_t E4 = 330U;
inline constexpr uint16_t F4 = 349U;
inline constexpr uint16_t G4 = 392U;
inline constexpr uint16_t A4 = 440U;
inline constexpr uint16_t B4 = 494U;

inline constexpr uint16_t C5 = 523U;
inline constexpr uint16_t D5 = 587U;
inline constexpr uint16_t E5 = 659U;
inline constexpr uint16_t F5 = 698U;
inline constexpr uint16_t G5 = 784U;
inline constexpr uint16_t A5 = 880U;
inline constexpr uint16_t B5 = 988U;

inline constexpr uint16_t C6 = 1047U;
inline constexpr uint16_t D6 = 1175U;
inline constexpr uint16_t E6 = 1319U;
inline constexpr uint16_t F6 = 1397U;
inline constexpr uint16_t G6 = 1568U;
inline constexpr uint16_t A6 = 1760U;
inline constexpr uint16_t B6 = 1976U;

} // namespace buzzer_note

#endif // DM_02_BSP_BUZZER_NOTE_H
