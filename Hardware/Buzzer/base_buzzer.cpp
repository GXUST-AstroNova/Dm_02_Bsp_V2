#include "base_buzzer.h"

Buzzer::Buzzer(uint16_t min_hz, uint16_t max_hz) noexcept
    : min_hz_(min_hz), max_hz_(max_hz) {}

int8_t Buzzer::Init() noexcept {
    if (ready_) {
        return 0;
    }
    if (Hardware_Init() != 0) {
        return -1;
    }
    ready_ = true;
    Apply_Tone(0U); // 上电静音
    return 0;
}

bool Buzzer::Play(const BuzzerNote *notes, uint16_t count, uint32_t now_ms,
                  bool loop) noexcept {
    if (!ready_ || notes == nullptr || count == 0U) {
        return false;
    }

    uint32_t total_ms = 0U;
    for (uint16_t i = 0U; i < count; ++i) {
        const uint16_t freq = notes[i].freq_hz;
        if (freq != buzzer_note::kRest && (freq < min_hz_ || freq > max_hz_)) {
            return false;
        }
        total_ms += notes[i].duration_ms;
    }
    if (total_ms == 0U) {
        return false; // 全是 0 时长：无法播放
    }

    notes_ = notes;
    count_ = count;
    index_ = 0U;
    note_start_ms_ = now_ms;
    loop_ = loop;
    playing_ = true;
    applied_ = false;
    return true;
}

void Buzzer::Stop(uint32_t now_ms) noexcept {
    (void)now_ms;
    if (!ready_) {
        return;
    }
    Apply_Tone(0U);
    playing_ = false;
    applied_ = false;
    index_ = 0U;
}

void Buzzer::Update(uint32_t now_ms) noexcept {
    if (!ready_ || !playing_ || notes_ == nullptr || count_ == 0U) {
        return;
    }

    // 首次 Update：应用当前音符（第一拍从 Play 的 now_ms 起算）
    if (!applied_) {
        Apply_Tone(notes_[index_].freq_hz);
        applied_ = true;
    }

    // 有界追赶：主循环卡顿跨多个音符时最多推进 count+1 次，避免死循环
    uint32_t guard = static_cast<uint32_t>(count_) + 1U;
    while (playing_) {
        if (guard == 0U) {
            // 极端卡顿：重新对齐到当前时刻，保持当前音符继续播放
            note_start_ms_ = now_ms;
            break;
        }
        --guard;

        const uint32_t duration = notes_[index_].duration_ms;
        const uint32_t elapsed = now_ms - note_start_ms_;
        if (duration != 0U && elapsed < duration) {
            break; // 当前音符仍在播放
        }

        note_start_ms_ += duration; // 0 时长不推进时间
        if (index_ + 1U < count_) {
            ++index_;
        } else if (loop_) {
            index_ = 0U;
        } else {
            Apply_Tone(0U); // 播完静音
            playing_ = false;
            applied_ = false;
            index_ = 0U;
            return;
        }
        Apply_Tone(notes_[index_].freq_hz); // 换音（0 时长会继续循环直到有时长音符）
    }
}
