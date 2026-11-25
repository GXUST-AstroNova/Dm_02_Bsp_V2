//
// 遥控接收机基类实现
//

#include "base_rc_receiver.h"

#include "main.h"   // HAL_GetTick()

namespace {
/** 死区：|归一化值| 小于它就算 0（口径沿用 RM25 parse_DR16_receiver_msg 的 invalid_range） */
constexpr float kDeadband = 0.01f;

/**
 * 求某个原始值所在半边的量程跨度。
 * 中位两侧的跨度不一定相等（SBUS 是 992-172=820 与 1811-992=819），
 * 按所在半边各算各的，两端才能**精确**落到 -1.0/+1.0 与 1000us/2000us。
 */
inline float sideSpan(uint16_t v, uint16_t lo, uint16_t mid, uint16_t hi) noexcept {
    if (v >= mid) {
        return (hi > mid) ? static_cast<float>(hi - mid) : 0.0f;
    }
    return (mid > lo) ? static_cast<float>(mid - lo) : 0.0f;
}
}  // namespace

uint16_t RcReceiver::Get_Raw(uint8_t ch) const noexcept {
    return (ch < count_) ? channels_[ch] : raw_mid_;
}

float RcReceiver::Get_Channel(uint8_t ch) const noexcept {
    if (ch >= count_) {
        return 0.0f;
    }
    const float span = sideSpan(channels_[ch], raw_lo_, raw_mid_, raw_hi_);
    if (span <= 0.0f) {
        return 0.0f;
    }

    float v = (static_cast<float>(channels_[ch]) - static_cast<float>(raw_mid_)) / span;
    if (v > 1.0f)  { v = 1.0f; }
    if (v < -1.0f) { v = -1.0f; }
    if ((v <= kDeadband) && (v >= -kDeadband)) { v = 0.0f; }
    return v;
}

float RcReceiver::Get_Microseconds(uint8_t ch) const noexcept {
    if (ch >= count_) {
        return 1500.0f;
    }
    const float span = sideSpan(channels_[ch], raw_lo_, raw_mid_, raw_hi_);
    if (span <= 0.0f) {
        return 1500.0f;
    }
    // 中位 1500us，±满量程对应 ±500us
    return 1500.0f + (static_cast<float>(channels_[ch]) - static_cast<float>(raw_mid_)) * (500.0f / span);
}

bool RcReceiver::Is_Online(uint32_t timeout_ms) const noexcept {
    if (update_count_ == 0U) {
        return false;
    }
    return (HAL_GetTick() - last_update_tick_) < timeout_ms;
}

void RcReceiver::Get_Snapshot(RcSnapshot &out) const noexcept {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < kMaxChannels; ++i) {
        out.channels[i] = channels_[i];
    }
    out.raw_min = raw_lo_;
    out.raw_mid = raw_mid_;
    out.raw_max = raw_hi_;
    out.last_update_tick = last_update_tick_;
    out.update_count = update_count_;
    out.count = count_;
    out.ready = ready_;
    out.failsafe = failsafe_;
    out.frame_lost = frame_lost_;
    __set_PRIMASK(primask);
}

void RcReceiver::setRawRange(uint16_t lo, uint16_t mid, uint16_t hi) noexcept {
    raw_lo_  = lo;
    raw_mid_ = mid;
    raw_hi_  = hi;
}

void RcReceiver::markUpdated(const uint16_t *channels, uint8_t count,
                             bool failsafe, bool frame_lost) noexcept {
    if (channels == nullptr) {
        return;
    }
    if (count > kMaxChannels) {
        count = kMaxChannels;
    }
    for (uint8_t i = 0U; i < count; ++i) {
        channels_[i] = channels[i];
    }
    count_      = count;
    failsafe_   = failsafe;
    frame_lost_ = frame_lost;
    last_update_tick_ = HAL_GetTick();
    ++update_count_;
}
