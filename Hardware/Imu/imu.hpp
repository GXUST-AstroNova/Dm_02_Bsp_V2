#pragma once

#include <array>
#include <cstdint>
#include <numbers>

struct ImuData {
    std::array<float, 3> accel_m_s2{};
    std::array<float, 3> gyro_deg_s{};
    float temperature_c{};

    [[nodiscard]] std::array<float, 3> gyroRadS() const noexcept {
        constexpr float scale = std::numbers::pi_v<float> / 180.0f;
        return {gyro_deg_s[0] * scale, gyro_deg_s[1] * scale, gyro_deg_s[2] * scale};
    }
};

class Imu {
public:
    virtual ~Imu() = default;
    virtual int8_t Init() = 0;
    virtual int8_t Start() = 0;
    virtual int8_t Process() = 0;   // 1 = new complete sample, 0 = pending, -1 = error

    ImuData Get_Data() const noexcept;
    bool Is_Ready() const noexcept { return ready_; }
    bool Is_Online(uint32_t timeout_ms = 100) const noexcept;
    uint32_t Get_LastUpdateTick() const noexcept { return last_update_tick_; }
    uint32_t Get_UpdateCount() const noexcept { return update_count_; }
    uint32_t Get_RefreshErrors() const noexcept { return refresh_errors_; }
protected:
    void setReady(bool value) noexcept { ready_ = value; }
    void markUpdated(const ImuData& sample) noexcept;
    void markFailed() noexcept;
    ImuData data_{};
    uint32_t last_update_tick_{}, update_count_{}, refresh_errors_{};
    bool ready_{};
};
