#pragma once

#include <cstdint>

struct MotorFeedback {
    float single_turn_rad{};         // [-pi, pi)
    double multi_turn_rad{};         // includes the first encoder sample
    float velocity_rad_s{};
    float torque_nm{};
    bool valid{};
};

struct SendResult {
    unsigned attempted_frames{};
    unsigned queued_frames{};
};

class Motor {
public:
    virtual ~Motor() = default;
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    virtual bool setTorque(float torque_nm) noexcept = 0;
    virtual bool enable() noexcept = 0;
    virtual bool disable() noexcept = 0;
    [[nodiscard]] MotorFeedback feedback() const noexcept;
protected:
    Motor() = default;
    MotorFeedback feedback_{};
};
