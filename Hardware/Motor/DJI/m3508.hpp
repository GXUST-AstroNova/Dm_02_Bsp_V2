#pragma once

#include "motor_can.hpp"

class DjiMotorGroup {
public:
    static SendResult sendAll() noexcept;
    static bool dispatch(FDCAN_HandleTypeDef* bus, uint16_t feedback_id,
                         const motor_can::Payload& data) noexcept;
};

class M3508 final : public Motor {
public:
    // Rotor-side N·m/A, inferred from the P19 output specification and reduction.
    static constexpr float rotor_kt = 0.3f / (3591.0f / 187.0f);
    M3508(FDCAN_HandleTypeDef* bus, uint8_t id, float torque_constant = rotor_kt) noexcept;
    ~M3508() override;
    [[nodiscard]] bool init() noexcept;   // ID 1..8; joins the group at zero current
    bool setTorque(float torque_nm) noexcept override;
    bool disable() noexcept override;
    bool enable() noexcept;
    bool resetAccumulatedAngle() noexcept;
private:
    friend class DjiMotorGroup;
    void receive(const motor_can::Payload& data) noexcept;
    FDCAN_HandleTypeDef* const bus_;
    const uint8_t id_;
    const float kt_;
    int64_t total_counts_{};
    int last_raw_{};
    int16_t current_target_{};
    bool registered_{};
    bool enabled_{};
};
