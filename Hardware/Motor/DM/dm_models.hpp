#pragma once
#include "dm_motor.hpp"

// Set each T_MAX to the torque range configured in that model's drive (N·m).
// Constructing a model whose T_MAX is unset is a compile-time error.
class Dm4310 final : public DmMotor {
public:
    static constexpr float T_MAX = 7.0f;

    template<float Max = T_MAX>
    explicit Dm4310(FDCAN_HandleTypeDef* bus) noexcept : DmMotor(bus, checkedTorqueMax<Max>()) {}

    template<float Max = T_MAX>
    Dm4310(FDCAN_HandleTypeDef* bus, DmCanId command_id, DmCanId feedback_id) noexcept
        : DmMotor(bus, checkedTorqueMax<Max>(), command_id, feedback_id) {}
};

class Dm4340 final : public DmMotor {
public:
    static constexpr float T_MAX = 28.0f;

    template<float Max = T_MAX>
    explicit Dm4340(FDCAN_HandleTypeDef* bus) noexcept : DmMotor(bus, checkedTorqueMax<Max>()) {}

    template<float Max = T_MAX>
    Dm4340(FDCAN_HandleTypeDef* bus, DmCanId command_id, DmCanId feedback_id) noexcept
        : DmMotor(bus, checkedTorqueMax<Max>(), command_id, feedback_id) {}
};

class Dm8009p final : public DmMotor {
public:
    static constexpr float T_MAX = 40.0f;

    template<float Max = T_MAX>
    explicit Dm8009p(FDCAN_HandleTypeDef* bus) noexcept : DmMotor(bus, checkedTorqueMax<Max>()) {}

    template<float Max = T_MAX>
    Dm8009p(FDCAN_HandleTypeDef* bus, DmCanId command_id, DmCanId feedback_id) noexcept
        : DmMotor(bus, checkedTorqueMax<Max>(), command_id, feedback_id) {}
};

class Dm10010 final : public DmMotor {
public:
    static constexpr float T_MAX = 0.0f;

    template<float Max = T_MAX>
    explicit Dm10010(FDCAN_HandleTypeDef* bus) noexcept : DmMotor(bus, checkedTorqueMax<Max>()) {}

    template<float Max = T_MAX>
    Dm10010(FDCAN_HandleTypeDef* bus, DmCanId command_id, DmCanId feedback_id) noexcept
        : DmMotor(bus, checkedTorqueMax<Max>(), command_id, feedback_id) {}
};
