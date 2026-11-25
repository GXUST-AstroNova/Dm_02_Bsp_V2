#pragma once
#include "dm_motor.hpp"

// Set each T_MAX to the torque range configured in that model's drive (N·m).
// Constructing a model whose T_MAX is unset is a compile-time error.
class Dm4310 final : public DmMotorModel<Dm4310> {
public:
    static constexpr float T_MAX = 7.0f;
    using DmMotorModel::DmMotorModel;
};

class Dm4340 final : public DmMotorModel<Dm4340> {
public:
    static constexpr float T_MAX = 0.0f;
    using DmMotorModel::DmMotorModel;
};

class Dm8009p final : public DmMotorModel<Dm8009p> {
public:
    static constexpr float T_MAX = 0.0f;
    using DmMotorModel::DmMotorModel;
};

class Dm10010 final : public DmMotorModel<Dm10010> {
public:
    static constexpr float T_MAX = 0.0f;
    using DmMotorModel::DmMotorModel;
};
